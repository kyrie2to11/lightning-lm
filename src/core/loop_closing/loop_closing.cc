//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/debug_visualization.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <iomanip>
#include <sstream>

#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/cauchy.h"
#include "core/types/edge_se3.h"
#include "core/types/edge_se3_height_prior.h"
#include "core/types/vertex_se3.h"
#include "io/yaml_io.h"

namespace lightning {

namespace {

std::string PaddedBackendId(std::uint64_t id) {
    std::ostringstream stream;
    stream << std::setw(6) << std::setfill('0') << id;
    return stream.str();
}

std::string LoopEventName(std::uint64_t first, std::uint64_t second) {
    return "loop_" + PaddedBackendId(first) + "_" + PaddedBackendId(second);
}

std::string PoseCsv(const SE3& pose) {
    const Eigen::Quaterniond q(pose.so3().unit_quaternion());
    std::ostringstream row;
    row << std::setprecision(15)
        << pose.translation().x() << "," << pose.translation().y() << ","
        << pose.translation().z() << "," << q.w() << "," << q.x() << ","
        << q.y() << "," << q.z();
    return row.str();
}

std::string Matrix4Csv(const Mat4f& matrix) {
    std::ostringstream row;
    row << std::setprecision(15);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (r != 0 || c != 0) row << ",";
            row << matrix(r, c);
        }
    }
    return row.str();
}

std::string Matrix6Csv(const Mat6d& matrix) {
    std::ostringstream row;
    row << std::setprecision(15);
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            if (r != 0 || c != 0) row << ",";
            row << matrix(r, c);
        }
    }
    return row.str();
}

}  // namespace

LoopClosing::~LoopClosing() {
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

void LoopClosing::Init(const std::string yaml_path) {
    /// setup miao
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);

        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");

        const YAML::Node root = YAML::LoadFile(yaml_path);
        if (root["fasterlio"]["imu_frame_id"]) {
            imu_frame_id_ = root["fasterlio"]["imu_frame_id"].as<std::string>();
        }
        if (root["fasterlio"]["world_frame_id"]) {
            world_frame_id_ = root["fasterlio"]["world_frame_id"].as<std::string>();
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);

    // 检测回环候选
    DetectLoopCandidates();

    if (options_.verbose_) {
        LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
    }

    // 计算回环位姿
    ComputeLoopCandidates();

    // 位姿图优化
    PoseOptimization();

    last_kf_ = kf;
}

void LoopClosing::DetectLoopCandidates() {
    candidates_.clear();

    auto& kfs_mapping = all_keyframes_;
    Keyframe::Ptr check_first = nullptr;

    if (last_loop_kf_ == nullptr) {
        if (data_capture_ && data_capture_->enabled() &&
            data_capture_->params().capture_all_loop_candidates) {
            data_capture_->appendBackendRow(
                "candidate_detection", "candidates.csv",
                "current_keyframe,historical_keyframe,id_interval,distance_xy,accepted,reason",
                std::to_string(cur_kf_->GetID()) + ",-1,0,0,0,initialize_last_loop_keyframe");
        }
        last_loop_kf_ = cur_kf_;
        return;
    }

    if (last_loop_kf_ && (cur_kf_->GetID() - last_loop_kf_->GetID()) <= options_.loop_kf_gap_) {
        LOG(INFO) << "skip because last loop kf: " << last_loop_kf_->GetID();
        if (data_capture_ && data_capture_->enabled() &&
            data_capture_->params().capture_all_loop_candidates) {
            data_capture_->appendBackendRow(
                "candidate_detection", "candidates.csv",
                "current_keyframe,historical_keyframe,id_interval,distance_xy,accepted,reason",
                std::to_string(cur_kf_->GetID()) + "," + std::to_string(last_loop_kf_->GetID()) + "," +
                    std::to_string(cur_kf_->GetID() - last_loop_kf_->GetID()) +
                    ",0,0,loop_kf_gap");
        }
        return;
    }

    for (auto kf : kfs_mapping) {
        if (check_first != nullptr && abs(int(kf->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            // 同条轨迹内，跳过一定的ID区间
            if (data_capture_ && data_capture_->enabled() &&
                data_capture_->params().capture_all_loop_candidates) {
                data_capture_->appendBackendRow(
                    "candidate_detection", "candidates.csv",
                    "current_keyframe,historical_keyframe,id_interval,distance_xy,accepted,reason",
                    std::to_string(cur_kf_->GetID()) + "," + std::to_string(kf->GetID()) + "," +
                        std::to_string(std::abs(int(kf->GetID() - cur_kf_->GetID()))) +
                        ",0,0,min_candidate_interval");
            }
            continue;
        }

        if (abs(int(kf->GetID() - cur_kf_->GetID())) < options_.closest_id_th_) {
            /// 在同一条轨迹中，如果间隔太近，就不考虑回环
            if (data_capture_ && data_capture_->enabled() &&
                data_capture_->params().capture_all_loop_candidates) {
                data_capture_->appendBackendRow(
                    "candidate_detection", "candidates.csv",
                    "current_keyframe,historical_keyframe,id_interval,distance_xy,accepted,reason",
                    std::to_string(cur_kf_->GetID()) + "," + std::to_string(kf->GetID()) + "," +
                        std::to_string(std::abs(int(kf->GetID() - cur_kf_->GetID()))) +
                        ",0,0,closest_id_threshold");
            }
            break;
        }

        Vec3d dt = kf->GetOptPose().translation() - cur_kf_->GetOptPose().translation();
        double t2d = dt.head<2>().norm();  // x-y distance
        double range_th = options_.max_range_;

        if (t2d < range_th) {
            LoopCandidate c(kf->GetID(), cur_kf_->GetID());
            c.Tij_ = kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();

            candidates_.emplace_back(c);
            check_first = kf;
        }
        if (data_capture_ && data_capture_->enabled() &&
            data_capture_->params().capture_all_loop_candidates) {
            std::ostringstream row;
            row << cur_kf_->GetID() << "," << kf->GetID() << ","
                << std::abs(int(kf->GetID() - cur_kf_->GetID())) << ","
                << t2d << "," << (t2d < range_th ? 1 : 0) << ","
                << (t2d < range_th ? "within_range" : "outside_range");
            data_capture_->appendBackendRow(
                "candidate_detection", "candidates.csv",
                "current_keyframe,historical_keyframe,id_interval,distance_xy,accepted,reason",
                row.str());
        }
    }

    if (!candidates_.empty()) {
        last_loop_kf_ = cur_kf_;
    }

    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

void LoopClosing::ComputeLoopCandidates() {
    if (candidates_.empty()) {
        return;
    }

    // 执行计算
    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });
    // 保存成功的候选
    std::vector<LoopCandidate> succ_candidates;
    for (const auto& lc : candidates_) {
        // LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        if (lc.ndt_score_ > options_.ndt_score_th_) {
            succ_candidates.emplace_back(lc);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    candidates_.swap(succ_candidates);
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    // LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    const int submap_idx_range = 40;
    auto kf1 = all_keyframes_.at(c.idx1_), kf2 = all_keyframes_.at(c.idx2_);

    auto build_submap = [this](int given_id, bool build_in_world) -> CloudPtr {
        CloudPtr submap(new PointCloudType);
        for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
            int id = idx + given_id;
            if (id < 0 || id >= all_keyframes_.size()) {
                continue;
            }

            auto kf = all_keyframes_[id];
            CloudPtr cloud = kf->GetCloud();

            // RemoveGround(cloud, 0.1);

            if (cloud->empty()) {
                continue;
            }

            // 转到世界系下
            SE3 Twb = kf->GetOptPose();

            if (!build_in_world) {
                Twb = all_keyframes_.at(given_id)->GetOptPose().inverse() * Twb;
            }

            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());

            *submap += *cloud_trans;
        }
        return submap;
    };

    auto submap_kf1 = build_submap(kf1->GetID(), true);

    CloudPtr submap_kf2 = kf2->GetCloud();
    const std::string event = LoopEventName(c.idx1_, c.idx2_);
    const bool capture_event =
        data_capture_ && data_capture_->enabled() &&
        data_capture_->params().capture_all_loop_candidates;

    if (submap_kf1->empty() || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        if (capture_event) {
            data_capture_->appendBackendRow(
                event, "result.csv",
                "accepted,reason,score,threshold,constraint_tx,constraint_ty,constraint_tz,"
                "constraint_qw,constraint_qx,constraint_qy,constraint_qz",
                "0,empty_source_or_target,0," + std::to_string(options_.ndt_score_th_) +
                    ",0,0,0,1,0,0,0");
        }
        return;
    }

    Mat4f Tw2 = kf2->GetOptPose().matrix().cast<float>();
    if (debug_visualization_) {
        PointCloudType source_world_initial;
        pcl::transformPointCloud(*submap_kf2, source_world_initial, Tw2);
        debug_visualization_->publishCloud(
            "backend/ndt/source", source_world_initial, world_frame_id_,
            kf2->GetState().timestamp_, kf2->GetID(), true);
        debug_visualization_->publishCloud(
            "backend/ndt/target", *submap_kf1, world_frame_id_,
            kf2->GetState().timestamp_, kf2->GetID(), true);
    }
    if (capture_event) {
        PointCloudType source_world_initial;
        pcl::transformPointCloud(*submap_kf2, source_world_initial, Tw2);
        data_capture_->saveBackendCloud(event, "source_body", *submap_kf2, imu_frame_id_);
        data_capture_->saveBackendCloud(event, "source_world_initial", source_world_initial, world_frame_id_);
        data_capture_->saveBackendCloud(event, "target_submap_world", *submap_kf1, world_frame_id_);
        data_capture_->appendBackendRow(
            event, "submap.csv",
            "target_keyframe,source_keyframe,target_points,source_points,initial_transform_00,"
            "initial_transform_01,initial_transform_02,initial_transform_03,initial_transform_10,"
            "initial_transform_11,initial_transform_12,initial_transform_13,initial_transform_20,"
            "initial_transform_21,initial_transform_22,initial_transform_23,initial_transform_30,"
            "initial_transform_31,initial_transform_32,initial_transform_33",
            std::to_string(c.idx1_) + "," + std::to_string(c.idx2_) + "," +
                std::to_string(submap_kf1->size()) + "," + std::to_string(submap_kf2->size()) +
                "," + Matrix4Csv(Tw2));
    }

    /// 不同分辨率下的匹配
    CloudPtr output(new PointCloudType);
    std::vector<double> res{10.0, 5.0, 2.0, 1.0};

    CloudPtr rough_map1, rough_map2;

    for (auto& r : res) {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.05);
        ndt.setStepSize(0.7);
        ndt.setMaximumIterations(40);

        ndt.setResolution(r);
        rough_map1 = VoxelGrid(submap_kf1, r * 0.1);
        rough_map2 = VoxelGrid(submap_kf2, r * 0.1);
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        const Mat4f initial = Tw2;
        ndt.align(*output, initial);
        Tw2 = ndt.getFinalTransformation();

        c.ndt_score_ = ndt.getTransformationProbability();
        if (debug_visualization_) {
            debug_visualization_->publishCloud(
                "backend/ndt/aligned_resolution_" +
                    std::to_string(static_cast<int>(r)),
                *output, world_frame_id_, kf2->GetState().timestamp_, kf2->GetID(), true);
            debug_visualization_->publishMetrics(
                {{"ndt/resolution", r},
                 {"ndt/probability", c.ndt_score_},
                 {"ndt/iterations", static_cast<double>(ndt.getFinalNumIteration())},
                 {"ndt/converged", ndt.hasConverged() ? 1.0 : 0.0},
                 {"ndt/source_keyframe", static_cast<double>(c.idx2_)},
                 {"ndt/target_keyframe", static_cast<double>(c.idx1_)}},
                kf2->GetState().timestamp_, kf2->GetID(), true);
        }
        if (capture_event) {
            const std::string resolution = std::to_string(static_cast<int>(r));
            data_capture_->saveBackendCloud(
                event, "ndt_resolution_" + resolution + "_target_voxel_world",
                *rough_map1, world_frame_id_);
            data_capture_->saveBackendCloud(
                event, "ndt_resolution_" + resolution + "_source_voxel_body",
                *rough_map2, imu_frame_id_);
            data_capture_->saveBackendCloud(
                event, "ndt_resolution_" + resolution + "_aligned_world",
                *output, world_frame_id_);

            std::ostringstream row;
            row << r << "," << rough_map1->size() << "," << rough_map2->size() << ","
                << (ndt.hasConverged() ? 1 : 0) << "," << ndt.getFinalNumIteration() << ","
                << c.ndt_score_ << "," << Matrix4Csv(initial) << "," << Matrix4Csv(Tw2);
            std::string header =
                "resolution,target_voxel_points,source_voxel_points,converged,iterations,probability";
            for (const char* prefix : {"initial", "final"}) {
                for (int matrix_row = 0; matrix_row < 4; ++matrix_row) {
                    for (int matrix_col = 0; matrix_col < 4; ++matrix_col) {
                        header += "," + std::string(prefix) + "_" + std::to_string(matrix_row) +
                                  std::to_string(matrix_col);
                    }
                }
            }
            data_capture_->appendBackendRow(event, "ndt.csv", header, row.str());
        }
    }

    Mat4d T = Tw2.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);

    c.Tij_ = kf1->GetOptPose().inverse() * SE3(q, t);

    if (capture_event) {
        const Eigen::Quaterniond constraint_q(c.Tij_.so3().unit_quaternion());
        std::ostringstream row;
        row << (c.ndt_score_ > options_.ndt_score_th_ ? 1 : 0) << ","
            << (c.ndt_score_ > options_.ndt_score_th_ ? "score_above_threshold" : "score_below_threshold")
            << "," << c.ndt_score_ << "," << options_.ndt_score_th_ << ","
            << c.Tij_.translation().x() << "," << c.Tij_.translation().y() << ","
            << c.Tij_.translation().z() << ","
            << constraint_q.w() << "," << constraint_q.x() << ","
            << constraint_q.y() << "," << constraint_q.z();
        data_capture_->appendBackendRow(
            event, "result.csv",
            "accepted,reason,score,threshold,constraint_tx,constraint_ty,constraint_tz,"
            "constraint_qw,constraint_qx,constraint_qy,constraint_qz",
            row.str());
    }

    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_out.pcd", *output);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_tgt.pcd", *rough_map1);
}

void LoopClosing::PoseOptimization() {
    const bool capture_pgo =
        data_capture_ && data_capture_->enabled() &&
        data_capture_->params().capture_all_pgo_events && !candidates_.empty();
    const std::string pgo_event =
        capture_pgo ? "pgo_" + PaddedBackendId(pgo_event_id_++) : std::string();
    std::vector<SE3> poses_before;
    if (capture_pgo) {
        poses_before.reserve(all_keyframes_.size());
        for (const auto& keyframe : all_keyframes_) {
            const SE3 pose = keyframe->GetOptPose();
            poses_before.push_back(pose);
            data_capture_->appendBackendRow(
                pgo_event, "poses_before.csv",
                "keyframe_id,x,y,z,qw,qx,qy,qz",
                std::to_string(keyframe->GetID()) + "," + PoseCsv(pose));
        }
    }

    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(cur_kf_->GetID());
    v->SetEstimate(cur_kf_->GetOptPose());

    optimizer_->AddVertex(v);
    kf_vert_.emplace_back(v);

    /// 上一个关键帧的运动约束
    for (int i = 1; i < 3; i++) {
        int id = cur_kf_->GetID() - i;
        if (id >= 0) {
            auto last_kf = all_keyframes_[id];
            auto e = std::make_shared<miao::EdgeSE3>();
            e->SetVertex(0, optimizer_->GetVertex(last_kf->GetID()));
            e->SetVertex(1, v);

            SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
            e->SetMeasurement(motion);
            e->SetInformation(info_motion_);
            optimizer_->AddEdge(e);
            if (capture_pgo) {
                data_capture_->appendBackendRow(
                    pgo_event, "edges.csv",
                    "type,from_id,to_id,measurement_x,measurement_y,measurement_z,"
                    "measurement_qw,measurement_qx,measurement_qy,measurement_qz,"
                    "robust_kernel_delta,information_00,information_01,information_02,information_03,"
                    "information_04,information_05,information_10,information_11,information_12,"
                    "information_13,information_14,information_15,information_20,information_21,"
                    "information_22,information_23,information_24,information_25,information_30,"
                    "information_31,information_32,information_33,information_34,information_35,"
                    "information_40,information_41,information_42,information_43,information_44,"
                    "information_45,information_50,information_51,information_52,information_53,"
                    "information_54,information_55",
                    "motion," + std::to_string(last_kf->GetID()) + "," +
                        std::to_string(cur_kf_->GetID()) + "," + PoseCsv(motion) +
                        ",0," + Matrix6Csv(info_motion_));
            }
        }
    }

    if (options_.with_height_) {
        /// 高度约束
        auto e = std::make_shared<miao::EdgeHeightPrior>();
        e->SetVertex(0, v);
        e->SetMeasurement(0);
        e->SetInformation(Mat1d::Identity() * 1.0 / (options_.height_noise_ * options_.height_noise_));
        optimizer_->AddEdge(e);
        if (capture_pgo) {
            data_capture_->appendBackendRow(
                pgo_event, "height_priors.csv",
                "type,keyframe_id,measurement_z,information",
                "height," + std::to_string(cur_kf_->GetID()) + ",0," +
                    std::to_string(1.0 / (options_.height_noise_ * options_.height_noise_)));
        }
    }

    /// 回环的约束
    for (auto& c : candidates_) {
        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, optimizer_->GetVertex(c.idx1_));
        e->SetVertex(1, optimizer_->GetVertex(c.idx2_));
        e->SetMeasurement(c.Tij_);
        e->SetInformation(info_loops_);

        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);
        e->SetRobustKernel(rk);

        optimizer_->AddEdge(e);
        edge_loops_.emplace_back(e);
        if (capture_pgo) {
            data_capture_->appendBackendRow(
                pgo_event, "edges.csv",
                "type,from_id,to_id,measurement_x,measurement_y,measurement_z,"
                "measurement_qw,measurement_qx,measurement_qy,measurement_qz,"
                "robust_kernel_delta,information_00,information_01,information_02,information_03,"
                "information_04,information_05,information_10,information_11,information_12,"
                "information_13,information_14,information_15,information_20,information_21,"
                "information_22,information_23,information_24,information_25,information_30,"
                "information_31,information_32,information_33,information_34,information_35,"
                "information_40,information_41,information_42,information_43,information_44,"
                "information_45,information_50,information_51,information_52,information_53,"
                "information_54,information_55",
                "loop," + std::to_string(c.idx1_) + "," + std::to_string(c.idx2_) + "," +
                    PoseCsv(c.Tij_) + "," + std::to_string(options_.rk_loop_th_) + "," +
                    Matrix6Csv(info_loops_));
        }
    }

    if (optimizer_->GetEdges().empty()) {
        return;
    }

    if (candidates_.empty()) {
        return;
    }

    optimizer_->InitializeOptimization();
    optimizer_->SetVerbose(false);

    optimizer_->Optimize(20);

    /// remove outliers
    int cnt_outliers = 0;
    for (auto& e : edge_loops_) {
        if (e->GetRobustKernel() == nullptr) {
            continue;
        }

        if (e->Chi2() > e->GetRobustKernel()->Delta()) {
            e->SetLevel(1);
            cnt_outliers++;
        } else {
            e->SetRobustKernel(nullptr);
        }
        if (capture_pgo) {
            data_capture_->appendBackendRow(
                pgo_event, "loop_edges_after.csv",
                "from_id,to_id,chi2,level,outlier",
                std::to_string(e->GetVertex(0)->GetId()) + "," +
                    std::to_string(e->GetVertex(1)->GetId()) + "," +
                    std::to_string(e->Chi2()) + "," + std::to_string(e->Level()) + "," +
                    std::to_string(e->Level() > 0 ? 1 : 0));
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "loop outliers: " << cnt_outliers << "/" << edge_loops_.size();
    }

    /// get results
    for (auto& vert : kf_vert_) {
        SE3 pose = vert->Estimate();
        all_keyframes_[vert->GetId()]->SetOptPose(pose);
    }

    if (capture_pgo) {
        for (const auto& keyframe : all_keyframes_) {
            const SE3 pose_after = keyframe->GetOptPose();
            const SE3 correction = poses_before.at(keyframe->GetID()).inverse() * pose_after;
            data_capture_->appendBackendRow(
                pgo_event, "poses_after.csv",
                "keyframe_id,x,y,z,qw,qx,qy,qz,"
                "correction_x,correction_y,correction_z,"
                "correction_qw,correction_qx,correction_qy,correction_qz",
                std::to_string(keyframe->GetID()) + "," + PoseCsv(pose_after) + "," +
                    PoseCsv(correction));
        }
    }

    if (debug_visualization_) {
        std::vector<SE3> lio_poses;
        std::vector<SE3> optimized_poses;
        lio_poses.reserve(all_keyframes_.size());
        optimized_poses.reserve(all_keyframes_.size());
        double max_translation_correction = 0.0;
        double max_rotation_correction = 0.0;
        for (const auto& keyframe : all_keyframes_) {
            lio_poses.push_back(keyframe->GetLIOPose());
            optimized_poses.push_back(keyframe->GetOptPose());
            const SE3 correction = keyframe->GetLIOPose().inverse() * keyframe->GetOptPose();
            max_translation_correction =
                std::max(max_translation_correction, correction.translation().norm());
            max_rotation_correction = std::max(
                max_rotation_correction, correction.so3().log().norm() * 180.0 / M_PI);
        }
        const auto frame_id = static_cast<std::uint64_t>(cur_kf_->GetID());
        const double timestamp = cur_kf_->GetState().timestamp_;
        debug_visualization_->publishPath(
            "path/lio", lio_poses, world_frame_id_, timestamp, frame_id, true);
        debug_visualization_->publishPath(
            "path/optimized", optimized_poses, world_frame_id_, timestamp, frame_id, true);
        debug_visualization_->publishMetrics(
            {{"pgo/loop_candidates", static_cast<double>(candidates_.size())},
             {"pgo/loop_outliers", static_cast<double>(cnt_outliers)},
             {"pgo/max_translation_correction", max_translation_correction},
             {"pgo/max_rotation_correction_deg", max_rotation_correction}},
            timestamp, frame_id, true);
    }

    if (loop_cb_) {
        loop_cb_();
    }

    LOG(INFO) << "optimize finished, loops: " << edge_loops_.size();

    // LOG(INFO) << "lc: cur kf " << cur_kf_->GetID() << ", opt: " << cur_kf_->GetOptPose().translation().transpose()
    //           << ", lio: " << cur_kf_->GetLIOPose().translation().transpose();
}

}  // namespace lightning
