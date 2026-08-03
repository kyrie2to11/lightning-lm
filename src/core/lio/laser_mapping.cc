#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <tf2_eigen/tf2_eigen.hpp>

#include "common/options.h"
#include "common/debug_visualization.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

namespace lightning {

namespace {

CloudPtr ConvertRosXyziForCapture(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::fromROSMsg(*msg, input);

    CloudPtr output(new PointCloudType());
    output->reserve(input.size());
    for (const auto& source : input) {
        PointType point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.intensity = source.intensity;
        point.time = 0.0;
        output->push_back(point);
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = input.is_dense;
    return output;
}

std::string StateCsv(const NavState& state) {
    const Eigen::Quaterniond q(state.rot_.unit_quaternion());
    std::ostringstream row;
    row << std::setprecision(15)
        << state.timestamp_ << ","
        << state.pos_.x() << "," << state.pos_.y() << "," << state.pos_.z() << ","
        << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
        << state.vel_.x() << "," << state.vel_.y() << "," << state.vel_.z() << ","
        << state.bg_.x() << "," << state.bg_.y() << "," << state.bg_.z() << ","
        << state.grav_.x() << "," << state.grav_.y() << "," << state.grav_.z();
    return row.str();
}

constexpr const char* kStateCsvHeader =
    "timestamp,pos_x,pos_y,pos_z,qw,qx,qy,qz,vel_x,vel_y,vel_z,"
    "bg_x,bg_y,bg_z,grav_x,grav_y,grav_z";

std::string PrefixedStateCsvHeader(const std::string& prefix) {
    std::stringstream input(kStateCsvHeader);
    std::ostringstream output;
    std::string column;
    bool first = true;
    while (std::getline(input, column, ',')) {
        if (!first) output << ",";
        output << prefix << column;
        first = false;
    }
    return output.str();
}

}  // namespace

bool LaserMapping::Init(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, ESKF::state_dim_, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState &s, ESKF::CustomObservationModel &obs) { ObsModel(s, obs); };
    eskf_options.iteration_callback_ = [this](const ESKF::IterationInfo& info) {
        CaptureEskfIteration(info);
    };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

void LaserMapping::SetExtrinsic(const Vec3d &translation, const Mat3d &rotation) {
    offset_t_lidar_fixed_ = translation;
    offset_R_lidar_fixed_ = rotation;

    extrinT_ = {translation.x(), translation.y(), translation.z()};
    extrinR_ = {rotation(0, 0), rotation(0, 1), rotation(0, 2),
                rotation(1, 0), rotation(1, 1), rotation(1, 2),
                rotation(2, 0), rotation(2, 1), rotation(2, 2)};

    p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
}

void LaserMapping::SetInitialWorldImuRotation(const Mat3d &R_world_imu) {
    p_imu_->SetInitialWorldImuRotation(R_world_imu);
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();

        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["fasterlio"]["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["fasterlio"]["ivox_nearby_type"].as<int>();
        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

        float height_max = yaml["roi"]["height_max"].as<float>();
        float height_min = yaml["roi"]["height_min"].as<float>();

        preprocess_->SetHeightROI(height_max, height_min);

        if (yaml["fasterlio"]["lidar_frame_id"]) {
            lidar_frame_id_ = yaml["fasterlio"]["lidar_frame_id"].as<std::string>();
            preprocess_->SetExpectedFrame(lidar_frame_id_);
        }
        if (yaml["fasterlio"]["imu_frame_id"]) {
            imu_frame_id_ = yaml["fasterlio"]["imu_frame_id"].as<std::string>();
        }
        if (yaml["fasterlio"]["world_frame_id"]) {
            world_frame_id_ = yaml["fasterlio"]["world_frame_id"].as<std::string>();
        }

        options_.kf_dis_th_ = yaml["fasterlio"]["kf_dis_th"].as<double>();
        options_.kf_angle_th_ = yaml["fasterlio"]["kf_angle_th"].as<double>() * M_PI / 180.0;
        options_.enable_icp_part_ = yaml["fasterlio"]["enable_icp_part"].as<bool>();
        options_.min_pts = yaml["fasterlio"]["min_pts"].as<int>();
        if (yaml["fasterlio"]["max_observation_points"]) {
            options_.max_observation_points = yaml["fasterlio"]["max_observation_points"].as<int>();
        }
        options_.plane_icp_weight_ = yaml["fasterlio"]["plane_icp_weight"].as<float>();

        bool use_imu_filter = yaml["fasterlio"]["imu_filter"].as<bool>();
        p_imu_->SetUseIMUFilter(use_imu_filter);
        options_.proj_kfs_ = yaml["fasterlio"]["proj_kfs"].as<bool>();

    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        LOG(INFO) << "Using RoboSense Lidar";
    } else if (lidar_type == 5) {
        preprocess_->SetLidarType(LidarType::POLKA_MERGED);
        LOG(INFO) << "Using pre-deskewed Polka merged cloud";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    offset_t_lidar_fixed_ = math::VecFromArray<double>(extrinT_);
    offset_R_lidar_fixed_ = math::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    publish_count_++;

    double timestamp = imu->timestamp;

    UL lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    if (p_imu_->IsIMUInited()) {
        /// 更新最新imu状态
        kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);

        /// 更新ui
        if (ui_) {
            ui_->UpdateNavState(kf_imu_.GetX());
        }
    }

    last_timestamp_imu_ = timestamp;

    // Data capture: IMU CSV
    if (data_capture_.enabled()) {
        data_capture_.appendImu(timestamp,
                                imu->linear_acceleration.x(), imu->linear_acceleration.y(), imu->linear_acceleration.z(),
                                imu->angular_velocity.x(), imu->angular_velocity.y(), imu->angular_velocity.z());
    }

    imu_buffer_.emplace_back(imu);
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu, int imu_id) {
    UL lock(mtx_buffer_);

    double timestamp = imu->timestamp;
    if (timestamp < mb_last_imu_time_[imu_id]) {
        LOG(WARNING) << "mb imu " << imu_id << " out-of-order — accepting";
    }
    mb_last_imu_time_[imu_id] = timestamp;
    mb_imu_buffers_[imu_id].emplace_back(imu);
    // Sort by timestamp to handle MCAP out-of-order delivery
    std::sort(mb_imu_buffers_[imu_id].begin(),
              mb_imu_buffers_[imu_id].end(),
              [](const IMUPtr& a, const IMUPtr& b) { return a->timestamp < b->timestamp; });

    // For the leader IMU, also mirror into the legacy path (kf_imu_ predict)
    const auto* imu_cfg = multibody_cfg_.findImuByBody(multibody_cfg_.leader_body_id);
    if (imu_cfg && imu_id == imu_cfg->id) {
        if (p_imu_->IsIMUInited()) {
            kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_,
                            imu->angular_velocity, imu->linear_acceleration);
            if (ui_) ui_->UpdateNavState(kf_imu_.GetX());
        }
        last_timestamp_imu_ = timestamp;
    }
}

void LaserMapping::ProcessJointStates(double timestamp, double angle) {
    UL lock(mtx_buffer_);
    joint_state_buffer_.addSample(timestamp, angle);
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, int lidar_id) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < mb_last_lidar_time_[lidar_id]) {
                LOG(WARNING) << "mb lidar " << lidar_id << " out-of-order, dt: "
                             << timestamp - mb_last_lidar_time_[lidar_id] << " — accepting and sorting buffer";
            }

            LOG(INFO) << "get cloud lidar=" << lidar_id << " at " << std::setprecision(14) << timestamp;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            LidarEntry entry;
            entry.cloud = cloud;
            entry.begin_time = timestamp;
            entry.end_time = ComputeLidarEndTime(timestamp, *cloud, lidar_mean_scantime_,
                                                  preprocess_->InputIsPredeskewed());
            mb_lidar_buffers_[lidar_id].push_back(entry);
            // Sort by begin_time to handle MCAP out-of-order delivery
            std::sort(mb_lidar_buffers_[lidar_id].begin(),
                      mb_lidar_buffers_[lidar_id].end(),
                      [](const LidarEntry& a, const LidarEntry& b) { return a.begin_time < b.begin_time; });
            mb_last_lidar_time_[lidar_id] = timestamp;
        },
        "Preprocess (MultiBody)");
}

bool LaserMapping::Run() {
    // ---- Sync ----
    NavState leader_seed_state;
    double seed_time = 0;

    if (multibody_cfg_.enabled) {
        if (!SyncPackagesMultiBody()) {
            return false;
        }

        // Populate measures_ from mb_measures_ for the leader body
        const auto* leader_lc = multibody_cfg_.findLeaderLidar();
        if (!leader_lc) {
            LOG(ERROR) << "no leader lidar configured";
            return false;
        }
        leader_lidar_id_ = leader_lc->id;

        auto lit = mb_measures_.lidars.find(leader_lidar_id_);
        if (lit == mb_measures_.lidars.end()) {
            LOG(ERROR) << "leader lidar not in synced measures";
            return false;
        }

        measures_.scan_ = lit->second.cloud;
        // Use the LEADER's own begin/end time — NOT the global min/max.
        // The global times include non-leader lidars which may have different
        // timestamps. Using global times would give the leader's deskew
        // an incorrect time range, causing IMU propagation errors.
        measures_.lidar_begin_time_ = lit->second.begin_time;
        measures_.lidar_end_time_ = lit->second.end_time;
        lidar_end_time_ = measures_.lidar_end_time_;

        // Leader IMU window
        const auto* leader_imu_cfg = multibody_cfg_.findImuByBody(multibody_cfg_.leader_body_id);
        if (!leader_imu_cfg) {
            LOG(ERROR) << "no leader IMU configured";
            return false;
        }
        auto iit = mb_measures_.imus.find(leader_imu_cfg->id);
        if (iit == mb_measures_.imus.end() || iit->second.empty()) {
            LOG(INFO) << "leader IMU window empty, skipping";
            return false;
        }
        measures_.imu_ = iit->second;

        // Save leader state before Process for non-leader seeding
        leader_seed_state = kf_.GetX();
        seed_time = prev_lidar_end_time_;

    } else {
        if (!SyncPackages()) {
            LOG(WARNING) << "sync package failed";
            return false;
        }
    }

    debug_frame_id_ = next_debug_frame_id_++;

    if (debug_visualization_) {
        debug_visualization_->publishCloud(
            "frontend/input", *current_input_cloud_, lidar_frame_id_,
            measures_.lidar_end_time_, debug_frame_id_);
        debug_visualization_->publishCloud(
            "frontend/preprocessed", *measures_.scan_, lidar_frame_id_,
            measures_.lidar_end_time_, debug_frame_id_);
        debug_visualization_->publishMetrics(
            {{"frontend/input_points", static_cast<double>(current_preprocess_stats_.input_points)},
             {"frontend/preprocessed_points", static_cast<double>(measures_.scan_->size())},
             {"sync/imu_count", static_cast<double>(measures_.imu_.size())}},
            measures_.lidar_end_time_, debug_frame_id_);
    }

    if (data_capture_.enabled() && !multibody_cfg_.enabled) {
        capture_frame_ =
            data_capture_.beginProcessedFrame(measures_.lidar_begin_time_, measures_.lidar_end_time_);
        capture_frame_valid_ = true;

        data_capture_.saveFrontendCloud(
            capture_frame_, "00_lightning_input_from_polka", *current_input_cloud_, lidar_frame_id_);
        data_capture_.saveFrontendCloud(
            capture_frame_, "01_preprocessed_lidar", *measures_.scan_, lidar_frame_id_);
        std::ostringstream preprocess_row;
        preprocess_row << current_preprocess_stats_.input_points << ","
                       << current_preprocess_stats_.stride_rejected << ","
                       << current_preprocess_stats_.range_rejected << ","
                       << current_preprocess_stats_.height_rejected << ","
                       << current_preprocess_stats_.output_points;
        data_capture_.appendFrameRow(
            capture_frame_, "preprocess.csv",
            "input_points,stride_rejected,range_rejected,height_rejected,output_points",
            preprocess_row.str());

        std::ostringstream sync_row;
        const double first_imu = measures_.imu_.empty() ? 0.0 : measures_.imu_.front()->timestamp;
        const double last_imu = measures_.imu_.empty() ? 0.0 : measures_.imu_.back()->timestamp;
        sync_row << std::setprecision(15)
                 << measures_.lidar_begin_time_ << "," << measures_.lidar_end_time_ << ","
                 << first_imu << "," << last_imu << "," << measures_.imu_.size() << ","
                 << lidar_buffer_.size() << "," << imu_buffer_.size();
        data_capture_.appendFrameRow(
            capture_frame_, "synchronization.csv",
            "lidar_begin_time,lidar_end_time,imu_begin_time,imu_end_time,imu_count,"
            "remaining_lidar_buffer,remaining_imu_buffer",
            sync_row.str());
    } else {
        capture_frame_valid_ = false;
    }

    /// IMU process, kf prediction, undistortion
    const NavState state_before_imu = kf_.GetX();
    const ESKF::CovType covariance_before_imu = kf_.GetP();
    p_imu_->Process(measures_, kf_, scan_undistort_, !preprocess_->InputIsPredeskewed());

    if (debug_visualization_ && scan_undistort_) {
        debug_visualization_->publishCloud(
            "frontend/imu_body", *scan_undistort_, imu_frame_id_,
            measures_.lidar_end_time_, debug_frame_id_);
        const auto& predicted = kf_.GetX();
        auto predicted_metrics = DebugVisualization::AttitudeMetrics(
            "predicted", predicted.rot_.matrix(), offset_R_lidar_fixed_);
        predicted_metrics.insert(
            predicted_metrics.end(),
            {{"state/predicted/px", predicted.pos_.x()},
             {"state/predicted/py", predicted.pos_.y()},
             {"state/predicted/pz", predicted.pos_.z()},
             {"state/predicted/vx", predicted.vel_.x()},
             {"state/predicted/vy", predicted.vel_.y()},
             {"state/predicted/vz", predicted.vel_.z()},
             {"state/predicted/bg_x", predicted.bg_.x()},
             {"state/predicted/bg_y", predicted.bg_.y()},
             {"state/predicted/bg_z", predicted.bg_.z()}});
        debug_visualization_->publishMetrics(
            predicted_metrics,
            measures_.lidar_end_time_, debug_frame_id_);
    }

    if (capture_frame_valid_) {
        const auto append_state = [this](const std::string& phase, const NavState& state,
                                         const ESKF::CovType& covariance) {
            std::ostringstream row;
            row << phase << "," << StateCsv(state);
            for (int i = 0; i < ESKF::state_dim_; ++i) {
                row << "," << covariance(i, i);
            }
            std::string header = std::string("phase,") + kStateCsvHeader;
            for (int i = 0; i < ESKF::state_dim_; ++i) {
                header += ",cov_diag_" + std::to_string(i);
            }
            data_capture_.appendFrameRow(capture_frame_, "imu_prediction.csv", header, row.str());
        };
        append_state("before_imu_process", state_before_imu, covariance_before_imu);
        append_state("after_imu_process", kf_.GetX(), kf_.GetP());

        const auto init = p_imu_->GetInitializationSnapshot();
        const NavState init_state = kf_.GetX();
        std::ostringstream init_row;
        init_row << capture_frame_.id << "," << init.sample_count << ","
                 << (init.initialized ? 1 : 0) << ","
                 << init.mean_acc.x() << "," << init.mean_acc.y() << "," << init.mean_acc.z() << ","
                 << init.mean_gyr.x() << "," << init.mean_gyr.y() << "," << init.mean_gyr.z() << ","
                 << init.cov_acc.x() << "," << init.cov_acc.y() << "," << init.cov_acc.z() << ","
                 << init.cov_gyr.x() << "," << init.cov_gyr.y() << "," << init.cov_gyr.z() << ","
                 << init.acceleration_scale << ","
                 << init_state.bg_.x() << "," << init_state.bg_.y() << "," << init_state.bg_.z() << ","
                 << init_state.grav_.x() << "," << init_state.grav_.y() << "," << init_state.grav_.z();
        data_capture_.appendGlobalRow(
            "imu_initialization.csv",
            "processed_frame_id,sample_count,initialized,mean_ax,mean_ay,mean_az,"
            "mean_gx,mean_gy,mean_gz,cov_ax,cov_ay,cov_az,cov_gx,cov_gy,cov_gz,"
            "acceleration_scale,bg_x,bg_y,bg_z,grav_x,grav_y,grav_z",
            init_row.str());

        std::ostringstream mode_row;
        mode_row << "polka_predeskewed," << (preprocess_->InputIsPredeskewed() ? 0 : 1) << ","
                 << lidar_frame_id_ << "," << imu_frame_id_ << ",imu_from_lidar";
        data_capture_.appendFrameRow(
            capture_frame_, "imu_process_mode.csv",
            "input_mode,pointwise_deskew,lidar_frame,output_frame,extrinsic_direction",
            mode_row.str());

        if (scan_undistort_) {
            data_capture_.saveFrontendCloud(
                capture_frame_, "02_imu_body_cloud", *scan_undistort_, imu_frame_id_);
        }
    }

    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    /// the first scan
    if (flg_first_scan_) {
        LOG(INFO) << "first scan pts: " << scan_undistort_->size();

        state_point_ = kf_.GetX();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);
        if (capture_frame_valid_) {
            data_capture_.saveFrontendCloud(
                capture_frame_, "03_first_scan_world", *scan_down_world_, world_frame_id_);
            data_capture_.saveFrontendCloud(
                capture_frame_, "04_ivox_initial_points", *scan_down_world_, world_frame_id_);
            data_capture_.appendFrameRow(
                capture_frame_, "map_events.csv", "event,points,ivox_valid_grids",
                "first_scan_ivox_initialization," + std::to_string(scan_down_world_->size()) + "," +
                    std::to_string(ivox_->NumValidGrids()));
        }

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        flg_first_scan_ = false;
        prev_lidar_end_time_ = measures_.lidar_end_time_;
        return true;
    }

    // ---- Multi-body: non-leader deskew + cross-body transform + merge ----
    if (multibody_cfg_.enabled && p_imu_->IsIMUInited()) {
        ProcessNonLeaderBodies(leader_seed_state, seed_time);
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        if (skip_lidar_cnt_ != 0) {
            /// 更新UI中的内容
            if (ui_) {
                ui_->UpdateNavState(kf_.GetX());
                ui_->UpdateScan(scan_undistort_, kf_.GetX().GetPose());
            }

            return false;
        }
    }

    LOG(INFO) << "=============================";
    LOG(INFO) << "LIO get cloud at beg: " << std::setprecision(14) << measures_.lidar_begin_time_
              << ", end: " << measures_.lidar_end_time_;

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LOG(ERROR) << "检测到雷达断流，时长：" << (measures_.lidar_begin_time_ - last_lidar_time_);
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    /// downsample
    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    // if (options_.proj_kfs_) {
    //     ProjectKFs();
    // }

    int cur_pts = scan_down_body_->size();

    if (cur_pts < (scan_undistort_->size() * 0.1) || cur_pts < options_.min_pts) {
        /// 降采样太狠了,有效点数不够，用0.1分辨率代替
        // LOG(INFO) << "too few points, using 0.1 resol";
        auto v = voxel_scan_;
        v.setLeafSize(0.1, 0.1, 0.1);
        v.setInputCloud(scan_undistort_);
        v.filter(*scan_down_body_);

        // LOG(INFO) << "Now pts: " << scan_down_body_->size() << ", before: " << cur_pts;
        cur_pts = scan_down_body_->size();
    }

    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }

    if (capture_frame_valid_) {
        data_capture_.saveFrontendCloud(
            capture_frame_, "03_voxel_pre_limit", *scan_down_body_, imu_frame_id_);
    }
    if (debug_visualization_) {
        debug_visualization_->publishCloud(
            "frontend/voxel_pre_limit", *scan_down_body_, imu_frame_id_,
            measures_.lidar_end_time_, debug_frame_id_);
    }

    // Limit observation points (colleague's max_observation_points)
    if (options_.max_observation_points > 0 && cur_pts > options_.max_observation_points) {
        auto& pts = scan_down_body_->points;
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(pts.begin(), pts.end(), g);
        pts.resize(options_.max_observation_points);
        scan_down_body_->width = options_.max_observation_points;
        cur_pts = options_.max_observation_points;
    }

    if (capture_frame_valid_) {
        data_capture_.saveFrontendCloud(
            capture_frame_, "04_observation_input", *scan_down_body_, imu_frame_id_);
        std::ostringstream row;
        row << scan_undistort_->size() << "," << cur_pts << ","
            << options_.max_observation_points;
        data_capture_.appendFrameRow(
            capture_frame_, "downsampling.csv",
            "imu_body_points,observation_points,max_observation_points", row.str());
    }
    if (debug_visualization_) {
        debug_visualization_->publishCloud(
            "frontend/observation_input", *scan_down_body_, imu_frame_id_,
            measures_.lidar_end_time_, debug_frame_id_);
    }

    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);

    // 成员变量预分配
    residuals_.resize(cur_pts, 0);
    std::fill(residuals_.begin(), residuals_.end(), 0.0F);
    point_selected_surf_.resize(cur_pts, 1);
    point_selected_icp_.resize(cur_pts, 1);
    plane_coef_.resize(cur_pts, Vec4f::Zero());
    capture_rejection_reason_.resize(cur_pts, 0);

    auto pred_state = kf_.GetX();
    // pred_state.pos_ = state_point_.pos_;  // 假定位置不动行不行,防止速度漂移
    // kf_.ChangeX(pred_state);

    kf_.Update(ESKF::ObsType::LIDAR, 1.0);

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;

    if (debug_visualization_ && debug_visualization_->cloudEnabled(debug_frame_id_)) {
        PointCloudType final_world;
        pcl::transformPointCloud(*scan_down_body_, final_world, state_point_.GetPose().matrix());
        debug_visualization_->publishCloud(
            "frontend/state_updated", final_world, world_frame_id_,
            measures_.lidar_end_time_, debug_frame_id_);
    }
    if (debug_visualization_) {
        auto updated_metrics = DebugVisualization::AttitudeMetrics(
            "updated", state_point_.rot_.matrix(), offset_R_lidar_fixed_);
        updated_metrics.insert(
            updated_metrics.end(),
            {{"state/updated/px", state_point_.pos_.x()},
             {"state/updated/py", state_point_.pos_.y()},
             {"state/updated/pz", state_point_.pos_.z()},
             {"state/updated/vx", state_point_.vel_.x()},
             {"state/updated/vy", state_point_.vel_.y()},
             {"state/updated/vz", state_point_.vel_.z()},
             {"frontend/iterations", static_cast<double>(kf_.GetIterations())},
             {"frontend/final_residual_ratio", kf_.GetFinalRes()},
             {"frontend/surface_matches", static_cast<double>(effect_feat_surf_)},
             {"frontend/icp_matches", static_cast<double>(effect_feat_icp_)}});
        debug_visualization_->publishMetrics(
            updated_metrics,
            measures_.lidar_end_time_, debug_frame_id_);
    }

    if (capture_frame_valid_) {
        PointCloudType final_world;
        pcl::transformPointCloud(
            *scan_down_body_, final_world, state_point_.GetPose().matrix());
        data_capture_.saveFrontendCloud(
            capture_frame_, "05_state_updated_world", final_world, world_frame_id_);

        std::ostringstream row;
        row << StateCsv(pred_state) << "," << StateCsv(state_point_) << ","
            << kf_.GetIterations() << "," << kf_.GetFinalRes() << ","
            << effect_feat_surf_ << "," << effect_feat_icp_;
        data_capture_.appendFrameRow(
            capture_frame_, "frontend_result.csv",
            PrefixedStateCsvHeader("pred_") + "," + PrefixedStateCsvHeader("updated_") +
                ",iterations,final_residual_ratio,surface_matches,icp_matches",
            row.str());
    }

    const double delta_translation = (pred_state.pos_ - state_point_.pos_).norm();
    const double delta_rotation_deg = (pred_state.rot_.inverse() * state_point_.rot_).log().norm() * 180.0 / M_PI;
    const double delta_velocity = (pred_state.vel_ - state_point_.vel_).norm();

    const double current_speed = state_point_.vel_.norm();

    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_surf_ << ", "
              << effect_feat_icp_;
    LOG(INFO) << "delta trans: " << (pred_state.pos_ - state_point_.pos_).transpose()
              << ", ang: " << delta_rotation_deg;
    // LOG(INFO) << "P diag: " << kf_.GetP().diagonal().transpose();

    // Vec3d v_from_last = (state_point_.pos_ - last_state.pos_) / (state_point_.timestamp_ - last_state.timestamp_);
    // LOG(INFO) << "v from last: " << v_from_last.transpose();

    // if (delta_velocity > 1.0 || current_speed > 4.0) {
    //     LOG(ERROR) << "detected very large vel change, last: " << last_state.vel_.transpose()
    //                << ", pred: " << pred_state.vel_.transpose() << ", cur:" << state_point_.vel_.transpose();
    //     LOG(ERROR) << "please check";
    // }

    /// keyframes
    if (last_kf_ == nullptr) {
        if (capture_frame_valid_) {
            data_capture_.appendGlobalRow(
                "keyframe_decisions.csv",
                "processed_frame_id,timestamp,translation,rotation_deg,translation_threshold,"
                "rotation_threshold_deg,is_keyframe,reason",
                std::to_string(capture_frame_.id) + "," + std::to_string(state_point_.timestamp_) +
                    ",0,0," + std::to_string(options_.kf_dis_th_) + "," +
                    std::to_string(options_.kf_angle_th_ * 180.0 / M_PI) + ",1,first_keyframe");
        }
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = state_point_.GetPose();
        const double keyframe_translation = (last_pose.translation() - cur_pose.translation()).norm();
        const double keyframe_rotation =
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm();
        const bool motion_keyframe = keyframe_translation > options_.kf_dis_th_ ||
                                     keyframe_rotation > options_.kf_angle_th_;
        const bool localization_timeout_keyframe =
            !options_.is_in_slam_mode_ &&
            (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0;
        if (capture_frame_valid_) {
            const bool is_keyframe = motion_keyframe || localization_timeout_keyframe;
            const std::string reason = motion_keyframe
                                           ? "motion_threshold"
                                           : (localization_timeout_keyframe ? "localization_timeout" : "below_threshold");
            std::ostringstream row;
            row << capture_frame_.id << "," << std::setprecision(15) << state_point_.timestamp_ << ","
                << keyframe_translation << "," << keyframe_rotation * 180.0 / M_PI << ","
                << options_.kf_dis_th_ << "," << options_.kf_angle_th_ * 180.0 / M_PI << ","
                << (is_keyframe ? 1 : 0) << "," << reason;
            data_capture_.appendGlobalRow(
                "keyframe_decisions.csv",
                "processed_frame_id,timestamp,translation,rotation_deg,translation_threshold,"
                "rotation_threshold_deg,is_keyframe,reason",
                row.str());
        }
        if (motion_keyframe) {
            MakeKF();
        } else if (localization_timeout_keyframe) {
            MakeKF();
        } else if ((last_pose.so3().inverse() * cur_pose.so3()).log().norm() > 1.0 * M_PI / 180.0) {
            // MapIncremental();
        }
    }

    /// 更新kf_for_imu
    kf_imu_ = kf_;
    if (!measures_.imu_.empty()) {
        double t = measures_.imu_.back()->timestamp;
        for (auto &imu : imu_buffer_) {
            double dt = imu->timestamp - t;
            kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);
            t = imu->timestamp;
        }
    }

    if (ui_) {
        ui_->UpdateScan(scan_down_body_, state_point_.GetPose());
    }

    const Mat3d R_w_imu = state_point_.rot_.matrix();
    LOG(INFO) << "LIO state: " << state_point_.pos_.transpose()
              << ", yawZ " << state_point_.rot_.angleZ<double>() * 180 / M_PI
              << ", pitchY " << asin(std::clamp(-R_w_imu(2, 0), -1.0, 1.0)) * 180 / M_PI
              << ", rollX " << atan2(R_w_imu(2, 1), R_w_imu(2, 2)) * 180 / M_PI
              << ", vel: " << state_point_.vel_.transpose()
              << ", grav: " << state_point_.grav_.transpose() << ", grav norm: " << state_point_.grav_.norm();

    // Debug: print R_world_imu columns every 50 frames to verify axes
    if (scan_count_ % 50 == 0) {
        LOG(INFO) << "[DBG R_wi] col0(IMU_X)=" << R_w_imu.col(0).transpose()
                  << " col1(IMU_Y)=" << R_w_imu.col(1).transpose()
                  << " col2(IMU_Z)=" << R_w_imu.col(2).transpose();
    }

    prev_lidar_end_time_ = measures_.lidar_end_time_;
    return true;
}

void LaserMapping::ProjectKFs(CloudPtr cloud, int size_limit) {
    auto state = kf_.GetX();
    SE3 pose_cur(state.rot_, state.pos_);
    pose_cur = pose_cur.inverse();

    for (auto kf : proj_kfs_) {
        // LOG(INFO) << "projecting kf: " << kf->GetID();
        // if (last_kf_) {
        // auto kf = last_kf_;
        SE3 pose = pose_cur * kf->GetLIOPose() * SE3(offset_R_lidar_fixed_, offset_t_lidar_fixed_);

        int cnt = 0;
        for (auto &pt : kf->GetCloud()->points) {
            Vec3d p = pose * ToVec3d(pt);
            PointType pcl_pt;

            pcl_pt.x = p.x();
            pcl_pt.y = p.y();
            pcl_pt.z = p.z();
            pcl_pt.intensity = pt.intensity;

            cloud->push_back(pcl_pt);
            cnt++;

            if (cnt > size_limit) {
                break;
            }
        }
        // }
    }
}

void LaserMapping::MakeKF() {
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_);

    if (last_kf_) {
        /// opt pose 用之前的递推
        SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();
        kf->SetOptPose(last_kf_->GetOptPose() * delta);
    } else {
        kf->SetOptPose(kf->GetLIOPose());
    }

    kf->SetState(state_point_);

    if (capture_frame_valid_) {
        const bool force = data_capture_.params().capture_all_keyframes;
        data_capture_.saveFrontendCloud(
            capture_frame_, "07_keyframe_body", *kf->GetCloud(), imu_frame_id_, force);
        PointCloudType keyframe_world;
        pcl::transformPointCloud(
            *kf->GetCloud(), keyframe_world, kf->GetLIOPose().matrix());
        data_capture_.saveFrontendCloud(
            capture_frame_, "08_keyframe_world_lio", keyframe_world, world_frame_id_, force);

        const Eigen::Quaterniond lio_q(kf->GetLIOPose().so3().unit_quaternion());
        const Eigen::Quaterniond opt_q(kf->GetOptPose().so3().unit_quaternion());
        const Eigen::Vector3d lio_t = kf->GetLIOPose().translation();
        const Eigen::Vector3d opt_t = kf->GetOptPose().translation();
        std::ostringstream row;
        row << capture_frame_.id << "," << kf->GetID() << "," << std::setprecision(15)
            << state_point_.timestamp_ << ","
            << lio_t.x() << "," << lio_t.y() << "," << lio_t.z() << ","
            << lio_q.w() << "," << lio_q.x() << "," << lio_q.y() << "," << lio_q.z() << ","
            << opt_t.x() << "," << opt_t.y() << "," << opt_t.z() << ","
            << opt_q.w() << "," << opt_q.x() << "," << opt_q.y() << "," << opt_q.z();
        data_capture_.appendGlobalRow(
            "keyframes.csv",
            "processed_frame_id,keyframe_id,timestamp,"
            "lio_x,lio_y,lio_z,lio_qw,lio_qx,lio_qy,lio_qz,"
            "opt_x,opt_y,opt_z,opt_qw,opt_qx,opt_qy,opt_qz",
            row.str());
    }

    if (debug_visualization_) {
        debug_visualization_->publishCloud(
            "frontend/keyframe_body", *kf->GetCloud(), imu_frame_id_,
            state_point_.timestamp_, debug_frame_id_, true);
        if (debug_visualization_->cloudEnabled(debug_frame_id_, true)) {
            PointCloudType keyframe_world;
            pcl::transformPointCloud(
                *kf->GetCloud(), keyframe_world, kf->GetLIOPose().matrix());
            debug_visualization_->publishCloud(
                "frontend/keyframe_world", keyframe_world, world_frame_id_,
                state_point_.timestamp_, debug_frame_id_, true);
        }
        debug_visualization_->publishMetrics(
            {{"keyframe/id", static_cast<double>(kf->GetID())},
             {"keyframe/created", 1.0}},
            state_point_.timestamp_, debug_frame_id_, true);
    }

    LOG(INFO) << "LIO: create kf " << kf->GetID() << ", state: " << state_point_.pos_.transpose()
              << ", kf opt pose: " << kf->GetOptPose().translation().transpose()
              << ", lio pose: " << kf->GetLIOPose().translation().transpose() << ", time: " << std::setprecision(14)
              << state_point_.timestamp_;

    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;

    // 有keyframes时更新local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    /// 更新project kfs
    if (proj_kfs_.size() >= options_.max_proj_kfs_) {
        auto last = proj_kfs_.back();

        SE3 delta = last->GetLIOPose().inverse() * kf->GetLIOPose();

        if (delta.translation().norm() < 3 || delta.so3().log().norm() < 20 / 180 * M_PI) {
            // proj_kfs_.pop_back();
        } else {
            proj_kfs_.pop_front();
            proj_kfs_.emplace_back(kf);
        }
    } else {
        proj_kfs_.emplace_back(kf);
    }

    // for (auto &kf : proj_kfs_) {
    //     LOG(INFO) << "proj kf: " << kf->GetID();
    // }
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp <= last_timestamp_lidar_) {
                LOG_EVERY_N(WARNING, 100)
                    << "ignore non-increasing lidar timestamp, dt: " << timestamp - last_timestamp_lidar_;
                if (data_capture_.enabled()) {
                    std::ostringstream row;
                    row << std::setprecision(15) << timestamp << ",lidar_rejected,non_increasing_timestamp,"
                        << last_timestamp_lidar_;
                    data_capture_.appendGlobalRow(
                        "input_events.csv", "timestamp,event,reason,reference_timestamp", row.str());
                }
                return;
            }

            LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
                      << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            if (data_capture_.enabled() ||
                (debug_visualization_ && debug_visualization_->params().live_cloud_enabled)) {
                try {
                    lidar_input_buffer_.push_back(ConvertRosXyziForCapture(msg));
                } catch (const std::exception& e) {
                    LOG(ERROR) << "DataCapture failed to convert Polka input: " << e.what();
                    lidar_input_buffer_.push_back(cloud);
                }
                preprocess_stats_buffer_.push_back(preprocess_->GetLastStats());
            }
            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            // LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
            //           << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            double timestamp = math::ToSec(cloud->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

double LaserMapping::ComputeLidarEndTime(double begin_time, const PointCloudType &cloud, double mean_scan_time,
                                         bool input_is_predeskewed) {
    if (input_is_predeskewed) {
        return begin_time;
    }
    if (cloud.size() <= 1 || cloud.points.back().time / 1000.0 < 0.5 * mean_scan_time) {
        return begin_time + mean_scan_time;
    }
    return begin_time + cloud.points.back().time / 1000.0;
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        LOG(INFO) << "lidar or imu is empty";
        if (data_capture_.enabled()) {
            std::ostringstream row;
            row << std::setprecision(15) << last_timestamp_lidar_
                << ",sync_wait,empty_buffer," << lidar_buffer_.size() << "," << imu_buffer_.size();
            data_capture_.appendGlobalRow(
                "sync_events.csv", "timestamp,event,reason,lidar_buffer_size,imu_buffer_size", row.str());
        }
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        lidar_end_time_ = ComputeLidarEndTime(measures_.lidar_begin_time_, *measures_.scan_, lidar_mean_scantime_,
                                              preprocess_->InputIsPredeskewed());

        if (preprocess_->InputIsPredeskewed()) {
            // Polka has already transformed every point to the header timestamp.
        } else if (measures_.scan_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
        } else {
            scan_num_++;
            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;

            if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
                /// timestamp 有异常
                lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
                lidar_mean_scantime_ = lo::lidar_time_interval;
            }
        }

        lo::lidar_time_interval = lidar_mean_scantime_;

        // LOG(INFO) << "recompute lidar end time: " << std::setprecision(14) << lidar_end_time_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        LOG(INFO) << "sync failed: " << std::setprecision(14) << last_timestamp_imu_ << ", " << lidar_end_time_;
        if (data_capture_.enabled()) {
            std::ostringstream row;
            row << std::setprecision(15) << lidar_end_time_
                << ",sync_wait,imu_coverage," << lidar_buffer_.size() << "," << imu_buffer_.size();
            data_capture_.appendGlobalRow(
                "sync_events.csv", "timestamp,event,reason,lidar_buffer_size,imu_buffer_size", row.str());
        }
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());

        imu_buffer_.pop_front();
    }

    if ((data_capture_.enabled() || debug_visualization_) && !lidar_input_buffer_.empty()) {
        current_input_cloud_ = lidar_input_buffer_.front();
        lidar_input_buffer_.pop_front();
        if (!preprocess_stats_buffer_.empty()) {
            current_preprocess_stats_ = preprocess_stats_buffer_.front();
            preprocess_stats_buffer_.pop_front();
        }
    } else {
        current_input_cloud_ = measures_.scan_;
    }
    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    // LOG(INFO) << "sync: " << std::setprecision(14) << measures_.lidar_begin_time_ << ", " <<
    // measures_.lidar_end_time_;

    return true;
}

bool LaserMapping::SyncPackagesMultiBody() {
    // Check all lidar buffers have data
    for (const auto& lc : multibody_cfg_.lidars) {
        auto it = mb_lidar_buffers_.find(lc.id);
        if (it == mb_lidar_buffers_.end() || it->second.empty()) {
            LOG(INFO) << "mb sync: lidar " << lc.id << " buffer empty";
            return false;
        }
    }

    // Take leader lidar front as reference
    auto& leader_buf = mb_lidar_buffers_[leader_lidar_id_];
    if (leader_buf.empty()) {
        LOG(INFO) << "mb sync: leader lidar " << leader_lidar_id_ << " buffer empty";
        return false;
    }

    const double ref_begin = leader_buf.front().begin_time;
    const double ref_end = leader_buf.front().end_time;

    // Update mean scantime (from leader cloud)
    if (!preprocess_->InputIsPredeskewed() && leader_buf.front().cloud->size() > 1) {
        double scan_dur = leader_buf.front().cloud->points.back().time / 1000.0;
        if (scan_dur > 0.5 * lidar_mean_scantime_) {
            scan_num_++;
            lidar_mean_scantime_ += (scan_dur - lidar_mean_scantime_) / scan_num_;
            lo::lidar_time_interval = lidar_mean_scantime_;
        }
    }

    // For each non-leader lidar, find the entry closest to ref_begin
    // Store selected begin_times (avoid iterator invalidation on pop)
    std::map<int, double> selected_begin;
    selected_begin[leader_lidar_id_] = ref_begin;

    for (const auto& lc : multibody_cfg_.lidars) {
        if (lc.id == leader_lidar_id_) continue;
        auto& buf = mb_lidar_buffers_[lc.id];
        // Find entry within tolerance of ref_begin (buffers are sorted by begin_time)
        double best_dt = 1e9;
        double best_begin = -1;
        bool found = false;
        for (const auto& entry : buf) {
            double dt = std::abs(entry.begin_time - ref_begin);
            if (dt < best_dt) {
                best_dt = dt;
                best_begin = entry.begin_time;
                found = true;
            }
        }
        if (!found || best_dt > multibody_cfg_.sync_tolerance) {
            LOG(INFO) << "mb sync: lidar " << lc.id << " not in tolerance (dt=" << best_dt << ")";
            continue;
        }
        selected_begin[lc.id] = best_begin;
    }

    // Check IMU coverage: leader IMU must cover up to ref_end
    const auto* leader_imu_cfg = multibody_cfg_.findImuByBody(multibody_cfg_.leader_body_id);
    if (!leader_imu_cfg) return false;
    auto& leader_imu_buf = mb_imu_buffers_[leader_imu_cfg->id];
    if (leader_imu_buf.empty() || leader_imu_buf.back()->timestamp < ref_end) {
        LOG(INFO) << "mb sync: leader IMU not enough coverage, last_imu="
                  << (leader_imu_buf.empty() ? 0.0 : leader_imu_buf.back()->timestamp)
                  << " < ref_end=" << ref_end;
        return false;
    }

    // Build measure group
    mb_measures_.lidars.clear();
    mb_measures_.imus.clear();
    mb_measures_.lidar_begin_time = ref_begin;
    mb_measures_.lidar_end_time = ref_end;

    // Collect selected entries and compute global time bounds
    double global_begin = ref_begin;
    double global_end = ref_end;

    for (const auto& [lidar_id, begin_time] : selected_begin) {
        auto& buf = mb_lidar_buffers_[lidar_id];
        for (const auto& entry : buf) {
            if (entry.begin_time == begin_time) {
                mb_measures_.lidars[lidar_id] = entry;
                global_begin = std::min(global_begin, entry.begin_time);
                global_end = std::max(global_end, entry.end_time);
                break;
            }
        }
    }
    mb_measures_.lidar_begin_time = global_begin;
    mb_measures_.lidar_end_time = global_end;

    // Collect IMU samples per body
    // CRITICAL: use per-body end times, NOT global_end. The leader's deskew
    // (p_imu_->Process) uses measures_.lidar_begin/end_time_ which are the
    // leader's own times. If the IMU window was collected up to global_end
    // (which may be later than leader_end due to non-leader lidars), samples
    // between leader_end and global_end are wasted. Worse, if global_begin
    // is earlier than leader_begin, the window starts too early and the
    // sync pops samples that the next frame's leader needs.
    for (const auto& ic : multibody_cfg_.imus) {
        auto& buf = mb_imu_buffers_[ic.id];
        std::deque<IMUPtr> window;

        // Determine this IMU's body's lidar end time
        double body_end = global_end;  // fallback
        for (const auto& [lidar_id, begin_time] : selected_begin) {
            const auto* lc = multibody_cfg_.findLidar(lidar_id);
            if (lc && lc->body_id == ic.body_id) {
                auto& lbuf = mb_lidar_buffers_[lidar_id];
                for (const auto& entry : lbuf) {
                    if (entry.begin_time == begin_time) {
                        body_end = entry.end_time;
                        break;
                    }
                }
            }
        }
        // For the leader body, use ref_end (the leader lidar's end time)
        if (ic.body_id == multibody_cfg_.leader_body_id) {
            body_end = ref_end;
        }

        // Keep one sample before begin for interpolation continuity
        while (!buf.empty() && buf.front()->timestamp < global_begin) {
            if (window.empty()) {
                window.push_back(buf.front());
            } else {
                window.front() = buf.front();
            }
            buf.pop_front();
        }
        while (!buf.empty() && buf.front()->timestamp <= body_end) {
            window.push_back(buf.front());
            buf.pop_front();
        }
        if (!window.empty()) {
            mb_measures_.imus[ic.id] = std::move(window);
        }
    }

    // Pop consumed lidar entries (by begin_time match)
    for (const auto& [lidar_id, begin_time] : selected_begin) {
        auto& buf = mb_lidar_buffers_[lidar_id];
        while (!buf.empty() && buf.front().begin_time <= begin_time) {
            buf.pop_front();
        }
    }

    return true;
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);  // FIXME 这并发可能有点问题
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");

    if (capture_frame_valid_ ||
        (debug_visualization_ &&
         (debug_visualization_->cloudEnabled(debug_frame_id_) ||
          debug_visualization_->timeseriesEnabled(debug_frame_id_)))) {
        PointCloudType added;
        added.reserve(points_to_add.size() + point_no_need_downsample.size());
        for (const auto& point : points_to_add) added.push_back(point);
        for (const auto& point : point_no_need_downsample) added.push_back(point);
        added.width = static_cast<std::uint32_t>(added.size());
        added.height = 1;
        added.is_dense = false;

        if (capture_frame_valid_) {
            const bool force = data_capture_.params().capture_all_keyframes;
            data_capture_.saveFrontendCloud(
                capture_frame_, "06_ivox_points_added", added, world_frame_id_, force);
            std::ostringstream row;
            row << scan_down_body_->size() << "," << points_to_add.size() << ","
                << point_no_need_downsample.size() << "," << added.size() << ","
                << ivox_->NumValidGrids();
            data_capture_.appendFrameRow(
                capture_frame_, "ivox_incremental.csv",
                "candidate_points,voxel_checked_added,no_downsample_added,total_added,valid_grids",
                row.str(), force);
        }

        if (debug_visualization_) {
            debug_visualization_->publishCloud(
                "frontend/ivox_added", added, world_frame_id_,
                measures_.lidar_end_time_, debug_frame_id_);
            debug_visualization_->publishMetrics(
                {{"ivox/added_points", static_cast<double>(added.size())},
                 {"ivox/valid_grids", static_cast<double>(ivox_->NumValidGrids())}},
                measures_.lidar_end_time_, debug_frame_id_);
        }

        if (capture_frame_valid_ && capture_frame_.sampled &&
            data_capture_.params().capture_ivox_snapshot) {
            const auto snapshot_points = ivox_->GetAllPoints();
            PointCloudType snapshot;
            snapshot.points.assign(snapshot_points.begin(), snapshot_points.end());
            snapshot.width = static_cast<std::uint32_t>(snapshot.size());
            snapshot.height = 1;
            snapshot.is_dense = false;
            data_capture_.saveFrontendCloud(
                capture_frame_, "06_ivox_full_snapshot", snapshot, world_frame_id_);
        }
    }
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(NavState &s, ESKF::CustomObservationModel &obs) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    // LOG(INFO) << "obs from state: " << s.pos_.transpose() << ", " << s.rot_.unit_quaternion().coeffs().transpose();

    Timer::Evaluate(
        [&, this]() {
            // Points in IMU body frame — no LiDAR extrinsic needed
            Mat3f R_wl = s.rot_.matrix().cast<float>();
            Vec3f t_wl = s.pos_.cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                Vec3f p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                points_near.clear();

                /** Find the closest surfaces in the map **/
                ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;
                capture_rejection_reason_[i] = point_selected_surf_[i] ? 0 : 1;

                point_selected_icp_[i] = point_selected_surf_[i];

                /// 能找到3个点以上，则估计平面
                if (point_selected_surf_[i]) {
                    point_selected_surf_[i] =
                        math::esti_plane(plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
                    if (!point_selected_surf_[i]) {
                        capture_rejection_reason_[i] = 2;
                    }
                }

                /// 计算平面阈值
                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    if (p_body.norm() > 81 * pd2 * pd2) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    } else {
                        point_selected_surf_[i] = false;
                        capture_rejection_reason_[i] = 3;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_surf_ = 0;
    effect_feat_icp_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_surf_] = plane_coef_[i];
            corr_pts_[effect_feat_surf_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_surf_][3] = residuals_[i];

            effect_feat_surf_++;
        }

        if (point_selected_icp_[i]) {
            effect_feat_icp_++;
        }
    }

    corr_pts_.resize(effect_feat_surf_);
    corr_norm_.resize(effect_feat_surf_);
    obs.effective_feature_count_ = effect_feat_surf_;

    const bool capture_iteration = capture_frame_valid_ && capture_frame_.sampled &&
                                   data_capture_.shouldCaptureIteration(obs.iteration_);
    const bool publish_iteration = debug_visualization_ &&
                                   debug_visualization_->cloudEnabled(debug_frame_id_);
    if (capture_iteration || publish_iteration) {
        PointCloudType accepted_world;
        PointCloudType rejected_world;
        PointCloudType accepted_neighbors;
        accepted_world.reserve(effect_feat_surf_);
        rejected_world.reserve(cnt_pts - effect_feat_surf_);

        std::ostringstream correspondences;
        for (int i = 0; i < cnt_pts; ++i) {
            const bool accepted = point_selected_surf_[i];
            if (accepted) {
                accepted_world.push_back(scan_down_world_->points[i]);
                for (const auto& neighbor : nearest_points_[i]) {
                    accepted_neighbors.push_back(neighbor);
                }
            } else {
                rejected_world.push_back(scan_down_world_->points[i]);
            }

            const auto& body = scan_down_body_->points[i];
            const auto& world = scan_down_world_->points[i];
            correspondences << i << "," << (accepted ? 1 : 0) << ","
                            << capture_rejection_reason_[i] << ","
                            << body.x << "," << body.y << "," << body.z << ","
                            << world.x << "," << world.y << "," << world.z << ","
                            << residuals_[i] << ","
                            << plane_coef_[i].x() << "," << plane_coef_[i].y() << ","
                            << plane_coef_[i].z() << "," << plane_coef_[i].w() << "\n";
        }
        accepted_world.width = static_cast<std::uint32_t>(accepted_world.size());
        accepted_world.height = 1;
        rejected_world.width = static_cast<std::uint32_t>(rejected_world.size());
        rejected_world.height = 1;
        accepted_neighbors.width = static_cast<std::uint32_t>(accepted_neighbors.size());
        accepted_neighbors.height = 1;

        if (capture_iteration) {
            data_capture_.saveFrontendIterationCloud(
                capture_frame_, obs.iteration_, "scan_world", *scan_down_world_, world_frame_id_);
            data_capture_.saveFrontendIterationCloud(
                capture_frame_, obs.iteration_, "accepted_source_world", accepted_world, world_frame_id_);
            data_capture_.saveFrontendIterationCloud(
                capture_frame_, obs.iteration_, "rejected_source_world", rejected_world, world_frame_id_);
            data_capture_.saveFrontendIterationCloud(
                capture_frame_, obs.iteration_, "accepted_neighbors_world", accepted_neighbors, world_frame_id_);
            data_capture_.appendFrontendIterationRow(
                capture_frame_, obs.iteration_, "correspondences.csv",
                "point_index,accepted,rejection_reason,body_x,body_y,body_z,"
                "world_x,world_y,world_z,residual,plane_a,plane_b,plane_c,plane_d",
                correspondences.str());
        }
        if (publish_iteration) {
            // ROS 2 topic tokens may not begin with a digit.
            const std::string prefix =
                "iteration/iter_" + std::to_string(obs.iteration_) + "/";
            debug_visualization_->publishCloud(
                prefix + "scan_world", *scan_down_world_, world_frame_id_,
                measures_.lidar_end_time_, debug_frame_id_);
            debug_visualization_->publishCloud(
                prefix + "accepted", accepted_world, world_frame_id_,
                measures_.lidar_end_time_, debug_frame_id_);
            debug_visualization_->publishCloud(
                prefix + "rejected", rejected_world, world_frame_id_,
                measures_.lidar_end_time_, debug_frame_id_);
            debug_visualization_->publishCloud(
                prefix + "neighbors", accepted_neighbors, world_frame_id_,
                measures_.lidar_end_time_, debug_frame_id_);
        }
    }

    if (effect_feat_surf_ < 20) {
        obs.valid_ = false;
        LOG(WARNING) << "No enough effective surface points: " << effect_feat_surf_ << ", icp: " << effect_feat_icp_
                     << ", required: " << 20;
        return;
    }

    index.resize(effect_feat_surf_);
    // Points already in IMU body frame — no extrinsic
    const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

    /// 点面ICP部分
    obs.HTH_.setZero();
    obs.HTr_.setZero();

    std::vector<Mat6d> JTJ(effect_feat_surf_);
    std::vector<Vec6d> JTr(effect_feat_surf_);

    std::vector<double> res_sq(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        Vec3f point_this_be = corr_pts_[i].head<3>();
        Vec3f point_this = point_this_be;  // already in IMU frame
        Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        Vec3f norm_vec = corr_norm_[i].head<3>();

        /*** calculate the Measurement Jacobian matrix H ***/
        Vec3f C(Rt * norm_vec);
        Vec3f A(point_crossmat * C);

        Eigen::Matrix<double, 1, ESKF::pose_obs_dim_> J;
        J.setZero();
        J << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];

        float res = -corr_pts_[i][3];

        // double w = huber_weight(res);
        double w = 1.0;

        JTJ[i] = (J.transpose() * J).eval() * w;
        JTr[i] = J.transpose() * res * w;

        res_sq[i] = res * res;
    });

    for (int i = 0; i < index.size(); ++i) {
        obs.HTH_ += JTJ[i] * options_.plane_icp_weight_;
        obs.HTr_ += JTr[i] * options_.plane_icp_weight_;
    }

    if (!res_sq.empty()) {
        std::sort(res_sq.begin(), res_sq.end());
        obs.lidar_residual_mean_ = res_sq[res_sq.size() / 2];
        obs.lidar_residual_max_ = res_sq[res_sq.size() - 1];
        // LOG(INFO) << "residual mean: " << obs.lidar_residual_mean_ << ", max: " << obs.lidar_residual_max_
        //           << ", 85%: " << res_sq[res_sq.size() * 0.85];
    }

    /// 点到点ICP部分

    if (options_.enable_icp_part_) {
        JTJ.resize(cnt_pts);
        JTr.resize(cnt_pts);

        std::vector<size_t> index(cnt_pts);
        for (size_t i = 0; i < index.size(); ++i) {
            index[i] = i;
        }

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
            if (point_selected_icp_[i] == false) {
                return;
            }

            /// TODO: 外参
            Vec3d q = scan_down_body_->points[i].getVector3fMap().cast<double>();
            Vec3d qs = scan_down_world_->points[i].getVector3fMap().cast<double>();

            Eigen::Matrix<double, 3, ESKF::pose_obs_dim_> J;
            J.setZero();

            /// translation 部分
            J.block<3, 3>(0, 0) = Mat3d::Identity();

            /// rotation 部分
            J.block<3, 3>(0, 3) = -s.rot_.matrix() * SO3::hat(q);

            Vec3d e = qs - nearest_points_[i][0].getVector3fMap().cast<double>();

            if (e.norm() > 0.5) {
                point_selected_icp_[i] = false;
                return;
            }

            JTJ[i] = J.transpose() * J;
            JTr[i] = -J.transpose() * e;
        });

        for (int i = 0; i < cnt_pts; ++i) {
            if (point_selected_icp_[i] == false) {
                continue;
            }
            obs.HTH_ += JTJ[i] * options_.icp_weight_;
            obs.HTr_ += JTr[i] * options_.icp_weight_;
        }
    }
}

void LaserMapping::CaptureEskfIteration(const ESKF::IterationInfo& info) {
    const bool capture = capture_frame_valid_ && capture_frame_.sampled &&
                         data_capture_.shouldCaptureIteration(info.iteration);
    const bool publish = debug_visualization_ &&
                         debug_visualization_->timeseriesEnabled(debug_frame_id_);
    if (!capture && !publish) {
        return;
    }

    if (publish) {
        debug_visualization_->publishMetrics(
            {{"eskf/iteration", static_cast<double>(info.iteration)},
             {"eskf/valid", info.valid ? 1.0 : 0.0},
             {"eskf/accepted", info.accepted ? 1.0 : 0.0},
             {"eskf/converged", info.converged ? 1.0 : 0.0},
             {"eskf/effective_features", static_cast<double>(info.effective_feature_count)},
             {"eskf/observable_rank", static_cast<double>(info.observable_rank)},
             {"eskf/residual_mean", info.residual_mean},
             {"eskf/residual_max", info.residual_max},
             {"eskf/dx_rotation", info.increment.head<3>().norm()},
             {"eskf/dx_position", info.increment.segment<3>(3).norm()}},
            measures_.lidar_end_time_, debug_frame_id_);
    }
    if (!capture) {
        return;
    }

    std::ostringstream row;
    row << info.iteration << "," << (info.valid ? 1 : 0) << ","
        << (info.accepted ? 1 : 0) << "," << (info.converged ? 1 : 0) << ","
        << info.effective_feature_count << "," << info.observable_rank << ","
        << info.residual_mean << "," << info.residual_max << ","
        << StateCsv(info.state_before) << "," << StateCsv(info.state_after);
    for (int i = 0; i < ESKF::state_dim_; ++i) row << "," << info.increment(i);
    for (int i = 0; i < ESKF::pose_obs_dim_; ++i) row << "," << info.eigenvalues(i);
    for (int r = 0; r < ESKF::pose_obs_dim_; ++r) {
        for (int c = 0; c < ESKF::pose_obs_dim_; ++c) row << "," << info.hth(r, c);
    }
    for (int i = 0; i < ESKF::pose_obs_dim_; ++i) row << "," << info.htr(i);

    std::string header =
        std::string("iteration,valid,accepted,converged,effective_features,observable_rank,")
        + "residual_mean,residual_max," + PrefixedStateCsvHeader("before_") + "," +
        PrefixedStateCsvHeader("after_");
    for (int i = 0; i < ESKF::state_dim_; ++i) header += ",dx_" + std::to_string(i);
    for (int i = 0; i < ESKF::pose_obs_dim_; ++i) header += ",eigenvalue_" + std::to_string(i);
    for (int r = 0; r < ESKF::pose_obs_dim_; ++r) {
        for (int c = 0; c < ESKF::pose_obs_dim_; ++c) {
            header += ",hth_" + std::to_string(r) + "_" + std::to_string(c);
        }
    }
    for (int i = 0; i < ESKF::pose_obs_dim_; ++i) header += ",htr_" + std::to_string(i);

    data_capture_.appendFrameRow(
        capture_frame_, "state_iterations.csv", header, row.str());
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    for (auto &kf : all_keyframes_) {
        CloudPtr cloud = kf->GetCloud();

        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);

        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        // Points in IMU body frame, direct transform
        if (use_lio_pose) {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetLIOPose().matrix());
        } else {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetOptPose().matrix());
        }

        *global_map += *cloud_trans;

        LOG(INFO) << "kf " << kf->GetID() << ", pose: " << kf->GetOptPose().translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    LOG(INFO) << "global map: " << global_map_filtered->size();

    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    /// 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    LOG(INFO) << "lio map is saved to ./data/lio.pcd";
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

CloudPtr LaserMapping::GetProjCloud() {
    auto cloud = scan_undistort_;
    ProjectKFs(cloud);
    return cloud;
}

void LaserMapping::ProcessNonLeaderBodies(const NavState& leader_seed, double seed_time) {
    if (!tf_buffer_) {
        LOG(WARNING) << "[non-leader] no TF buffer, skipping";
        return;
    }

    const double lidar_begin = mb_measures_.lidar_begin_time;
    const double lidar_end = mb_measures_.lidar_end_time;
    const std::string& leader_imu_frame = multibody_cfg_.leader_imu_frame;

    // Re-init non-leader states on first call
    if (nonleader_states_.empty()) {
        for (const auto& lc : multibody_cfg_.lidars) {
            if (lc.is_leader) continue;
            // Find or create state for this body
            auto it = std::find_if(nonleader_states_.begin(), nonleader_states_.end(),
                                   [&](const NonLeaderBodyState& s) { return s.body_id == lc.body_id; });
            if (it == nonleader_states_.end()) {
                NonLeaderBodyState state;
                state.body_id = lc.body_id;
                // Find IMU for this body
                const auto* imu_cfg = multibody_cfg_.findImuByBody(lc.body_id);
                if (imu_cfg) state.imu_id = imu_cfg->id;
                nonleader_states_.push_back(std::move(state));
                it = std::prev(nonleader_states_.end());
            }
            // Add this lidar to the body's list
            if (std::find(it->lidar_ids.begin(), it->lidar_ids.end(), lc.id) == it->lidar_ids.end()) {
                it->lidar_ids.push_back(lc.id);
            }
            // Set extrinsics on processor
            it->processor.setExtrinsic(lc.R_lidar_imu, lc.t_lidar_imu);
            it->processor.setGravity(leader_seed.grav_);
        }
    }

    // Use seed_time if valid, else lidar_begin
    const double effective_seed_time = (seed_time > 1e6) ? seed_time : lidar_begin;

    // Gate: skip non-leader during sharp turns (omega > 0.3 rad/s).
    // The 12-DOF EKF uses constant-velocity prediction (no online ba/grav).
    // During turns, non-leader deskew amplifies prediction errors → pitch oscillation.
    const double omega_mag = p_imu_->GetAngvelLast().norm();
    if (omega_mag > 0.3) return;

    for (auto& nl : nonleader_states_) {
        // 1. Accumulate bias
        auto imu_it = mb_measures_.imus.find(nl.imu_id);
        if (imu_it == mb_measures_.imus.end() || imu_it->second.empty()) continue;

        if (!nl.processor.isBiasReady()) {
            nl.processor.accumulateBias(imu_it->second);
            if (!nl.processor.isBiasReady()) continue;
        }

        // 2. Cross-body transforms via explicit articulation kinematics
        Mat3d R_cross_seed, R_cross_end;
        Vec3d t_cross_seed, t_cross_end;

        auto compute_cross = [&](double time, Mat3d& R_out, Vec3d& t_out) -> bool {
            if (multibody_cfg_.joints.empty() || joint_state_buffer_.empty()) return false;
            auto angle_opt = joint_state_buffer_.interpolate(time);
            if (!angle_opt) return false;

            // Compute T(leader_base ← nl_base) from joint angle
            Mat3d R_base; Vec3d t_base;
            computeJointTransform(*angle_opt, multibody_cfg_.joints[0], R_base, t_base);

            // Compose to T(leader_imu ← nl_imu)
            auto it_r = R_nl_imu_base_.find(nl.body_id);
            auto it_t = t_nl_imu_base_.find(nl.body_id);
            if (it_r == R_nl_imu_base_.end()) return false;
            composeCrossTransformIMU(R_base, t_base,
                                     R_leader_base_imu_, t_leader_base_imu_,
                                     it_r->second, it_t->second,
                                     R_out, t_out);
            return true;
        };

        // Try seed_time first, fall back to lidar_begin
        bool seed_ok = compute_cross(effective_seed_time, R_cross_seed, t_cross_seed);
        if (!seed_ok) {
            seed_ok = compute_cross(lidar_begin, R_cross_seed, t_cross_seed);
        }
        bool end_ok = compute_cross(lidar_end, R_cross_end, t_cross_end);

        if (!seed_ok || !end_ok) {
            // Fallback to TF if explicit kinematics failed
            const auto* nl_lidar_cfg = multibody_cfg_.findLidar(nl.lidar_ids.front());
            if (!nl_lidar_cfg || !tf_buffer_) continue;
            try {
                auto tp_seed = tf2::timeFromSec(effective_seed_time);
                auto tp_end = tf2::timeFromSec(lidar_end);
                auto tf_seed = tf_buffer_->lookupTransform(leader_imu_frame, nl_lidar_cfg->imu_frame,
                                                           tp_seed, tf2::durationFromSec(0.05));
                auto tf_end = tf_buffer_->lookupTransform(leader_imu_frame, nl_lidar_cfg->imu_frame,
                                                          tp_end, tf2::durationFromSec(0.05));
                Eigen::Isometry3d T_seed = tf2::transformToEigen(tf_seed.transform);
                Eigen::Isometry3d T_end = tf2::transformToEigen(tf_end.transform);
                R_cross_seed = T_seed.rotation(); t_cross_seed = T_seed.translation();
                R_cross_end = T_end.rotation(); t_cross_end = T_end.translation();
                seed_ok = end_ok = true;
            } catch (const tf2::TransformException& ex) {
                LOG(WARNING) << "[non-leader] cross-body transform failed for body " << nl.body_id
                             << " (explicit + TF fallback): " << ex.what();
                continue;
            }
        }

        // 3. Compute seed state for non-leader
        DeskewSeedState seed;
        seed.rot = leader_seed.rot_.matrix() * R_cross_seed;
        seed.pos = leader_seed.pos_ + leader_seed.rot_.matrix() * t_cross_seed;
        // Velocity lever-arm correction: use leader's angular velocity
        // (leader and non-leader are rigidly connected through the joint;
        //  omega_leader × lever_arm gives the velocity difference)
        Vec3d omega_world = leader_seed.rot_.matrix() * p_imu_->GetAngvelLast();
        Vec3d lever = seed.pos - leader_seed.pos_;
        seed.vel = leader_seed.vel_ + omega_world.cross(lever);
        seed.bg = nl.processor.getMeanGyr();
        seed.grav = leader_seed.grav_;

        // 4. Pure forward propagation with non-leader IMU
        nl.processor.pureForwardPropagation(imu_it->second, seed, effective_seed_time,
                                            lidar_begin, lidar_end);

        // 5. Deskew each non-leader lidar + cross-body align + merge
        for (int lidar_id : nl.lidar_ids) {
            auto lc_it = mb_measures_.lidars.find(lidar_id);
            if (lc_it == mb_measures_.lidars.end() || !lc_it->second.cloud || lc_it->second.cloud->empty())
                continue;

            const auto* lc = multibody_cfg_.findLidar(lidar_id);
            if (!lc) continue;

            // Update extrinsics for this specific lidar (may differ within same body)
            nl.processor.setExtrinsic(lc->R_lidar_imu, lc->t_lidar_imu);

            // Copy + deskew (output: non-leader LiDAR frame at scan-end)
            CloudPtr cloud_deskew = std::make_shared<PointCloudType>(*lc_it->second.cloud);
            nl.processor.undistortLidar(cloud_deskew, lc_it->second.begin_time, lc_it->second.end_time);

            // Cross-body alignment at scan-end: non-leader LiDAR → leader LiDAR.
            // The deskew outputs points in the non-leader LiDAR frame (lightning
            // convention: R_L_I^T × (… − t_L_I) wrapping transforms back to LiDAR).
            // So the cross-body transform must be T(leader_lidar ← nonleader_lidar),
            // NOT T(leader_imu ← nonleader_imu) which the colleague uses (their
            // deskew outputs IMU-frame points without the wrapping).
            //
            // Compute from IMU-to-IMU cross_end + extrinsics:
            //   T(leader_lidar ← nl_lidar) = T(leader_lidar ← leader_imu)
            //                               × T(leader_imu ← nl_imu)      [= cross_end]
            //                               × T(nl_imu ← nl_lidar)
            // T(leader_lidar ← leader_imu) = inverse of leader extrinsic
            // T(nl_imu ← nl_lidar) = nl extrinsic
            const Mat3d& R_L_I_leader = multibody_cfg_.findLeaderLidar()->R_lidar_imu;
            const Vec3d& t_L_I_leader = multibody_cfg_.findLeaderLidar()->t_lidar_imu;
            // R_leader_lidar_from_imu = R_L_I_leader^T (IMU→LiDAR)
            // t_leader_lidar_from_imu = -R_L_I_leader^T × t_L_I_leader
            Mat3d R_ldr_cross = R_L_I_leader.transpose() * R_cross_end * lc->R_lidar_imu;
            Vec3d t_ldr_cross = R_L_I_leader.transpose() *
                                (R_cross_end * lc->t_lidar_imu + t_cross_end - t_L_I_leader);

            for (auto& pt : cloud_deskew->points) {
                Vec3d p(pt.x, pt.y, pt.z);
                Vec3d p_aligned = R_ldr_cross * p + t_ldr_cross;
                if (p_aligned.allFinite()) {
                    pt.x = p_aligned(0);
                    pt.y = p_aligned(1);
                    pt.z = p_aligned(2);
                }
            }

            // Merge into leader scan_undistort_ (in leader LiDAR frame)
            *scan_undistort_ += *cloud_deskew;
            LOG(INFO) << "[non-leader] body=" << nl.body_id << " lidar=" << lidar_id
                      << " merged " << cloud_deskew->size() << " pts";
        }
    }
}

}  // namespace lightning
