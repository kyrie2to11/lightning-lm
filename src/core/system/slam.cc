//
// Created by xiang on 25-5-6.
//

#include <sensor_msgs/msg/joint_state.hpp>

#include "core/system/slam.h"
#include "common/debug_visualization.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/lio/multibody.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <chrono>
#include <sstream>
#include <thread>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);

    DebugVisualization::Params debug_params;
    if (const auto debug = yaml["debug_visualization"]) {
        debug_params.live_cloud_enabled = debug["live_cloud_enabled"].as<bool>(false);
        debug_params.live_cloud_every_n_frames =
            debug["live_cloud_every_n_frames"].as<int>(10);
        debug_params.live_timeseries_enabled =
            debug["live_timeseries_enabled"].as<bool>(false);
        debug_params.live_timeseries_every_n_frames =
            debug["live_timeseries_every_n_frames"].as<int>(1);
        debug_params.max_cloud_points = debug["max_cloud_points"].as<std::size_t>(50000);
        debug_params.max_correspondence_markers =
            debug["max_correspondence_markers"].as<std::size_t>(2000);
    }
    DebugVisualization::Validate(debug_params);

    // Data capture config
    if (yaml["data_capture"] && yaml["data_capture"]["enabled"].as<bool>(false)) {
        const auto capture = yaml["data_capture"];
        DataCapture::Params dp;
        dp.enabled = true;
        dp.output_dir = capture["output_dir"].as<std::string>("/tmp/lightning_capture");
        dp.every_n_frames = capture["every_n_frames"].as<int>(1);
        dp.frame_start = capture["frame_start"].as<std::int64_t>(0);
        dp.frame_end = capture["frame_end"].as<std::int64_t>(-1);
        dp.eskf_iteration_stride = capture["eskf_iteration_stride"].as<int>(1);
        dp.imu_sample_stride = capture["imu_sample_stride"].as<int>(1);
        dp.frontend_enabled = capture["frontend_enabled"].as<bool>(true);
        dp.frontend_iterations_enabled = capture["frontend_iterations_enabled"].as<bool>(true);
        dp.backend_enabled = capture["backend_enabled"].as<bool>(true);
        dp.map_output_enabled = capture["map_output_enabled"].as<bool>(true);
        dp.capture_all_keyframes = capture["capture_all_keyframes"].as<bool>(true);
        dp.capture_all_loop_candidates = capture["capture_all_loop_candidates"].as<bool>(true);
        dp.capture_all_pgo_events = capture["capture_all_pgo_events"].as<bool>(true);
        dp.capture_ivox_snapshot = capture["capture_ivox_snapshot"].as<bool>(false);
        dp.binary_compressed = capture["binary_compressed"].as<bool>(true);
        dp.max_points_per_cloud = capture["max_points_per_cloud"].as<std::size_t>(0);
        lio_->data_capture_.configure(dp);
        LOG(INFO) << "Data capture enabled: " << dp.output_dir
                  << ", every_n_frames=" << dp.every_n_frames;
    }

    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    if (options_.map_path_.empty() && yaml["system"]["map_path"]) {
        options_.map_path_ = yaml["system"]["map_path"].as<std::string>();
    }

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->SetDataCapture(&lio_->data_capture_);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次重绘
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");
        if (debug_params.live_cloud_enabled || debug_params.live_timeseries_enabled) {
            debug_visualization_ = std::make_shared<DebugVisualization>(debug_params, node_);
            lio_->SetDebugVisualization(debug_visualization_);
            if (lc_) {
                lc_->SetDebugVisualization(debug_visualization_);
            }
            LOG(INFO) << "Lightning debug visualization enabled: clouds="
                      << debug_params.live_cloud_enabled
                      << ", timeseries=" << debug_params.live_timeseries_enabled;
        }

        // Check multi-body config
        const bool multibody_enabled =
            yaml["multibody"] && yaml["multibody"]["enabled"] && yaml["multibody"]["enabled"].as<bool>();

        if (multibody_enabled) {
            if (!InitMultiBody(yaml)) {
                return false;
            }
        } else {
            // Legacy single-lidar path
            imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
            cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
            livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

            if (!ConfigureExtrinsicFromTf(yaml)) {
                return false;
            }

            rclcpp::QoS qos(10);

            imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                    IMUPtr imu = std::make_shared<IMU>();
                    imu->timestamp = ToSec(msg->header.stamp);
                    imu->linear_acceleration =
                        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                    imu->angular_velocity =
                        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                    ProcessIMU(imu);
                });

            cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                    Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
                });

            livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                    Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
                });
        }

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        LOG(INFO) << "online slam node has been created.";
    }

    if (lio_->data_capture_.enabled()) {
        const auto fasterlio = yaml["fasterlio"];
        std::ostringstream runtime;
        runtime << "config_file: " << yaml_path << "\n"
                << "pipeline: polka_merged_single_body\n"
                << "lidar_frame: " << fasterlio["lidar_frame_id"].as<std::string>("unknown") << "\n"
                << "imu_frame: " << fasterlio["imu_frame_id"].as<std::string>("unknown") << "\n"
                << "world_frame: " << fasterlio["world_frame_id"].as<std::string>("world") << "\n"
                << "pointwise_deskew: "
                << (lio_->GetInputIsPredeskewed() ? "false" : "true") << "\n"
                << "extrinsic_direction: imu_from_lidar\n"
                << "extrinsic_translation: [" << lio_->GetExtrinsicTranslation().transpose() << "]\n"
                << "extrinsic_rotation_row_major: [";
        const Mat3d& rotation = lio_->GetExtrinsicRotation();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                if (row != 0 || col != 0) runtime << ", ";
                runtime << rotation(row, col);
            }
        }
        runtime << "]\n"
                << "loop_closing_enabled: " << (options_.with_loop_closing_ ? "true" : "false") << "\n";
        lio_->data_capture_.writeRunMetadata(runtime.str());
    }

    return true;
}

bool SlamSystem::ConfigureExtrinsicFromTf(const YAML::Node& yaml) {
    const auto fasterlio = yaml["fasterlio"];
    const bool extrinsic_from_tf =
        fasterlio["extrinsic_from_tf"] && fasterlio["extrinsic_from_tf"].as<bool>();
    if (!extrinsic_from_tf) {
        return true;
    }

    const std::string lidar_frame_id =
        fasterlio["lidar_frame_id"] ? fasterlio["lidar_frame_id"].as<std::string>() : "";
    const std::string imu_frame_id =
        fasterlio["imu_frame_id"] ? fasterlio["imu_frame_id"].as<std::string>() : "";
    if (lidar_frame_id.empty() || imu_frame_id.empty()) {
        LOG(ERROR) << "fasterlio.extrinsic_from_tf requires lidar_frame_id and imu_frame_id";
        return false;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

    // lookupTransform 的 duration 超时只在 frame 已存在但请求时刻 transform 未到时才等待；
    // 对 “frame 尚未被接收” 会立刻抛 “does not exist”。离线回放时 /tf_static 是一次性 latched，
    // 可能晚于本函数到达，故先轮询 canTransform 等 frame 出现再做 lookup。
    auto wait_for_transform = [&](const std::string& target, const std::string& source,
                                  double timeout_sec) -> bool {
        for (int i = 0; i < static_cast<int>(timeout_sec * 10.0); ++i) {
            if (tf_buffer_->canTransform(target, source, tf2::TimePointZero)) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return tf_buffer_->canTransform(target, source, tf2::TimePointZero);
    };

    try {
        if (!wait_for_transform(imu_frame_id, lidar_frame_id, 5.0)) {
            LOG(ERROR) << "Timed out waiting for TF " << imu_frame_id << " <- " << lidar_frame_id;
            return false;
        }
        const auto transform = tf_buffer_->lookupTransform(
            imu_frame_id, lidar_frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
        const Eigen::Isometry3d T_imu_lidar = tf2::transformToEigen(transform.transform);
        lio_->SetExtrinsic(T_imu_lidar.translation(), T_imu_lidar.rotation());

        LOG(INFO) << "Loaded fasterlio extrinsic from TF: " << imu_frame_id << " <- "
                  << lidar_frame_id << ", t=" << T_imu_lidar.translation().transpose()
                  << ", R=\n" << T_imu_lidar.rotation().matrix();

        const bool init_world_from_tf =
            fasterlio["init_world_from_tf"] && fasterlio["init_world_from_tf"].as<bool>();
        if (init_world_from_tf) {
            const std::string world_frame_id =
                fasterlio["world_frame_id"] ? fasterlio["world_frame_id"].as<std::string>() : "";
            const std::string initialization_frame_id = fasterlio["initialization_frame_id"]
                                                            ? fasterlio["initialization_frame_id"].as<std::string>()
                                                            : world_frame_id;
            if (initialization_frame_id.empty()) {
                LOG(ERROR) << "fasterlio.init_world_from_tf requires initialization_frame_id "
                              "or world_frame_id";
                return false;
            }

            if (!wait_for_transform(initialization_frame_id, imu_frame_id, 5.0)) {
                LOG(ERROR) << "Timed out waiting for TF " << initialization_frame_id << " <- "
                           << imu_frame_id;
                return false;
            }
            const auto world_transform = tf_buffer_->lookupTransform(
                initialization_frame_id, imu_frame_id, tf2::TimePointZero,
                tf2::durationFromSec(0.5));
            const Eigen::Isometry3d T_world_imu = tf2::transformToEigen(world_transform.transform);
            lio_->SetInitialWorldImuRotation(T_world_imu.rotation());

            LOG(INFO) << "Loaded fasterlio initial world rotation from TF: "
                      << initialization_frame_id << " <- " << imu_frame_id
                      << "; output world frame: " << world_frame_id;
        }
    } catch (const tf2::TransformException& ex) {
        LOG(ERROR) << "Failed to lookup fasterlio extrinsic TF " << imu_frame_id << " <- "
                   << lidar_frame_id << ": " << ex.what();
        return false;
    }

    return true;
}

bool SlamSystem::InitMultiBody(const YAML::Node& yaml) {
    MultiBodyConfig cfg;
    const auto& mb = yaml["multibody"];

    cfg.enabled = true;
    cfg.leader_body_id = mb["leader_body_id"].as<std::string>();
    cfg.leader_imu_frame = mb["leader_imu_frame"].as<std::string>();
    if (mb["sync_tolerance"]) cfg.sync_tolerance = mb["sync_tolerance"].as<double>();

    // Parse lidars
    for (const auto& l : mb["lidars"]) {
        MultiBodyLidarConfig lc;
        lc.id = l["id"].as<int>();
        lc.body_id = l["body"].as<std::string>();
        lc.topic = l["topic"].as<std::string>();
        lc.lidar_frame = l["lidar_frame"].as<std::string>();
        lc.imu_frame = l["imu_frame"].as<std::string>();
        lc.is_leader = (lc.body_id == cfg.leader_body_id);
        cfg.lidars.push_back(lc);
    }

    // Parse imus
    for (const auto& i : mb["imus"]) {
        MultiBodyImuConfig ic;
        ic.id = i["id"].as<int>();
        ic.body_id = i["body"].as<std::string>();
        ic.topic = i["topic"].as<std::string>();
        cfg.imus.push_back(ic);
    }

    // Parse articulation joint config
    if (mb["articulation"] && mb["articulation"]["joints"]) {
        for (const auto& j : mb["articulation"]["joints"]) {
            MultiBodyConfig::JointConfig jc;
            jc.name = j["name"].as<std::string>();
            jc.body_a = j["body_a"].as<std::string>();
            jc.body_b = j["body_b"].as<std::string>();
            auto pa = j["pin_position_a"].as<std::vector<double>>();
            auto pb = j["pin_position_b"].as<std::vector<double>>();
            auto ax = j["axis_a"].as<std::vector<double>>();
            jc.pin_position_a = Vec3d(pa[0], pa[1], pa[2]);
            jc.pin_position_b = Vec3d(pb[0], pb[1], pb[2]);
            jc.axis = Vec3d(ax[0], ax[1], ax[2]);
            if (j["zero_offset"]) jc.zero_offset = j["zero_offset"].as<double>();
            jc.joint_state_name = j["joint_state_name"].as<std::string>();
            cfg.joints.push_back(jc);
        }
        if (mb["articulation"]["joint_states_topic"]) {
            cfg.joint_states_topic = mb["articulation"]["joint_states_topic"].as<std::string>();
        }
    }

    LOG(INFO) << "Multi-body config: " << cfg.lidars.size() << " lidars, " << cfg.imus.size()
              << " imus, " << cfg.joints.size() << " joints, leader=" << cfg.leader_body_id;

    // Create TF buffer for extrinsics + cross-body lookups
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

    // Load extrinsics for each lidar from TF (T(imu_frame ← lidar_frame))
    auto wait_for_tf = [&](const std::string& target, const std::string& source, double timeout) -> bool {
        for (int i = 0; i < static_cast<int>(timeout * 10.0); ++i) {
            if (tf_buffer_->canTransform(target, source, tf2::TimePointZero)) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return tf_buffer_->canTransform(target, source, tf2::TimePointZero);
    };

    for (auto& lc : cfg.lidars) {
        if (!wait_for_tf(lc.imu_frame, lc.lidar_frame, 5.0)) {
            LOG(ERROR) << "Timed out waiting for TF " << lc.imu_frame << " <- " << lc.lidar_frame;
            return false;
        }
        auto tf = tf_buffer_->lookupTransform(lc.imu_frame, lc.lidar_frame, tf2::TimePointZero,
                                               tf2::durationFromSec(0.5));
        Eigen::Isometry3d T = tf2::transformToEigen(tf.transform);
        lc.R_lidar_imu = T.rotation();
        lc.t_lidar_imu = T.translation();
        LOG(INFO) << "Extrinsic " << lc.imu_frame << " <- " << lc.lidar_frame
                  << ": t=" << lc.t_lidar_imu.transpose();
    }

    // Load static base←imu transforms for explicit articulation kinematics
    if (!cfg.joints.empty()) {
        // Leader: T(leader_base ← leader_imu)
        const std::string& leader_base = cfg.findLeaderLidar()->body_id == cfg.leader_body_id
            ? (cfg.leader_body_id == "rear" ? "base_footprint" : "front_link")
            : "base_footprint";
        // Determine leader base frame from config
        std::string leader_base_frame = "base_footprint";
        std::string nonleader_base_frame = "front_link";
        // Simple heuristic: rear body → base_footprint, front body → front_link
        if (cfg.leader_body_id == "rear") {
            leader_base_frame = "base_footprint";
            nonleader_base_frame = "front_link";
        } else {
            leader_base_frame = "base_footprint";
            nonleader_base_frame = "rear_link";
        }

        if (wait_for_tf(leader_base_frame, cfg.leader_imu_frame, 5.0)) {
            auto tf = tf_buffer_->lookupTransform(leader_base_frame, cfg.leader_imu_frame,
                                                   tf2::TimePointZero, tf2::durationFromSec(0.5));
            Eigen::Isometry3d T = tf2::transformToEigen(tf.transform);
            lio_->SetLeaderBaseImu(T.rotation(), T.translation());
            LOG(INFO) << "Static T(" << leader_base_frame << " ← " << cfg.leader_imu_frame
                      << "): t=" << T.translation().transpose();
        }

        // Non-leader: T(nl_imu ← nl_base) for each non-leader body
        for (const auto& lc : cfg.lidars) {
            if (lc.is_leader) continue;
            std::string nl_base = (lc.body_id == "rear") ? "base_footprint" : "front_link";
            if (wait_for_tf(lc.imu_frame, nl_base, 5.0)) {
                auto tf = tf_buffer_->lookupTransform(lc.imu_frame, nl_base,
                                                       tf2::TimePointZero, tf2::durationFromSec(0.5));
                Eigen::Isometry3d T = tf2::transformToEigen(tf.transform);
                lio_->SetNonLeaderImuBase(lc.body_id, T.rotation(), T.translation());
                LOG(INFO) << "Static T(" << lc.imu_frame << " ← " << nl_base
                          << "): t=" << T.translation().transpose();
            }
        }
    }

    // Set leader extrinsic on LaserMapping (for ESKF observation model)
    const auto* leader_lc = cfg.findLeaderLidar();
    if (!leader_lc) {
        LOG(ERROR) << "no leader lidar found";
        return false;
    }
    lio_->SetExtrinsic(leader_lc->t_lidar_imu, leader_lc->R_lidar_imu);

    // Initial world rotation from TF (same as legacy path)
    const auto& fasterlio = yaml["fasterlio"];
    const bool init_world_from_tf =
        fasterlio["init_world_from_tf"] && fasterlio["init_world_from_tf"].as<bool>();
    if (init_world_from_tf) {
        const std::string world_frame = fasterlio["world_frame_id"].as<std::string>();
        const std::string initialization_frame = fasterlio["initialization_frame_id"]
                                                     ? fasterlio["initialization_frame_id"].as<std::string>()
                                                     : world_frame;
        if (!wait_for_tf(initialization_frame, cfg.leader_imu_frame, 5.0)) {
            LOG(ERROR) << "Timed out waiting for TF " << initialization_frame << " <- "
                       << cfg.leader_imu_frame;
            return false;
        }
        auto world_tf = tf_buffer_->lookupTransform(initialization_frame, cfg.leader_imu_frame,
                                                     tf2::TimePointZero,
                                                     tf2::durationFromSec(0.5));
        Eigen::Isometry3d T_world_imu = tf2::transformToEigen(world_tf.transform);
        lio_->SetInitialWorldImuRotation(T_world_imu.rotation());
        LOG(INFO) << "Loaded initial world rotation from TF: " << initialization_frame << " <- "
                  << cfg.leader_imu_frame << "; output world frame: " << world_frame;
    }

    // Pass config + TF buffer to LaserMapping
    lio_->SetMultiBodyConfig(cfg);
    lio_->SetTfBuffer(tf_buffer_);

    // Create per-lidar subscriptions
    rclcpp::QoS qos(10);
    for (const auto& lc : cfg.lidars) {
        int lidar_id = lc.id;
        auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            lc.topic, qos, [this, lidar_id](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                Timer::Evaluate(
                    [&]() {
                        if (running_ == false) return;
                        lio_->ProcessPointCloud2(cloud, lidar_id);
                        lio_->Run();
                        auto kf = lio_->GetKeyframe();
                        if (kf != cur_kf_) {
                            cur_kf_ = kf;
                            if (cur_kf_) {
                                if (options_.with_loop_closing_) lc_->AddKF(cur_kf_);
                                if (options_.with_gridmap_) g2p5_->PushKeyframe(cur_kf_);
                                if (ui_) ui_->UpdateKF(cur_kf_);
                            }
                        }
                    },
                    "Proc Lidar MB", true);
            });
        mb_cloud_subs_.push_back(sub);
        LOG(INFO) << "Subscribed lidar " << lidar_id << " (" << lc.body_id << ") on " << lc.topic;
    }

    // Create per-IMU subscriptions
    for (const auto& ic : cfg.imus) {
        int imu_id = ic.id;
        auto sub = node_->create_subscription<sensor_msgs::msg::Imu>(
            ic.topic, qos, [this, imu_id](sensor_msgs::msg::Imu::SharedPtr msg) {
                if (running_ == false) return;
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
                lio_->ProcessIMU(imu, imu_id);
            });
        mb_imu_subs_.push_back(sub);
        LOG(INFO) << "Subscribed imu " << imu_id << " (" << ic.body_id << ") on " << ic.topic;
    }

    // Joint states subscription
    if (!cfg.joints.empty()) {
        const std::string& js_topic = cfg.joint_states_topic;
        const std::string& joint_name = cfg.joints[0].joint_state_name;
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            js_topic, qos, [this, joint_name](sensor_msgs::msg::JointState::SharedPtr msg) {
                if (running_ == false) return;
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    if (msg->name[i] == joint_name && i < msg->position.size()) {
                        double ts = ToSec(msg->header.stamp);
                        lio_->ProcessJointStates(ts, msg->position[i]);
                        break;
                    }
                }
            });
        LOG(INFO) << "Subscribed joint_states on " << js_topic << " (joint: " << joint_name << ")";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    const std::filesystem::path base = options_.map_path_.empty() ? std::filesystem::path("./data/")
                                                                   : std::filesystem::path(options_.map_path_);
    const std::string save_path = (base / map_name_).string();

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        const std::filesystem::path base = options_.map_path_.empty() ? std::filesystem::path("./data/")
                                                                       : std::filesystem::path(options_.map_path_);
        save_path = (base / map_name_).string();
    }

    LOG(INFO) << "slam map saving to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);

    if (lio_->data_capture_.enabled() && lio_->data_capture_.params().map_output_enabled) {
        CloudPtr lio_map =
            options_.with_loop_closing_ ? lio_->GetGlobalMap(true) : global_map;
        CloudPtr optimized_map =
            options_.with_loop_closing_ ? global_map : lio_->GetGlobalMap(false);
        const std::string& world_frame = lio_->GetWorldFrameId();
        lio_->data_capture_.saveMapCloud("assembled_lio", *lio_map, world_frame);
        lio_->data_capture_.saveMapCloud("assembled_optimized", *optimized_map, world_frame);
        lio_->data_capture_.saveMapCloud("selected_global", *global_map, world_frame);
        lio_->data_capture_.appendGlobalRow(
            "map_output.csv",
            "production_path,loop_closing_enabled,lio_points,optimized_points,selected_points",
            save_path + "/global.pcd," + (options_.with_loop_closing_ ? "1," : "0,") +
                std::to_string(lio_map->size()) + "," + std::to_string(optimized_map->size()) + "," +
                std::to_string(global_map->size()));
    }
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LOG(ERROR) << "failed to write map.yaml";
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
