//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <cmath>
#include <thread>

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() { loc_->Finish(); }

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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

    initial_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            if (msg->header.frame_id != "map") {
                LOG(WARNING) << "Ignoring /initialpose in frame '" << msg->header.frame_id
                             << "'; expected 'map'";
                return;
            }

            const auto& pose = msg->pose.pose;
            Vec3d translation(pose.position.x, pose.position.y, pose.position.z);
            Eigen::Quaterniond quaternion(
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            if (!translation.allFinite() || !std::isfinite(quaternion.norm()) ||
                quaternion.norm() < 1e-6) {
                LOG(WARNING) << "Ignoring invalid /initialpose";
                return;
            }

            quaternion.normalize();
            SetInitPose(SE3(quaternion, translation));
            LOG(INFO) << "Accepted /initialpose in map frame";
        });

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        loc_->SetTFCallback(
            [this](const geometry_msgs::msg::TransformStamped &loc_tf) {
                // loc_tf is map -> base_footprint from localization
                // We need to compute map -> odom by subtracting odom -> base_footprint
                geometry_msgs::msg::TransformStamped map_to_odom;
                map_to_odom.header.frame_id = "map";
                map_to_odom.header.stamp = loc_tf.header.stamp;
                map_to_odom.child_frame_id = "odom";

                try {
                    auto odom_to_base = tf_buffer_->lookupTransform(
                        "odom", "base_footprint", loc_tf.header.stamp,
                        rclcpp::Duration::from_seconds(0.1));

                    // T_map_odom = T_map_base * T_base_odom
                    // T_map_odom = T_map_base * inv(T_odom_base)
                    Eigen::Isometry3d T_map_base = tf2::transformToEigen(loc_tf.transform);
                    Eigen::Isometry3d T_odom_base = tf2::transformToEigen(odom_to_base.transform);
                    Eigen::Isometry3d T_map_odom = T_map_base * T_odom_base.inverse();

                    map_to_odom.transform = tf2::eigenToTransform(T_map_odom).transform;
                } catch (const tf2::TransformException &ex) {
                    // Fallback: if odom not available, publish map -> base_footprint directly
                    LOG(WARNING) << "TF lookup failed: " << ex.what() << ", publishing map->odom as identity";
                    map_to_odom.transform.translation.x = 0;
                    map_to_odom.transform.translation.y = 0;
                    map_to_odom.transform.translation.z = 0;
                    map_to_odom.transform.rotation.w = 1;
                    map_to_odom.transform.rotation.x = 0;
                    map_to_odom.transform.rotation.y = 0;
                    map_to_odom.transform.rotation.z = 0;
                }
                tf_broadcaster_->sendTransform(map_to_odom);
            });
    }

    bool ret = loc_->Init(yaml_path, map_path);
    const auto config = YAML::LoadFile(yaml_path);
    const auto fasterlio = config["fasterlio"];
    const bool extrinsic_from_tf =
        fasterlio["extrinsic_from_tf"] && fasterlio["extrinsic_from_tf"].as<bool>();
    if (ret && extrinsic_from_tf) {
        if (!tf_buffer_) {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

        const std::string lidar_frame_id = fasterlio["lidar_frame_id"].as<std::string>("");
        const std::string imu_frame_id = fasterlio["imu_frame_id"].as<std::string>("");
        auto wait_for_transform = [&](const std::string& target, const std::string& source) {
            for (int i = 0; i < 50; ++i) {
                if (tf_buffer_->canTransform(target, source, tf2::TimePointZero)) return true;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            return tf_buffer_->canTransform(target, source, tf2::TimePointZero);
        };

        try {
            if (lidar_frame_id.empty() || imu_frame_id.empty() ||
                !wait_for_transform(imu_frame_id, lidar_frame_id)) {
                LOG(ERROR) << "Failed to resolve localization LIO extrinsic TF "
                           << imu_frame_id << " <- " << lidar_frame_id;
                ret = false;
            } else {
                const auto transform = tf_buffer_->lookupTransform(
                    imu_frame_id, lidar_frame_id, tf2::TimePointZero,
                    tf2::durationFromSec(0.5));
                const Eigen::Isometry3d T_imu_lidar = tf2::transformToEigen(transform.transform);
                loc_->SetLidarExtrinsic(T_imu_lidar.translation(), T_imu_lidar.rotation());
                LOG(INFO) << "Loaded localization LIO extrinsic from TF: " << imu_frame_id
                          << " <- " << lidar_frame_id;

                const bool init_world_from_tf =
                    fasterlio["init_world_from_tf"] && fasterlio["init_world_from_tf"].as<bool>();
                if (init_world_from_tf) {
                    const std::string world_frame_id =
                        fasterlio["world_frame_id"].as<std::string>("");
                    const std::string initialization_frame_id =
                        fasterlio["initialization_frame_id"]
                            ? fasterlio["initialization_frame_id"].as<std::string>()
                            : world_frame_id;
                    if (initialization_frame_id.empty() ||
                        !wait_for_transform(initialization_frame_id, imu_frame_id)) {
                        LOG(ERROR) << "Failed to resolve localization initial world TF "
                                   << initialization_frame_id << " <- " << imu_frame_id;
                        ret = false;
                    } else {
                        const auto world_transform = tf_buffer_->lookupTransform(
                            initialization_frame_id, imu_frame_id, tf2::TimePointZero,
                            tf2::durationFromSec(0.5));
                        const Eigen::Isometry3d T_world_imu =
                            tf2::transformToEigen(world_transform.transform);
                        loc_->SetInitialWorldImuRotation(T_world_imu.rotation());
                        LOG(INFO) << "Loaded localization initial world rotation from TF: "
                                  << initialization_frame_id << " <- " << imu_frame_id;
                    }
                }
            }
        } catch (const tf2::TransformException& ex) {
            LOG(ERROR) << "Failed to configure localization LIO from TF: " << ex.what();
            ret = false;
        }
    }
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
