//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

namespace loc {
class Localization;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;  // 是否发布tf
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径在yaml里配置
    bool Init(const std::string& yaml_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    static constexpr std::size_t kMaxPgoPathPoses = 2000;
    static constexpr std::size_t kPgoPathTrimPoses = 500;
    static constexpr double kMaxOdomTfAgeSec = 0.05;
    static constexpr std::size_t kMaxTrajectoryPoses = 500;        // 10Hz × 50s 滚动窗口
    static constexpr std::size_t kTrajectoryTrimPoses = 100;
    static constexpr double kTrajectorySamplePeriodSec = 0.1;      // 高频输出按 10Hz 降采样

    /// 高频定位结果降采样追加到轨迹话题（与 pgo_path 的低频 PGO 轨迹互补）
    void AppendTrajectorySample(const geometry_msgs::msg::TransformStamped& loc_tf);

    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localization_map_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_scan_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pgo_path_pub_ = nullptr;
    nav_msgs::msg::Path pgo_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_ = nullptr;
    nav_msgs::msg::Path trajectory_;
    double trajectory_last_sample_sec_ = 0.0;  // 上次轨迹采样时间，用于降采样
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
