#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <plotjuggler_msgs/msg/statistics_names.hpp>
#include <plotjuggler_msgs/msg/statistics_values.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

#include "common/point_def.h"
#include "common/eigen_types.h"

namespace lightning {

/// Optional ROS debug output for human inspection. When both switches are
/// false, no publishers are created and call sites return before copying data.
class DebugVisualization {
   public:
    struct AttitudeDegrees {
        double imu_roll_deg = 0.0;
        double imu_pitch_deg = 0.0;
        double imu_yaw_deg = 0.0;
        double base_roll_deg = 0.0;
        double base_pitch_deg = 0.0;
        double base_yaw_deg = 0.0;
    };

    struct Params {
        bool live_cloud_enabled = false;
        int live_cloud_every_n_frames = 10;
        bool live_timeseries_enabled = false;
        int live_timeseries_every_n_frames = 1;
        std::size_t max_cloud_points = 50000;
        std::size_t max_correspondence_markers = 2000;
    };

    using Metrics = std::vector<std::pair<std::string, double>>;

    DebugVisualization(const Params& params, rclcpp::Node::SharedPtr node);

    static void Validate(const Params& params);
    static bool ShouldPublishCloud(const Params& params, std::uint64_t frame_id, bool force);
    static bool ShouldPublishTimeseries(const Params& params, std::uint64_t frame_id, bool force);
    static AttitudeDegrees ToAttitudeDegrees(
        const Mat3d& R_world_imu, const Mat3d& R_imu_base);
    static Metrics AttitudeMetrics(
        const std::string& phase, const Mat3d& R_world_imu, const Mat3d& R_imu_base);

    bool cloudEnabled(std::uint64_t frame_id, bool force = false) const;
    bool timeseriesEnabled(std::uint64_t frame_id, bool force = false) const;

    void publishCloud(const std::string& topic_suffix, const PointCloudType& cloud,
                      const std::string& coordinate_frame, double timestamp,
                      std::uint64_t frame_id, bool force = false);
    void publishMetrics(const Metrics& metrics, double timestamp,
                        std::uint64_t frame_id, bool force = false);
    void publishPath(const std::string& topic_suffix, const std::vector<SE3>& poses,
                     const std::string& coordinate_frame, double timestamp,
                     std::uint64_t frame_id, bool force = false);

    const Params& params() const { return params_; }

   private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr CloudPublisher(
        const std::string& topic_suffix);
    void PublishNames(double timestamp);

    Params params_;
    rclcpp::Node::SharedPtr node_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        cloud_publishers_;
    rclcpp::Publisher<plotjuggler_msgs::msg::StatisticsNames>::SharedPtr names_publisher_;
    rclcpp::Publisher<plotjuggler_msgs::msg::StatisticsValues>::SharedPtr values_publisher_;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
        scalar_publishers_;
    std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>
        path_publishers_;
    std::unordered_map<std::string, std::uint16_t> metric_indices_;
    std::vector<std::string> metric_names_;
    std::uint32_t names_version_ = 0;
    std::mutex mutex_;
};

}  // namespace lightning
