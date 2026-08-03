#include "common/debug_visualization.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pcl_conversions/pcl_conversions.h>

namespace lightning {
namespace {

Vec3d ZyxDegrees(const Mat3d& rotation) {
    constexpr double kRadiansToDegrees = 180.0 / M_PI;
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const double pitch = std::atan2(
        -rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    return Vec3d(roll, pitch, yaw) * kRadiansToDegrees;
}

}  // namespace

DebugVisualization::DebugVisualization(const Params& params, rclcpp::Node::SharedPtr node)
    : params_(params), node_(std::move(node)) {
    Validate(params_);
    if ((params_.live_cloud_enabled || params_.live_timeseries_enabled) && !node_) {
        throw std::invalid_argument("DebugVisualization requires a ROS node when enabled");
    }
    if (params_.live_timeseries_enabled) {
        auto dictionary_qos = rclcpp::QoS(1).reliable().transient_local();
        dictionary_publisher_ = node_->create_publisher<plotjuggler_msgs::msg::Dictionary>(
            "/lightning/debug/metrics/dictionary", dictionary_qos);
        metrics_publisher_ = node_->create_publisher<plotjuggler_msgs::msg::DataPoints>(
            "/lightning/debug/metrics/data", rclcpp::QoS(10).best_effort());
    }
}

void DebugVisualization::Validate(const Params& params) {
    if (params.live_cloud_enabled && params.live_cloud_every_n_frames < 1) {
        throw std::invalid_argument("live_cloud_every_n_frames must be >= 1");
    }
    if (params.live_timeseries_enabled && params.live_timeseries_every_n_frames < 1) {
        throw std::invalid_argument("live_timeseries_every_n_frames must be >= 1");
    }
}

bool DebugVisualization::ShouldPublishCloud(
    const Params& params, std::uint64_t frame_id, bool force) {
    return params.live_cloud_enabled &&
           (force || frame_id % static_cast<std::uint64_t>(params.live_cloud_every_n_frames) == 0);
}

bool DebugVisualization::ShouldPublishTimeseries(
    const Params& params, std::uint64_t frame_id, bool force) {
    return params.live_timeseries_enabled &&
           (force || frame_id % static_cast<std::uint64_t>(params.live_timeseries_every_n_frames) == 0);
}

DebugVisualization::AttitudeDegrees DebugVisualization::ToAttitudeDegrees(
    const Mat3d& R_world_imu, const Mat3d& R_imu_base) {
    const Vec3d imu = ZyxDegrees(R_world_imu);
    const Vec3d base = ZyxDegrees(R_world_imu * R_imu_base);
    return {imu.x(), imu.y(), imu.z(), base.x(), base.y(), base.z()};
}

DebugVisualization::Metrics DebugVisualization::AttitudeMetrics(
    const std::string& phase, const Mat3d& R_world_imu, const Mat3d& R_imu_base) {
    const AttitudeDegrees attitude = ToAttitudeDegrees(R_world_imu, R_imu_base);
    const std::string prefix = "state/" + phase + "/";
    return {{prefix + "imu_roll_deg", attitude.imu_roll_deg},
            {prefix + "imu_pitch_deg", attitude.imu_pitch_deg},
            {prefix + "imu_yaw_deg", attitude.imu_yaw_deg},
            {prefix + "base_roll_deg", attitude.base_roll_deg},
            {prefix + "base_pitch_deg", attitude.base_pitch_deg},
            {prefix + "base_yaw_deg", attitude.base_yaw_deg}};
}

bool DebugVisualization::cloudEnabled(std::uint64_t frame_id, bool force) const {
    return ShouldPublishCloud(params_, frame_id, force);
}

bool DebugVisualization::timeseriesEnabled(std::uint64_t frame_id, bool force) const {
    return ShouldPublishTimeseries(params_, frame_id, force);
}

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
DebugVisualization::CloudPublisher(const std::string& topic_suffix) {
    const std::string topic = "/lightning/debug/" + topic_suffix;
    auto found = cloud_publishers_.find(topic);
    if (found != cloud_publishers_.end()) {
        return found->second;
    }
    auto publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::QoS(1).best_effort());
    cloud_publishers_.emplace(topic, publisher);
    return publisher;
}

void DebugVisualization::publishCloud(
    const std::string& topic_suffix, const PointCloudType& cloud,
    const std::string& coordinate_frame, double timestamp,
    std::uint64_t frame_id, bool force) {
    if (!cloudEnabled(frame_id, force)) {
        return;
    }

    PointCloudType display;
    const PointCloudType* source = &cloud;
    if (params_.max_cloud_points > 0 && cloud.size() > params_.max_cloud_points) {
        display.reserve(params_.max_cloud_points);
        const double stride = static_cast<double>(cloud.size()) /
                              static_cast<double>(params_.max_cloud_points);
        for (std::size_t i = 0; i < params_.max_cloud_points; ++i) {
            display.push_back(cloud[static_cast<std::size_t>(std::floor(i * stride))]);
        }
        display.width = static_cast<std::uint32_t>(display.size());
        display.height = 1;
        display.is_dense = cloud.is_dense;
        source = &display;
    }

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(*source, message);
    message.header.frame_id = coordinate_frame;
    const auto nanoseconds = static_cast<std::int64_t>(timestamp * 1e9);
    message.header.stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);

    std::lock_guard<std::mutex> lock(mutex_);
    CloudPublisher(topic_suffix)->publish(message);
}

void DebugVisualization::PublishDictionary() {
    plotjuggler_msgs::msg::Dictionary dictionary;
    dictionary.dictionary_uuid = dictionary_uuid_;
    dictionary.names = metric_names_;
    dictionary_publisher_->publish(dictionary);
}

void DebugVisualization::publishMetrics(
    const Metrics& metrics, double timestamp, std::uint64_t frame_id, bool force) {
    if (!timeseriesEnabled(frame_id, force) || metrics.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    bool dictionary_changed = false;
    for (const auto& [name, value] : metrics) {
        (void)value;
        if (metric_indices_.find(name) == metric_indices_.end()) {
            if (metric_names_.size() >= std::numeric_limits<std::uint16_t>::max()) {
                throw std::runtime_error("too many Lightning debug metric names");
            }
            const auto index = static_cast<std::uint16_t>(metric_names_.size());
            metric_indices_.emplace(name, index);
            metric_names_.push_back(name);
            dictionary_changed = true;
        }
    }
    if (dictionary_changed) {
        ++dictionary_uuid_;
        PublishDictionary();
    }

    plotjuggler_msgs::msg::DataPoints message;
    message.dictionary_uuid = dictionary_uuid_;
    message.samples.reserve(metrics.size());
    for (const auto& [name, value] : metrics) {
        plotjuggler_msgs::msg::DataPoint sample;
        sample.name_index = metric_indices_.at(name);
        sample.stamp = timestamp;
        sample.value = value;
        message.samples.push_back(sample);
    }
    metrics_publisher_->publish(message);
}

void DebugVisualization::publishPath(
    const std::string& topic_suffix, const std::vector<SE3>& poses,
    const std::string& coordinate_frame, double timestamp,
    std::uint64_t frame_id, bool force) {
    if (!cloudEnabled(frame_id, force)) {
        return;
    }

    const auto nanoseconds = static_cast<std::int64_t>(timestamp * 1e9);
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);

    nav_msgs::msg::Path path;
    path.header.frame_id = coordinate_frame;
    path.header.stamp = stamp;
    path.poses.reserve(poses.size());
    for (const auto& pose : poses) {
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header = path.header;
        const auto translation = pose.translation();
        const Eigen::Quaterniond rotation(pose.so3().unit_quaternion());
        stamped.pose.position.x = translation.x();
        stamped.pose.position.y = translation.y();
        stamped.pose.position.z = translation.z();
        stamped.pose.orientation.w = rotation.w();
        stamped.pose.orientation.x = rotation.x();
        stamped.pose.orientation.y = rotation.y();
        stamped.pose.orientation.z = rotation.z();
        path.poses.push_back(stamped);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const std::string topic = "/lightning/debug/" + topic_suffix;
    auto& publisher = path_publishers_[topic];
    if (!publisher) {
        publisher = node_->create_publisher<nav_msgs::msg::Path>(
            topic, rclcpp::QoS(1).reliable().transient_local());
    }
    publisher->publish(path);
}

}  // namespace lightning
