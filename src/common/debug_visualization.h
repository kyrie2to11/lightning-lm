#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <plotjuggler_msgs/msg/data_points.hpp>
#include <plotjuggler_msgs/msg/dictionary.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/point_def.h"
#include "common/eigen_types.h"

namespace lightning {

/// Optional ROS debug output for human inspection. When both switches are
/// false, no publishers are created and call sites return before copying data.
class DebugVisualization {
   public:
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
    void PublishDictionary();

    Params params_;
    rclcpp::Node::SharedPtr node_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        cloud_publishers_;
    rclcpp::Publisher<plotjuggler_msgs::msg::Dictionary>::SharedPtr dictionary_publisher_;
    rclcpp::Publisher<plotjuggler_msgs::msg::DataPoints>::SharedPtr metrics_publisher_;
    std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>
        path_publishers_;
    std::unordered_map<std::string, std::uint16_t> metric_indices_;
    std::vector<std::string> metric_names_;
    std::uint32_t dictionary_uuid_ = 1;
    std::mutex mutex_;
};

}  // namespace lightning
