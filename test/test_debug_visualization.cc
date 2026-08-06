#include <cmath>
#include <chrono>
#include <optional>

#include <gtest/gtest.h>
#include <std_msgs/msg/float64.hpp>

#include "common/debug_visualization.h"

namespace lightning {
namespace {

TEST(DebugVisualizationParams, DefaultsDisableAllOnlineOutputs) {
    DebugVisualization::Params params;

    EXPECT_FALSE(params.live_cloud_enabled);
    EXPECT_FALSE(params.live_timeseries_enabled);
}

TEST(DebugVisualizationSampling, UsesIndependentFrameStrides) {
    DebugVisualization::Params params;
    params.live_cloud_enabled = true;
    params.live_cloud_every_n_frames = 10;
    params.live_timeseries_enabled = true;
    params.live_timeseries_every_n_frames = 3;

    EXPECT_TRUE(DebugVisualization::ShouldPublishCloud(params, 0, false));
    EXPECT_FALSE(DebugVisualization::ShouldPublishCloud(params, 9, false));
    EXPECT_TRUE(DebugVisualization::ShouldPublishCloud(params, 10, false));
    EXPECT_TRUE(DebugVisualization::ShouldPublishTimeseries(params, 3, false));
    EXPECT_FALSE(DebugVisualization::ShouldPublishTimeseries(params, 4, false));
}

TEST(DebugVisualizationSampling, ForcedEventsStillRespectMasterSwitch) {
    DebugVisualization::Params disabled;
    EXPECT_FALSE(DebugVisualization::ShouldPublishCloud(disabled, 1, true));
    EXPECT_FALSE(DebugVisualization::ShouldPublishTimeseries(disabled, 1, true));

    DebugVisualization::Params enabled;
    enabled.live_cloud_enabled = true;
    enabled.live_timeseries_enabled = true;
    EXPECT_TRUE(DebugVisualization::ShouldPublishCloud(enabled, 1, true));
    EXPECT_TRUE(DebugVisualization::ShouldPublishTimeseries(enabled, 1, true));
}

TEST(DebugVisualizationParams, RejectsInvalidEnabledStride) {
    DebugVisualization::Params params;
    params.live_cloud_enabled = true;
    params.live_cloud_every_n_frames = 0;

    EXPECT_THROW(DebugVisualization::Validate(params), std::invalid_argument);
}

TEST(DebugVisualizationAttitude, IdentityExtrinsicKeepsImuAndBaseEqual) {
    const Mat3d R_world_imu =
        Eigen::AngleAxisd(35.0 * M_PI / 180.0, Vec3d::UnitZ()).toRotationMatrix();

    const auto attitude =
        DebugVisualization::ToAttitudeDegrees(R_world_imu, Mat3d::Identity());

    EXPECT_NEAR(attitude.imu_yaw_deg, 35.0, 1e-9);
    EXPECT_NEAR(attitude.base_yaw_deg, 35.0, 1e-9);
}

TEST(DebugVisualizationAttitude, ComposesRuntimeImuFromBaseRotation) {
    const Mat3d R_world_imu =
        Eigen::AngleAxisd(30.0 * M_PI / 180.0, Vec3d::UnitZ()).toRotationMatrix();
    const Mat3d R_imu_base =
        Eigen::AngleAxisd(20.0 * M_PI / 180.0, Vec3d::UnitZ()).toRotationMatrix();

    const auto attitude =
        DebugVisualization::ToAttitudeDegrees(R_world_imu, R_imu_base);

    EXPECT_NEAR(attitude.imu_yaw_deg, 30.0, 1e-9);
    EXPECT_NEAR(attitude.base_yaw_deg, 50.0, 1e-9);
}

TEST(DebugVisualizationMetrics, PublishesEachMetricAsStandardScalarTopic) {
    if (!rclcpp::ok()) {
        int argc = 0;
        rclcpp::init(argc, nullptr);
    }

    auto publisher_node = std::make_shared<rclcpp::Node>("debug_metrics_publisher_test");
    auto subscriber_node = std::make_shared<rclcpp::Node>("debug_metrics_subscriber_test");
    std::optional<double> received;
    auto subscription = subscriber_node->create_subscription<std_msgs::msg::Float64>(
        "/lightning/debug/timeseries/state/predicted/base_yaw_deg", 10,
        [&received](const std_msgs::msg::Float64& message) { received = message.data; });
    (void)subscription;

    DebugVisualization::Params params;
    params.live_timeseries_enabled = true;
    DebugVisualization debug(params, publisher_node);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(publisher_node);
    executor.add_node(subscriber_node);
    debug.publishMetrics({{"state/predicted/base_yaw_deg", 12.5}}, 1.0, 0);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!received && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
    }

    ASSERT_TRUE(received.has_value());
    EXPECT_DOUBLE_EQ(*received, 12.5);
}

}  // namespace
}  // namespace lightning
