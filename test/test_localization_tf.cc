#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "core/system/localization_tf.h"

namespace lightning {
namespace {

geometry_msgs::msg::TransformStamped MakeTransform(
    double timestamp, double x, double y, double yaw = 0.0) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp.sec = static_cast<std::int32_t>(std::floor(timestamp));
    transform.header.stamp.nanosec = static_cast<std::uint32_t>(
        std::llround((timestamp - std::floor(timestamp)) * 1e9));
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.rotation.z = std::sin(yaw * 0.5);
    transform.transform.rotation.w = std::cos(yaw * 0.5);
    return transform;
}

TEST(LocalizationTf, UsesExactOdomTransformAndPreservesLocalizationStamp) {
    auto map_to_base = MakeTransform(10.0, 5.0, 3.0);
    map_to_base.header.frame_id = "map";
    map_to_base.child_frame_id = "base_footprint";
    auto exact_odom_to_base = MakeTransform(10.0, 2.0, 1.0);

    const auto result = SelectMapToOdomTransform(
        map_to_base, exact_odom_to_base, std::nullopt, 0.05);

    ASSERT_EQ(result.status, MapToOdomStatus::EXACT);
    EXPECT_EQ(result.transform.header.frame_id, "map");
    EXPECT_EQ(result.transform.child_frame_id, "odom");
    EXPECT_EQ(result.transform.header.stamp, map_to_base.header.stamp);
    EXPECT_NEAR(result.transform.transform.translation.x, 3.0, 1e-9);
    EXPECT_NEAR(result.transform.transform.translation.y, 2.0, 1e-9);
}

TEST(LocalizationTf, AcceptsLatestTransformAtOrInsideAgeLimit) {
    const auto map_to_base = MakeTransform(10.0, 5.0, 3.0);

    for (const double age : {0.049, 0.050, -0.050}) {
        const auto latest_odom_to_base = MakeTransform(10.0 - age, 2.0, 1.0);
        const auto result = SelectMapToOdomTransform(
            map_to_base, std::nullopt, latest_odom_to_base, 0.05);

        EXPECT_EQ(result.status, MapToOdomStatus::LATEST);
        EXPECT_NEAR(result.odom_tf_age_sec, std::abs(age), 1e-9);
    }
}

TEST(LocalizationTf, RejectsLatestTransformOutsideAgeLimit) {
    const auto map_to_base = MakeTransform(10.0, 5.0, 3.0);

    auto one_nanosecond_over = MakeTransform(0.0, 2.0, 1.0);
    one_nanosecond_over.header.stamp.sec = 9;
    one_nanosecond_over.header.stamp.nanosec = 949999999U;
    EXPECT_EQ(
        SelectMapToOdomTransform(
            map_to_base, std::nullopt, one_nanosecond_over, 0.05).status,
        MapToOdomStatus::STALE);

    const auto one_millisecond_over = MakeTransform(9.949, 2.0, 1.0);
    const auto result = SelectMapToOdomTransform(
        map_to_base, std::nullopt, one_millisecond_over, 0.05);

    EXPECT_EQ(result.status, MapToOdomStatus::STALE);
}

TEST(LocalizationTf, RejectsWhenNeitherExactNorLatestTransformExists) {
    const auto result = SelectMapToOdomTransform(
        MakeTransform(10.0, 5.0, 3.0), std::nullopt, std::nullopt, 0.05);

    EXPECT_EQ(result.status, MapToOdomStatus::MISSING);
}

TEST(LocalizationTf, ComposesRotatedOdomTransform) {
    const auto map_to_base = MakeTransform(10.0, 3.0, 2.0, M_PI_2);
    const auto odom_to_base = MakeTransform(10.0, 1.0, 0.0, M_PI_2);

    const auto result = SelectMapToOdomTransform(
        map_to_base, odom_to_base, std::nullopt, 0.05);

    ASSERT_EQ(result.status, MapToOdomStatus::EXACT);
    EXPECT_NEAR(result.transform.transform.translation.x, 2.0, 1e-9);
    EXPECT_NEAR(result.transform.transform.translation.y, 2.0, 1e-9);
    EXPECT_NEAR(result.transform.transform.rotation.z, 0.0, 1e-9);
    EXPECT_NEAR(result.transform.transform.rotation.w, 1.0, 1e-9);
}

}  // namespace
}  // namespace lightning
