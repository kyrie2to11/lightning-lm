#include "core/system/localization_tf.h"

#include <rclcpp/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cmath>
#include <cstdint>

namespace lightning {

MapToOdomResult SelectMapToOdomTransform(
    const geometry_msgs::msg::TransformStamped& map_to_base,
    const std::optional<geometry_msgs::msg::TransformStamped>& exact_odom_to_base,
    const std::optional<geometry_msgs::msg::TransformStamped>& latest_odom_to_base,
    double max_latest_age_sec) {
    MapToOdomResult result;
    const geometry_msgs::msg::TransformStamped* odom_to_base = nullptr;

    if (exact_odom_to_base) {
        result.status = MapToOdomStatus::EXACT;
        odom_to_base = &*exact_odom_to_base;
    } else if (latest_odom_to_base) {
        const auto timestamp_delta =
            rclcpp::Time(map_to_base.header.stamp) -
            rclcpp::Time(latest_odom_to_base->header.stamp);
        const std::int64_t odom_tf_age_ns = std::abs(timestamp_delta.nanoseconds());
        const std::int64_t max_latest_age_ns =
            static_cast<std::int64_t>(std::llround(max_latest_age_sec * 1e9));
        result.odom_tf_age_sec = static_cast<double>(odom_tf_age_ns) * 1e-9;
        if (odom_tf_age_ns > max_latest_age_ns) {
            result.status = MapToOdomStatus::STALE;
            return result;
        }
        result.status = MapToOdomStatus::LATEST;
        odom_to_base = &*latest_odom_to_base;
    } else {
        return result;
    }

    const Eigen::Isometry3d T_map_base = tf2::transformToEigen(map_to_base.transform);
    const Eigen::Isometry3d T_odom_base = tf2::transformToEigen(odom_to_base->transform);
    const Eigen::Isometry3d T_map_odom = T_map_base * T_odom_base.inverse();

    result.transform.header.frame_id = "map";
    result.transform.header.stamp = map_to_base.header.stamp;
    result.transform.child_frame_id = "odom";
    result.transform.transform = tf2::eigenToTransform(T_map_odom).transform;
    return result;
}

}  // namespace lightning
