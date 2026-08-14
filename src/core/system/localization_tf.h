#ifndef LIGHTNING_LOCALIZATION_TF_H
#define LIGHTNING_LOCALIZATION_TF_H

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <optional>

namespace lightning {

enum class MapToOdomStatus { EXACT, LATEST, STALE, MISSING };

struct MapToOdomResult {
    MapToOdomStatus status = MapToOdomStatus::MISSING;
    geometry_msgs::msg::TransformStamped transform;
    double odom_tf_age_sec = 0.0;
};

MapToOdomResult SelectMapToOdomTransform(
    const geometry_msgs::msg::TransformStamped& map_to_base,
    const std::optional<geometry_msgs::msg::TransformStamped>& exact_odom_to_base,
    const std::optional<geometry_msgs::msg::TransformStamped>& latest_odom_to_base,
    double max_latest_age_sec);

}  // namespace lightning

#endif  // LIGHTNING_LOCALIZATION_TF_H
