//
// Created by xiang on 25-3-24.
//

#ifndef LIGHTNING_ROS_UTILS_H
#define LIGHTNING_ROS_UTILS_H

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include "common/point_def.h"

namespace lightning {

inline double ToSec(const builtin_interfaces::msg::Time &time) { return double(time.sec) + 1e-9 * time.nanosec; }
inline uint64_t ToNanoSec(const builtin_interfaces::msg::Time &time) { return time.sec * 1e9 + time.nanosec; }

inline std::vector<std::string> NonRosArguments(int argc, const char *const argv[]) {
    return rclcpp::remove_ros_arguments(argc, argv);
}

}  // namespace lightning

#endif  // LIGHTNING_ROS_UTILS_H
