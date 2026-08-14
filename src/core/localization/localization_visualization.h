#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning::loc {

sensor_msgs::msg::PointCloud2 MakeAlignedScanMessage(
    const PointCloudType& scan, const SE3& map_from_scan, double timestamp);

}  // namespace lightning::loc
