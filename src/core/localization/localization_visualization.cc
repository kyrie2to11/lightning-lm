#include "core/localization/localization_visualization.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "core/lightning_math.hpp"

namespace lightning::loc {

sensor_msgs::msg::PointCloud2 MakeAlignedScanMessage(
    const PointCloudType& scan, const SE3& map_from_scan, double timestamp) {
    PointCloudType aligned_scan;
    pcl::transformPointCloud(scan, aligned_scan, map_from_scan.matrix());

    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned_scan, message);
    message.header.frame_id = "map";
    message.header.stamp = math::FromSec(timestamp);
    return message;
}

}  // namespace lightning::loc
