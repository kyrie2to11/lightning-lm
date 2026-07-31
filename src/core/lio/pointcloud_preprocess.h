#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/measure_group.h"
#include "common/point_def.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

enum class LidarType { AVIA = 1, VELO32 = 2, OUST64 = 3, ROBOSENSE = 4, POLKA_MERGED = 5 };

/**
 * point cloud preprocess
 * just unify the point format from livox/velodyne to PCL
 *
 * 预处理程序
 * 主要是对各种不同的雷达处理时间戳差异
 */
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Stats {
        std::size_t input_points = 0;
        std::size_t stride_rejected = 0;
        std::size_t range_rejected = 0;
        std::size_t height_rejected = 0;
        std::size_t output_points = 0;
    };

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// processors
    void Process(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg, PointCloudType::Ptr &pcl_out);

    void Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud, PointCloudType::Ptr &pcl_out);

    void Set(LidarType lid_type, double bld, int pfilt_num);

    // accessors
    double &Blind() { return blind_; }
    int &NumScans() { return num_scans_; }
    int &PointFilterNum() { return point_filter_num_; }
    float &TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }
    bool InputIsPredeskewed() const { return lidar_type_ == LidarType::POLKA_MERGED; }
    const Stats& GetLastStats() const { return last_stats_; }

    void SetHeightROI(float height_max, float height_min) {
        height_max_ = height_max;
        height_min_ = height_min;
    }
    void SetExpectedFrame(const std::string &frame) { expected_frame_ = frame; }

   private:
    void Oust64Handler(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg);
    void RoboSenseHandler(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg);
    void VelodyneHandler(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg);
    void PolkaMergedHandler(const sensor_msgs::msg::PointCloud2 ::SharedPtr &msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    float time_scale_ = 1e-3;
    bool given_offset_time_ = false;

    float height_max_ = 1.0;
    float height_min_ = -1.0;

    std::string expected_frame_ = "base_footprint";
    Stats last_stats_;
};
}  // namespace lightning

#endif
