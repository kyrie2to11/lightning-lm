#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/lio/pointcloud_preprocess.h"
#include "core/lio/laser_mapping.h"
#include "wrapper/ros_utils.h"

namespace lightning {
namespace {

sensor_msgs::msg::PointCloud2::SharedPtr MakeXyziCloud(const std::string &frame_id) {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->header.frame_id = frame_id;
    msg->header.stamp.sec = 100;
    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                  sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(*msg, "intensity");
    *x = 1.0F;
    *y = 0.0F;
    *z = 0.0F;
    *intensity = 42.0F;
    return msg;
}

sensor_msgs::msg::PointCloud2::SharedPtr MakeRobosenseCloud(
    double header_time, const std::vector<double> &timestamps) {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->header.frame_id = "rear_lidar_link";
    msg->header.stamp.sec = static_cast<int32_t>(header_time);
    msg->header.stamp.nanosec =
        static_cast<uint32_t>((header_time - static_cast<double>(msg->header.stamp.sec)) * 1e9);
    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "timestamp", 1,
                                  sensor_msgs::msg::PointField::FLOAT64);
    modifier.resize(timestamps.size());
    sensor_msgs::PointCloud2Iterator<float> x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(*msg, "intensity");
    sensor_msgs::PointCloud2Iterator<double> timestamp(*msg, "timestamp");
    for (double value : timestamps) {
        *x = 1.0F;
        *y = 0.0F;
        *z = 0.0F;
        *intensity = 42.0F;
        *timestamp = value;
        ++x;
        ++y;
        ++z;
        ++intensity;
        ++timestamp;
    }
    return msg;
}

TEST(PointCloudPreprocess, RejectsRobosenseWithoutFloat64Timestamp) {
    auto msg = MakeXyziCloud("rear_lidar_link");
    PointCloudPreprocess preprocess;
    preprocess.SetLidarType(LidarType::ROBOSENSE);
    PointCloudType::Ptr output(new PointCloudType);
    EXPECT_THROW(preprocess.Process(msg, output), std::invalid_argument);
}

TEST(PointCloudPreprocess, ConvertsRobosenseAbsoluteTimestampToMilliseconds) {
    auto msg = MakeRobosenseCloud(100.0, {100.001, 100.004});
    PointCloudPreprocess preprocess;
    preprocess.SetLidarType(LidarType::ROBOSENSE);
    PointCloudType::Ptr output(new PointCloudType);
    preprocess.Process(msg, output);
    ASSERT_EQ(output->size(), 2U);
    EXPECT_NEAR(output->at(0).time, 1.0, 1e-6);
    EXPECT_NEAR(output->at(1).time, 4.0, 1e-6);
}

TEST(PointCloudPreprocess, AcceptsPredeskewedPolkaXyzi) {
    auto msg = MakeXyziCloud("base_footprint");
    PointCloudPreprocess preprocess;
    preprocess.SetLidarType(LidarType::POLKA_MERGED);
    PointCloudType::Ptr output(new PointCloudType);
    preprocess.Process(msg, output);
    ASSERT_EQ(output->size(), 1U);
    EXPECT_FLOAT_EQ(output->front().x, 1.0F);
    EXPECT_DOUBLE_EQ(output->front().time, 0.0);
    EXPECT_TRUE(preprocess.InputIsPredeskewed());
}

TEST(PointCloudPreprocess, RejectsPolkaCloudWithoutFloat32Intensity) {
    auto msg = MakeXyziCloud("base_footprint");
    msg->fields.pop_back();
    PointCloudPreprocess preprocess;
    preprocess.SetLidarType(LidarType::POLKA_MERGED);
    PointCloudType::Ptr output(new PointCloudType);
    EXPECT_THROW(preprocess.Process(msg, output), std::invalid_argument);
}

TEST(LaserMappingTiming, PredeskewedSnapshotEndsAtHeaderTime) {
    PointCloudType cloud;
    cloud.push_back(PointType{});
    EXPECT_DOUBLE_EQ(LaserMapping::ComputeLidarEndTime(42.5, cloud, 0.1, true), 42.5);
}

TEST(ArticulatedVehicleConfig, UsesPolkaAndRearAiryImu) {
    const auto yaml = YAML::LoadFile(std::string(LIGHTNING_SOURCE_DIR) + "/config/articulated_vehicle_map.yaml");
    EXPECT_EQ(yaml["common"]["lidar_topic"].as<std::string>(), "/polka/merged_cloud");
    EXPECT_EQ(yaml["common"]["imu_topic"].as<std::string>(), "/imu/airy_rear");
    EXPECT_EQ(yaml["fasterlio"]["lidar_type"].as<int>(), 5);
    EXPECT_TRUE(yaml["fasterlio"]["extrinsic_from_tf"].as<bool>());
    EXPECT_EQ(yaml["fasterlio"]["lidar_frame_id"].as<std::string>(), "base_footprint");
    EXPECT_EQ(yaml["fasterlio"]["imu_frame_id"].as<std::string>(), "rear_lidar_imu_ned");
    EXPECT_EQ(yaml["system"]["map_path"].as<std::string>(), "src/robot_navigation/maps/");
    EXPECT_EQ(yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>(),
              (std::vector<double>{-0.000744, 0.640484, -0.415733}));
    EXPECT_EQ(yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>(),
              (std::vector<double>{-0.000992, 0.999963, -0.008486,
                                   0.000420, 0.008487, 0.999964,
                                   0.999999, 0.000988, -0.000429}));
    EXPECT_FALSE(yaml["system"]["step_on_kf"].as<bool>());
}

TEST(LaserMappingExtrinsic, CanOverrideManualExtrinsic) {
    LaserMapping mapping;
    Vec3d translation(1.0, 2.0, 3.0);
    Mat3d rotation = Eigen::AngleAxisd(M_PI / 2.0, Vec3d::UnitZ()).toRotationMatrix();

    mapping.SetExtrinsic(translation, rotation);

    EXPECT_TRUE(mapping.GetExtrinsicTranslation().isApprox(translation));
    EXPECT_TRUE(mapping.GetExtrinsicRotation().isApprox(rotation));
}

TEST(CommandLine, RemovesRosArgumentsBeforeGflagsParsing) {
    const char *argv[] = {"run_slam_online", "--config=/tmp/test.yaml", "--ros-args", "-r",
                          "__node:=lightning_slam"};
    const auto args = NonRosArguments(5, argv);
    EXPECT_EQ(args, (std::vector<std::string>{"run_slam_online", "--config=/tmp/test.yaml"}));
}

}  // namespace
}  // namespace lightning
