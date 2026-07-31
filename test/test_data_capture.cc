#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>

#include "common/data_capture.h"

namespace lightning {
namespace {

class DataCaptureTest : public ::testing::Test {
   protected:
    void SetUp() override {
        const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
        output_dir_ = std::filesystem::temp_directory_path() /
                      ("lightning_data_capture_test_" + std::to_string(nonce));
    }

    void TearDown() override { std::filesystem::remove_all(output_dir_); }

    DataCapture::Params EnabledParams() const {
        DataCapture::Params params;
        params.enabled = true;
        params.output_dir = output_dir_.string();
        params.every_n_frames = 2;
        params.frame_start = 0;
        params.frame_end = -1;
        return params;
    }

    PointCloudType OnePointCloud() const {
        PointCloudType cloud;
        PointType point;
        point.x = 1.0F;
        point.y = 2.0F;
        point.z = 3.0F;
        point.intensity = 4.0F;
        point.time = 0.0;
        cloud.push_back(point);
        return cloud;
    }

    std::filesystem::path output_dir_;
};

TEST_F(DataCaptureTest, DisabledCaptureCreatesNoOutputDirectory) {
    DataCapture capture;
    DataCapture::Params params;
    params.enabled = false;
    params.output_dir = output_dir_.string();

    capture.configure(params);
    const auto frame = capture.beginProcessedFrame(1.0, 1.0);
    capture.saveFrontendCloud(frame, "00_input", OnePointCloud(), "base_footprint");

    EXPECT_FALSE(std::filesystem::exists(output_dir_));
}

TEST_F(DataCaptureTest, FrameStrideUsesProcessedFrameIdsStartingAtZero) {
    DataCapture capture;
    capture.configure(EnabledParams());

    const auto frame0 = capture.beginProcessedFrame(1.0, 1.0);
    const auto frame1 = capture.beginProcessedFrame(2.0, 2.0);
    const auto frame2 = capture.beginProcessedFrame(3.0, 3.0);

    EXPECT_EQ(frame0.id, 0U);
    EXPECT_TRUE(frame0.sampled);
    EXPECT_EQ(frame1.id, 1U);
    EXPECT_FALSE(frame1.sampled);
    EXPECT_EQ(frame2.id, 2U);
    EXPECT_TRUE(frame2.sampled);
}

TEST_F(DataCaptureTest, FrameBoundsAreInclusive) {
    auto params = EnabledParams();
    params.every_n_frames = 1;
    params.frame_start = 1;
    params.frame_end = 2;

    DataCapture capture;
    capture.configure(params);

    EXPECT_FALSE(capture.beginProcessedFrame(0.0, 0.0).sampled);
    EXPECT_TRUE(capture.beginProcessedFrame(1.0, 1.0).sampled);
    EXPECT_TRUE(capture.beginProcessedFrame(2.0, 2.0).sampled);
    EXPECT_FALSE(capture.beginProcessedFrame(3.0, 3.0).sampled);
}

TEST_F(DataCaptureTest, FrontendCloudUsesStablePaddedFramePath) {
    DataCapture capture;
    capture.configure(EnabledParams());

    const auto frame = capture.beginProcessedFrame(1.0, 1.0);
    capture.saveFrontendCloud(frame, "00_input", OnePointCloud(), "base_footprint");

    EXPECT_TRUE(std::filesystem::exists(
        output_dir_ / "frontend" / "frame_000000" / "00_input.pcd"));
    EXPECT_TRUE(std::filesystem::exists(
        output_dir_ / "frontend" / "frame_000000" / "metadata.csv"));
}

TEST_F(DataCaptureTest, BackendEventPathDoesNotDependOnFrontendFrame) {
    DataCapture capture;
    capture.configure(EnabledParams());

    capture.beginProcessedFrame(1.0, 1.0);
    capture.beginProcessedFrame(2.0, 2.0);
    capture.saveBackendCloud(
        "loop_000080_000120", "source_body", OnePointCloud(), "rear_lidar_imu");

    EXPECT_TRUE(std::filesystem::exists(
        output_dir_ / "backend" / "loop_000080_000120" / "source_body.pcd"));
}

TEST_F(DataCaptureTest, IterationCloudUsesNestedIterationDirectory) {
    DataCapture capture;
    capture.configure(EnabledParams());

    const auto frame = capture.beginProcessedFrame(1.0, 1.0);
    capture.saveFrontendIterationCloud(
        frame, 3, "scan_world", OnePointCloud(), "world");
    capture.appendFrontendIterationRow(
        frame, 3, "observation.csv", "value", "42");

    EXPECT_TRUE(std::filesystem::exists(
        output_dir_ / "frontend" / "frame_000000" /
        "iteration_03" / "scan_world.pcd"));
    EXPECT_TRUE(std::filesystem::exists(
        output_dir_ / "frontend" / "frame_000000" /
        "iteration_03" / "observation.csv"));
}

}  // namespace
}  // namespace lightning
