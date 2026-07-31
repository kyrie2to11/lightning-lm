#include <gtest/gtest.h>

#include "common/constant.h"
#include "common/measure_group.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_processing.hpp"

namespace {

lightning::IMUPtr MakeImu(double timestamp, const lightning::Vec3d& acc) {
    auto imu = std::make_shared<lightning::IMU>();
    imu->timestamp = timestamp;
    imu->linear_acceleration = acc;
    imu->angular_velocity = lightning::Vec3d::Zero();
    return imu;
}

lightning::NavState RunInit(lightning::ImuProcess& process, const lightning::Vec3d& acc) {
    lightning::MeasureGroup meas;
    for (int i = 0; i < 25; ++i) {
        meas.imu_.push_back(MakeImu(0.01 * i, acc));
    }
    meas.lidar_begin_time_ = 0.0;
    meas.lidar_end_time_ = 0.24;
    meas.scan_ = std::make_shared<lightning::PointCloudType>();

    lightning::ESKF eskf;
    lightning::CloudPtr scan(new lightning::PointCloudType());
    process.Process(meas, eskf, scan, false);
    return eskf.GetX();
}

}  // namespace

TEST(ImuInitialization, DefaultKeepsMeanAccelerationGravityDirection) {
    lightning::ImuProcess process;
    const auto state = RunInit(process, lightning::Vec3d(0.03, 9.81, 0.45));

    EXPECT_NEAR(state.grav_.norm(), lightning::G_m_s2, 1e-6);
    EXPECT_LT(state.grav_.y(), -9.0);
}

TEST(ImuInitialization, TfAlignedModeUsesConfiguredWorldGravityAndRotation) {
    lightning::ImuProcess process;

    lightning::Mat3d R_world_imu;
    R_world_imu << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0;
    process.SetInitialWorldImuRotation(R_world_imu);

    // R_world_imu maps IMU +X to world +Z, so a stationary, consistently
    // framed accelerometer sample points along IMU +X.
    const auto state = RunInit(process, lightning::Vec3d(9.81, 0.0, 0.0));

    EXPECT_NEAR(state.grav_.x(), 0.0, 1e-9);
    EXPECT_NEAR(state.grav_.y(), 0.0, 1e-9);
    EXPECT_NEAR(state.grav_.z(), -lightning::G_m_s2, 1e-9);
    EXPECT_TRUE(state.rot_.matrix().isApprox(R_world_imu, 1e-9));
}

TEST(ImuInitialization, ExposesCaptureSnapshotWithoutChangingInitialization) {
    lightning::ImuProcess process;
    const lightning::Vec3d acceleration(0.03, 9.81, 0.45);
    RunInit(process, acceleration);

    const auto snapshot = process.GetInitializationSnapshot();

    EXPECT_GT(snapshot.sample_count, 20);
    EXPECT_TRUE(snapshot.mean_acc.isApprox(acceleration, 1e-9));
    EXPECT_TRUE(snapshot.mean_gyr.isApprox(lightning::Vec3d::Zero(), 1e-9));
    EXPECT_EQ(snapshot.initialized, process.IsIMUInited());
}
