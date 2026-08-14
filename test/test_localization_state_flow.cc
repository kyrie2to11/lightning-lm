#include <gtest/gtest.h>

#include "core/localization/localization_state_flow.h"

namespace lightning::loc {
namespace {

NavState MakeState(double timestamp, double x, bool valid = true) {
    NavState state;
    state.timestamp_ = timestamp;
    state.pose_is_ok_ = valid;
    state.SetPose(SE3(Eigen::Quaterniond::Identity(), Vec3d(x, 0.0, 0.0)));
    return state;
}

TEST(LocalizationInitializationPolicy, ExternalPosePreventsAutomaticFallback) {
    EXPECT_FALSE(ShouldTryAutomaticInitialization(true));
    EXPECT_TRUE(ShouldTryAutomaticInitialization(false));
}

TEST(LocalizationHighFrequencyState, RebaseUsesLatestReplayedImuState) {
    const NavState lidar_state = MakeState(10.0, 1.0);
    const NavState replayed_imu_state = MakeState(10.4, 2.0);

    const NavState selected = SelectHighFrequencyRebaseState(lidar_state, replayed_imu_state);

    EXPECT_DOUBLE_EQ(selected.timestamp_, 10.4);
    EXPECT_DOUBLE_EQ(selected.GetPose().translation().x(), 2.0);
}

TEST(LocalizationHighFrequencyState, RebaseRejectsInvalidOrOlderImuState) {
    const NavState lidar_state = MakeState(10.0, 1.0);

    EXPECT_DOUBLE_EQ(
        SelectHighFrequencyRebaseState(lidar_state, MakeState(10.4, 2.0, false)).timestamp_,
        10.0);
    EXPECT_DOUBLE_EQ(
        SelectHighFrequencyRebaseState(lidar_state, MakeState(9.9, 2.0)).timestamp_,
        10.0);
}

TEST(LocalizationOutputTimestamp, RejectsDuplicateAndRegressingResults) {
    MonotonicTimestampGate gate;

    EXPECT_TRUE(gate.Accept(10.0));
    EXPECT_FALSE(gate.Accept(9.9));
    EXPECT_FALSE(gate.Accept(10.0));
    EXPECT_TRUE(gate.Accept(10.1));

    gate.Reset();
    EXPECT_TRUE(gate.Accept(1.0));
}

TEST(LocalizationHighFrequencyState, LateDrCannotReplaceLidarRebase) {
    EXPECT_FALSE(IsStrictlyNewerTimestamp(10.0, 10.0));
    EXPECT_FALSE(IsStrictlyNewerTimestamp(9.9, 10.0));
    EXPECT_TRUE(IsStrictlyNewerTimestamp(10.1, 10.0));
}

TEST(LocalizationOutputTimestamp, RejectsNonFiniteResults) {
    MonotonicTimestampGate gate;

    EXPECT_FALSE(gate.Accept(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(gate.Accept(std::numeric_limits<double>::infinity()));
    EXPECT_TRUE(gate.Accept(10.0));
}

}  // namespace
}  // namespace lightning::loc
