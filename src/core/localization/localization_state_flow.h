#pragma once

#include <cmath>
#include <limits>

#include "common/nav_state.h"

namespace lightning::loc {

inline bool ShouldTryAutomaticInitialization(bool external_pose_pending) {
    return !external_pose_pending;
}

inline NavState SelectHighFrequencyRebaseState(
    const NavState& lidar_state, const NavState& replayed_imu_state) {
    if (replayed_imu_state.pose_is_ok_ &&
        replayed_imu_state.timestamp_ >= lidar_state.timestamp_) {
        return replayed_imu_state;
    }
    return lidar_state;
}

inline bool IsStrictlyNewerTimestamp(double timestamp, double last_timestamp) {
    return std::isfinite(timestamp) && timestamp > last_timestamp;
}

class MonotonicTimestampGate {
   public:
    bool Accept(double timestamp) {
        if (!IsStrictlyNewerTimestamp(timestamp, last_timestamp_)) return false;
        last_timestamp_ = timestamp;
        return true;
    }

    void Reset() { last_timestamp_ = -std::numeric_limits<double>::infinity(); }

   private:
    double last_timestamp_ = -std::numeric_limits<double>::infinity();
};

}  // namespace lightning::loc
