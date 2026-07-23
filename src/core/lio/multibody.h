#pragma once

#ifndef LIGHTNING_MULTIBODY_H
#define LIGHTNING_MULTIBODY_H

#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/point_def.h"
#include "core/lio/pose6d.h"
#include "core/lightning_math.hpp"

namespace lightning {

// ---------------------------------------------------------------------------
// Config types (parsed from YAML)
// ---------------------------------------------------------------------------

struct MultiBodyLidarConfig {
    int id = -1;
    std::string body_id;
    std::string topic;
    std::string lidar_frame;
    std::string imu_frame;  // paired IMU frame for TF extrinsic lookup
    bool is_leader = false;
    // Extrinsics T(imu_frame ← lidar_frame), loaded from TF at startup
    Mat3d R_lidar_imu = Mat3d::Identity();  // p_imu = R * p_lidar + t
    Vec3d t_lidar_imu = Vec3d::Zero();
};

struct MultiBodyImuConfig {
    int id = -1;
    std::string body_id;
    std::string topic;
};

struct MultiBodyConfig {
    bool enabled = false;
    std::string leader_body_id;
    std::string leader_imu_frame;  // TF target frame for cross-body lookups
    std::vector<MultiBodyLidarConfig> lidars;
    std::vector<MultiBodyImuConfig> imus;
    double sync_tolerance = 0.05;  // seconds — max time offset for multi-lidar grouping
    int nonleader_bias_init_frames = 20;

    // Convenience accessors
    const MultiBodyLidarConfig* findLidar(int id) const {
        for (const auto& l : lidars)
            if (l.id == id) return &l;
        return nullptr;
    }
    const MultiBodyImuConfig* findImuByBody(const std::string& body_id) const {
        for (const auto& i : imus)
            if (i.body_id == body_id) return &i;
        return nullptr;
    }
    const MultiBodyLidarConfig* findLeaderLidar() const {
        for (const auto& l : lidars)
            if (l.is_leader) return &l;
        return nullptr;
    }
};

// ---------------------------------------------------------------------------
// Runtime data structures
// ---------------------------------------------------------------------------

/// A buffered LiDAR scan
struct LidarEntry {
    CloudPtr cloud;
    double begin_time = 0;
    double end_time = 0;
};

/// Synced multi-body measurement group
struct MultiMeasureGroup {
    double lidar_begin_time = 0;
    double lidar_end_time = 0;
    /// lidar_id → buffered scan
    std::map<int, LidarEntry> lidars;
    /// imu_id → IMU samples covering [lidar_begin_time, lidar_end_time]
    std::map<int, std::deque<IMUPtr>> imus;
};

/// Lightweight seed state for non-leader manual IMU propagation (no EKF)
struct DeskewSeedState {
    Mat3d rot = Mat3d::Identity();   // world ← body
    Vec3d vel = Vec3d::Zero();
    Vec3d pos = Vec3d::Zero();
    Vec3d bg = Vec3d::Zero();        // gyro bias
    Vec3d grav = Vec3d(0, 0, -9.81); // gravity in world frame
};

// ---------------------------------------------------------------------------
// NonLeaderImuProcessor
//
// Per-non-leader-body IMU processor: accumulates bias, performs manual
// forward propagation (no EKF covariance), and deskews points using the
// resulting pose trajectory.
//
// Mirrors the colleague's pure_forward_propagation + undistort_single_lidar,
// adapted to lightning's data types and deskew formula.
// ---------------------------------------------------------------------------

class NonLeaderImuProcessor {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setExtrinsic(const Mat3d& R_lidar_imu, const Vec3d& t_lidar_imu) {
        R_lidar_imu_ = R_lidar_imu;
        t_lidar_imu_ = t_lidar_imu;
    }

    void setGravity(const Vec3d& g) { grav_ = g; }

    /// Accumulate mean gyro/acc for bias initialization.
    /// Returns true when enough samples have been collected.
    bool accumulateBias(const std::deque<IMUPtr>& imus) {
        if (bias_ready_) return true;
        for (const auto& imu : imus) {
            bias_count_++;
            mean_acc_ += (imu->linear_acceleration - mean_acc_) / bias_count_;
            mean_gyr_ += (imu->angular_velocity - mean_gyr_) / bias_count_;
            last_imu_ = imu;
        }
        if (bias_count_ >= target_count_) {
            bias_ready_ = true;
            // Determine acc scale (same logic as leader ImuProcess)
            double norm = mean_acc_.norm();
            if (norm > 0.5 && norm < 1.5) {
                acc_scale_ = 9.80665;
            } else if (norm > 7.0 && norm < 12.0) {
                acc_scale_ = 1.0;
            } else {
                acc_scale_ = 1.0;
            }
            LOG(INFO) << "[non-leader] bias ready: mean_gyr=" << mean_gyr_.transpose()
                      << " mean_acc=" << mean_acc_.transpose()
                      << " |acc|=" << norm << " acc_scale=" << acc_scale_;
        }
        return bias_ready_;
    }

    bool isBiasReady() const { return bias_ready_; }
    const Vec3d& getMeanGyr() const { return mean_gyr_; }
    const Vec3d& getMeanAcc() const { return mean_acc_; }
    const Vec3d& getAngvelLast() const { return angvel_last_; }

    /// Manual forward IMU propagation from seed state through the scan.
    /// No EKF predict — just dead reckoning.  Records imu_poses_ for deskew.
    ///
    /// @param imus        IMU samples covering (approximately) [beg_time, end_time]
    /// @param seed        Seed state (position, velocity, rotation, bias, gravity)
    /// @param seed_time   Time of the seed state
    /// @param beg_time    Scan begin time
    /// @param end_time    Scan end time
    void pureForwardPropagation(const std::deque<IMUPtr>& imus,
                                const DeskewSeedState& seed,
                                double seed_time,
                                double beg_time,
                                double end_time) {
        // Build a working IMU deque: prepend last_imu_ for continuity
        std::deque<IMUPtr> v_imu;
        if (last_imu_) v_imu.push_back(last_imu_);
        for (const auto& imu : imus) v_imu.push_back(imu);

        Mat3d rot = seed.rot;
        Vec3d pos = seed.pos;
        Vec3d vel = seed.vel;
        const Vec3d& bg = seed.bg;
        const Vec3d& grav = seed.grav;

        imu_poses_.clear();
        // Record the initial pose (offset = 0)
        imu_poses_.emplace_back(0.0, Vec3d::Zero(), Vec3d::Zero(), vel, pos, rot);

        double prev_t = seed_time;
        bool first = true;

        for (size_t i = 0; i + 1 < v_imu.size(); ++i) {
            const auto& head = v_imu[i];
            const auto& tail = v_imu[i + 1];

            // Skip intervals entirely before seed_time
            if (tail->timestamp < seed_time) {
                prev_t = tail->timestamp;
                continue;
            }

            Vec3d angvel_avr = 0.5 * (head->angular_velocity + tail->angular_velocity);
            Vec3d acc_avr = 0.5 * (head->linear_acceleration + tail->linear_acceleration);
            acc_avr *= acc_scale_;

            // Compute dt: clamp interval start to seed_time
            double interval_start = head->timestamp;
            if (first) {
                interval_start = std::max(head->timestamp, seed_time);
                first = false;
            }
            double dt = tail->timestamp - interval_start;
            if (dt <= 0) continue;
            if (dt > 0.1) continue;  // skip abnormal gaps

            // Corrected measurements
            Vec3d angvel_corrected = angvel_avr - bg;
            Vec3d acc_world = rot * acc_avr + grav;

            // State update
            pos += vel * dt + 0.5 * acc_world * dt * dt;
            vel += acc_world * dt;
            rot = rot * math::exp(angvel_corrected, dt).matrix();

            // Record pose with offset relative to scan begin (for deskew time matching)
            double offs_t = tail->timestamp - beg_time;
            Vec3d acc_s = rot * acc_avr + grav;
            imu_poses_.emplace_back(offs_t, acc_s, angvel_corrected, vel, pos, rot);

            prev_t = tail->timestamp;
            last_imu_ = tail;
        }

        // Final predict to end_time if needed
        if (prev_t < end_time && !v_imu.empty()) {
            double dt = end_time - prev_t;
            if (dt > 0 && dt < 0.1) {
                const auto& tail = v_imu.back();
                Vec3d angvel_corrected = tail->angular_velocity - bg;
                Vec3d acc_avr = tail->linear_acceleration * acc_scale_;
                Vec3d acc_world = rot * acc_avr + grav;
                pos += vel * dt + 0.5 * acc_world * dt * dt;
                vel += acc_world * dt;
                rot = rot * math::exp(angvel_corrected, dt).matrix();
            }
        }

        final_rot_ = rot;
        final_pos_ = pos;
        angvel_last_ = seed.bg;  // approximate — not used critically
    }

    /// Deskew a point cloud using the internally computed imu_poses_.
    /// Output points are in the non-leader IMU frame at scan-end time.
    ///
    /// @param cloud     In/out point cloud (points in LiDAR frame, per-point .time in ms offset)
    /// @param beg_time  Scan begin time (for reference)
    /// @param end_time  Scan end time
    void undistortLidar(CloudPtr cloud, double beg_time, double end_time) {
        if (imu_poses_.size() < 2 || cloud->empty()) return;

        // Sort points by time offset (ms)
        std::sort(cloud->points.begin(), cloud->points.end(),
                  [](const PointType& a, const PointType& b) { return a.time < b.time; });

        // Scan-end state (from pure forward propagation)
        const Mat3d& R_end = final_rot_;
        const Vec3d& pos_end = final_pos_;

        auto it_pcl = cloud->points.end() - 1;
        for (auto it_kp = imu_poses_.end() - 1; it_kp != imu_poses_.begin(); --it_kp) {
            auto head = it_kp - 1;
            auto tail = it_kp;
            const Mat3d& R_imu = head->rot;
            const Vec3d& vel_imu = head->vel;
            const Vec3d& pos_imu = head->pos;
            const Vec3d& acc_imu = tail->acc;
            const Vec3d& angvel_avr = tail->gyr;

            for (; it_pcl->time / 1000.0 > head->offset_time && it_pcl != cloud->points.begin(); --it_pcl) {
                double dt = it_pcl->time / 1000.0 - head->offset_time;
                if (dt < 0 || dt > 0.2) continue;

                Mat3d R_i(R_imu * math::exp(angvel_avr, dt).matrix());
                Vec3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - pos_end);

                Vec3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
                Vec3d p_comp = R_lidar_imu_.transpose() *
                               (R_end.transpose() * (R_i * (R_lidar_imu_ * P_i + t_lidar_imu_) + T_ei) -
                                t_lidar_imu_);

                it_pcl->x = p_comp(0);
                it_pcl->y = p_comp(1);
                it_pcl->z = p_comp(2);
            }
        }
    }

    const std::vector<Pose6D>& getImuPoses() const { return imu_poses_; }
    const Mat3d& getFinalRot() const { return final_rot_; }
    const Vec3d& getFinalPos() const { return final_pos_; }

   private:
    Mat3d R_lidar_imu_ = Mat3d::Identity();
    Vec3d t_lidar_imu_ = Vec3d::Zero();
    Vec3d grav_ = Vec3d(0, 0, -9.81);

    // Bias accumulation
    Vec3d mean_gyr_ = Vec3d::Zero();
    Vec3d mean_acc_ = Vec3d::Zero();
    int bias_count_ = 0;
    bool bias_ready_ = false;
    static constexpr int target_count_ = 20;
    double acc_scale_ = 1.0;

    // Output from pureForwardPropagation
    std::vector<Pose6D> imu_poses_;
    Mat3d final_rot_ = Mat3d::Identity();
    Vec3d final_pos_ = Vec3d::Zero();
    Vec3d angvel_last_ = Vec3d::Zero();

    IMUPtr last_imu_ = nullptr;
};

// ---------------------------------------------------------------------------
// Per-non-leader-body runtime state (lives in LaserMapping)
// ---------------------------------------------------------------------------

struct NonLeaderBodyState {
    std::string body_id;
    int imu_id = -1;
    std::vector<int> lidar_ids;
    NonLeaderImuProcessor processor;
};

}  // namespace lightning

#endif  // LIGHTNING_MULTIBODY_H
