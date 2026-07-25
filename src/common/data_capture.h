#pragma once

#ifndef LIGHTNING_DATA_CAPTURE_H
#define LIGHTNING_DATA_CAPTURE_H

#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include "common/point_def.h"

namespace lightning {

/// Lightweight pipeline data capture for offline debugging.
/// Inspired by colleague's fast_lio_multi_ros2 DataCapture.
///
/// Output structure:
///   {output_dir}/
///   ├── ekf.csv
///   ├── imu.csv
///   └── frame_NNNNNN/
///       ├── 01_raw.pcd            — raw scan (after preprocess)
///       ├── 02_deskewed.pcd       — after IMU deskew
///       ├── 03_merged.pcd         — after non-leader merge (multibody only)
///       ├── 04_obs_input.pcd      — after voxel downsample
///       └── 05_kf_cloud.pcd       — keyframe cloud (on keyframe creation)
class DataCapture {
   public:
    struct Params {
        bool enabled = false;
        std::string output_dir;
        bool raw = true;
        bool deskewed = true;
        bool merged = true;
        bool obs_input = true;
        bool kf_cloud = true;
        bool csv_ekf = true;
        bool csv_imu = true;
    };

    DataCapture() = default;

    void configure(const Params& p) {
        params_ = p;
        if (params_.enabled && !params_.output_dir.empty()) {
            std::filesystem::create_directories(params_.output_dir);
            if (params_.csv_ekf) {
                ekf_csv_.open(params_.output_dir + "/ekf.csv");
                ekf_csv_ << "frame,iter,pos_x,pos_y,pos_z,qw,qx,qy,qz,"
                         << "vel_x,vel_y,vel_z,bg_x,bg_y,bg_z,"
                         << "yaw_deg,pitch_deg,roll_deg,"
                         << "match_pts,residual_mean,residual_max\n";
            }
            if (params_.csv_imu) {
                imu_csv_.open(params_.output_dir + "/imu.csv");
                imu_csv_ << "timestamp,ax,ay,az,gx,gy,gz\n";
            }
        }
    }

    bool enabled() const { return params_.enabled; }
    const Params& params() const { return params_; }

    void startFrame(int frame_id) {
        frame_id_ = frame_id;
        if (params_.enabled) {
            frame_dir_ = params_.output_dir + "/frame_" + std::to_string(frame_id);
            std::filesystem::create_directories(frame_dir_);
        }
    }

    void savePcd(const std::string& stage, const PointCloudType& cloud) {
        if (!params_.enabled || frame_dir_.empty()) return;
        pcl::io::savePCDFileBinary(frame_dir_ + "/" + stage + ".pcd", cloud);
    }

    void appendEkf(int iter, const Vec3d& pos, const Eigen::Quaterniond& q,
                   const Vec3d& vel, const Vec3d& bg,
                   double yaw, double pitch, double roll,
                   int match_pts, double res_mean, double res_max) {
        if (!params_.csv_ekf || !ekf_csv_.is_open()) return;
        std::lock_guard<std::mutex> lock(mtx_);
        ekf_csv_ << frame_id_ << "," << iter << ","
                 << pos.x() << "," << pos.y() << "," << pos.z() << ","
                 << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
                 << vel.x() << "," << vel.y() << "," << vel.z() << ","
                 << bg.x() << "," << bg.y() << "," << bg.z() << ","
                 << yaw << "," << pitch << "," << roll << ","
                 << match_pts << "," << res_mean << "," << res_max << "\n";
    }

    void appendImu(double timestamp, double ax, double ay, double az,
                   double gx, double gy, double gz) {
        if (!params_.csv_imu || !imu_csv_.is_open()) return;
        std::lock_guard<std::mutex> lock(mtx_);
        imu_csv_ << std::setprecision(15) << timestamp << ","
                 << ax << "," << ay << "," << az << ","
                 << gx << "," << gy << "," << gz << "\n";
    }

    void flush() {
        if (ekf_csv_.is_open()) ekf_csv_.flush();
        if (imu_csv_.is_open()) imu_csv_.flush();
    }

   private:
    Params params_;
    int frame_id_ = 0;
    std::string frame_dir_;
    std::ofstream ekf_csv_;
    std::ofstream imu_csv_;
    std::mutex mtx_;
};

}  // namespace lightning

#endif  // LIGHTNING_DATA_CAPTURE_H
