#pragma once

#ifndef LIGHTNING_DATA_CAPTURE_H
#define LIGHTNING_DATA_CAPTURE_H

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning {

/// Thread-safe filesystem recorder for the active single-body SLAM pipeline.
///
/// Frontend artifacts are addressed by immutable processed-frame IDs. Backend
/// artifacts are addressed by immutable event names, so the asynchronous loop
/// closing thread never depends on mutable frontend state.
class DataCapture {
   public:
    struct Params {
        bool enabled = false;
        std::string output_dir;

        int every_n_frames = 1;
        std::int64_t frame_start = 0;
        std::int64_t frame_end = -1;
        int eskf_iteration_stride = 1;
        int imu_sample_stride = 1;

        bool frontend_enabled = true;
        bool frontend_iterations_enabled = true;
        bool backend_enabled = true;
        bool map_output_enabled = true;
        bool capture_all_keyframes = true;
        bool capture_all_loop_candidates = true;
        bool capture_all_pgo_events = true;
        bool capture_ivox_snapshot = false;

        bool binary_compressed = true;
        std::size_t max_points_per_cloud = 0;
    };

    struct FrameContext {
        std::uint64_t id = 0;
        double lidar_begin_time = 0.0;
        double lidar_end_time = 0.0;
        bool sampled = false;
    };

    DataCapture() = default;
    ~DataCapture();

    DataCapture(const DataCapture&) = delete;
    DataCapture& operator=(const DataCapture&) = delete;

    void configure(const Params& params);

    bool enabled() const { return params_.enabled; }
    const Params& params() const { return params_; }

    FrameContext beginProcessedFrame(double lidar_begin_time, double lidar_end_time);
    bool shouldCaptureFrame(std::uint64_t frame_id) const;
    bool shouldCaptureIteration(int iteration) const;

    void saveFrontendCloud(const FrameContext& frame, const std::string& stage,
                           const PointCloudType& cloud, const std::string& coordinate_frame,
                           bool force = false);
    void saveBackendCloud(const std::string& event, const std::string& stage,
                          const PointCloudType& cloud, const std::string& coordinate_frame);
    void saveMapCloud(const std::string& stage, const PointCloudType& cloud,
                      const std::string& coordinate_frame);

    void appendFrameRow(const FrameContext& frame, const std::string& filename,
                        const std::string& header, const std::string& row, bool force = false);
    void appendBackendRow(const std::string& event, const std::string& filename,
                          const std::string& header, const std::string& row);
    void appendGlobalRow(const std::string& filename, const std::string& header,
                         const std::string& row);
    void writeRunMetadata(const std::string& yaml_text);

    void appendImu(double timestamp, double ax, double ay, double az,
                   double gx, double gy, double gz);

    // Compatibility wrappers used while legacy call sites are migrated.
    void startFrame(int frame_id);
    void savePcd(const std::string& stage, const PointCloudType& cloud);
    void appendEkf(int iter, const Vec3d& pos, const Eigen::Quaterniond& q,
                   const Vec3d& vel, const Vec3d& bg,
                   double yaw, double pitch, double roll,
                   int match_pts, double res_mean, double res_max);

    void flush();

   private:
    std::filesystem::path FrontendDirectory(std::uint64_t frame_id) const;
    std::filesystem::path BackendDirectory(const std::string& event) const;
    static std::string PaddedId(std::uint64_t id);
    static std::string SafeComponent(const std::string& value);

    void SaveCloud(const std::filesystem::path& directory, const std::string& stage,
                   const PointCloudType& cloud, const std::string& coordinate_frame,
                   double lidar_begin_time, double lidar_end_time);
    void AppendRow(const std::filesystem::path& path, const std::string& header,
                   const std::string& row);

    Params params_;
    std::filesystem::path output_dir_;
    std::uint64_t next_processed_frame_id_ = 0;
    std::uint64_t imu_sample_count_ = 0;

    FrameContext legacy_frame_;
    bool legacy_frame_valid_ = false;

    mutable std::mutex mutex_;
};

}  // namespace lightning

#endif  // LIGHTNING_DATA_CAPTURE_H
