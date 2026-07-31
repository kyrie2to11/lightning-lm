#include "common/data_capture.h"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>

#include <glog/logging.h>

namespace lightning {

DataCapture::~DataCapture() { flush(); }

void DataCapture::configure(const Params& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    params_.every_n_frames = std::max(1, params_.every_n_frames);
    params_.eskf_iteration_stride = std::max(1, params_.eskf_iteration_stride);
    params_.imu_sample_stride = std::max(1, params_.imu_sample_stride);
    next_processed_frame_id_ = 0;
    imu_sample_count_ = 0;
    legacy_frame_valid_ = false;

    if (!params_.enabled || params_.output_dir.empty()) {
        params_.enabled = false;
        output_dir_.clear();
        return;
    }

    output_dir_ = params_.output_dir;
    try {
        std::filesystem::create_directories(output_dir_ / "frontend");
        std::filesystem::create_directories(output_dir_ / "backend");
        std::filesystem::create_directories(output_dir_ / "map_output");

        std::ofstream run(output_dir_ / "run.yaml", std::ios::trunc);
        run << "schema_version: 1\n"
            << "every_n_frames: " << params_.every_n_frames << "\n"
            << "frame_start: " << params_.frame_start << "\n"
            << "frame_end: " << params_.frame_end << "\n"
            << "eskf_iteration_stride: " << params_.eskf_iteration_stride << "\n"
            << "imu_sample_stride: " << params_.imu_sample_stride << "\n"
            << "binary_compressed: " << (params_.binary_compressed ? "true" : "false") << "\n"
            << "max_points_per_cloud: " << params_.max_points_per_cloud << "\n";
    } catch (const std::exception& e) {
        LOG(ERROR) << "DataCapture configure failed: " << e.what();
        params_.enabled = false;
        output_dir_.clear();
    }
}

DataCapture::FrameContext DataCapture::beginProcessedFrame(double lidar_begin_time, double lidar_end_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    FrameContext frame;
    frame.id = next_processed_frame_id_++;
    frame.lidar_begin_time = lidar_begin_time;
    frame.lidar_end_time = lidar_end_time;
    frame.sampled = params_.enabled && params_.frontend_enabled && shouldCaptureFrame(frame.id);

    if (params_.enabled) {
        std::ostringstream row;
        row << frame.id << "," << std::setprecision(15) << lidar_begin_time << ","
            << lidar_end_time << "," << (frame.sampled ? 1 : 0);
        AppendRow(output_dir_ / "frames.csv",
                  "processed_frame_id,lidar_begin_time,lidar_end_time,sampled", row.str());
    }
    return frame;
}

bool DataCapture::shouldCaptureFrame(std::uint64_t frame_id) const {
    if (!params_.enabled || !params_.frontend_enabled) {
        return false;
    }
    if (static_cast<std::int64_t>(frame_id) < params_.frame_start) {
        return false;
    }
    if (params_.frame_end >= 0 && static_cast<std::int64_t>(frame_id) > params_.frame_end) {
        return false;
    }
    return frame_id % static_cast<std::uint64_t>(params_.every_n_frames) == 0;
}

bool DataCapture::shouldCaptureIteration(int iteration) const {
    return params_.enabled && params_.frontend_iterations_enabled && iteration >= 0 &&
           iteration % params_.eskf_iteration_stride == 0;
}

void DataCapture::saveFrontendCloud(const FrameContext& frame, const std::string& stage,
                                    const PointCloudType& cloud, const std::string& coordinate_frame,
                                    bool force) {
    if (!params_.enabled || !params_.frontend_enabled || (!frame.sampled && !force)) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SaveCloud(FrontendDirectory(frame.id), stage, cloud, coordinate_frame,
              frame.lidar_begin_time, frame.lidar_end_time);
}

void DataCapture::saveFrontendIterationCloud(const FrameContext& frame, int iteration,
                                             const std::string& stage, const PointCloudType& cloud,
                                             const std::string& coordinate_frame) {
    if (!params_.enabled || !params_.frontend_enabled || !frame.sampled ||
        !shouldCaptureIteration(iteration)) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SaveCloud(
        FrontendDirectory(frame.id) / ("iteration_" + PaddedId(iteration).substr(4)),
        stage, cloud, coordinate_frame, frame.lidar_begin_time, frame.lidar_end_time);
}

void DataCapture::saveBackendCloud(const std::string& event, const std::string& stage,
                                   const PointCloudType& cloud, const std::string& coordinate_frame) {
    if (!params_.enabled || !params_.backend_enabled) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SaveCloud(BackendDirectory(event), stage, cloud, coordinate_frame, 0.0, 0.0);
}

void DataCapture::saveMapCloud(const std::string& stage, const PointCloudType& cloud,
                               const std::string& coordinate_frame) {
    if (!params_.enabled || !params_.map_output_enabled) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    SaveCloud(output_dir_ / "map_output", stage, cloud, coordinate_frame, 0.0, 0.0);
}

void DataCapture::appendFrameRow(const FrameContext& frame, const std::string& filename,
                                 const std::string& header, const std::string& row, bool force) {
    if (!params_.enabled || !params_.frontend_enabled || (!frame.sampled && !force)) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    AppendRow(FrontendDirectory(frame.id) / SafeComponent(filename), header, row);
}

void DataCapture::appendFrontendIterationRow(const FrameContext& frame, int iteration,
                                             const std::string& filename, const std::string& header,
                                             const std::string& row) {
    if (!params_.enabled || !params_.frontend_enabled || !frame.sampled ||
        !shouldCaptureIteration(iteration)) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto directory =
        FrontendDirectory(frame.id) / ("iteration_" + PaddedId(iteration).substr(4));
    AppendRow(directory / SafeComponent(filename), header, row);
}

void DataCapture::appendBackendRow(const std::string& event, const std::string& filename,
                                   const std::string& header, const std::string& row) {
    if (!params_.enabled || !params_.backend_enabled) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    AppendRow(BackendDirectory(event) / SafeComponent(filename), header, row);
}

void DataCapture::appendGlobalRow(const std::string& filename, const std::string& header,
                                  const std::string& row) {
    if (!params_.enabled) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    AppendRow(output_dir_ / SafeComponent(filename), header, row);
}

void DataCapture::writeRunMetadata(const std::string& yaml_text) {
    if (!params_.enabled) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    try {
        std::ofstream out(output_dir_ / "runtime.yaml", std::ios::trunc);
        out << yaml_text;
    } catch (const std::exception& e) {
        LOG(ERROR) << "DataCapture runtime metadata failed: " << e.what();
    }
}

void DataCapture::appendImu(double timestamp, double ax, double ay, double az,
                            double gx, double gy, double gz) {
    if (!params_.enabled) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint64_t sample_id = imu_sample_count_++;
    if (sample_id % static_cast<std::uint64_t>(params_.imu_sample_stride) != 0) {
        return;
    }
    std::ostringstream row;
    row << sample_id << "," << std::setprecision(15) << timestamp << ","
        << ax << "," << ay << "," << az << "," << gx << "," << gy << "," << gz;
    AppendRow(output_dir_ / "imu.csv", "sample_id,timestamp,ax,ay,az,gx,gy,gz", row.str());
}

void DataCapture::startFrame(int frame_id) {
    legacy_frame_.id = static_cast<std::uint64_t>(std::max(0, frame_id));
    legacy_frame_.sampled = shouldCaptureFrame(legacy_frame_.id);
    legacy_frame_valid_ = true;
}

void DataCapture::savePcd(const std::string& stage, const PointCloudType& cloud) {
    if (!legacy_frame_valid_) {
        return;
    }
    saveFrontendCloud(legacy_frame_, stage, cloud, "unknown");
}

void DataCapture::appendEkf(int iter, const Vec3d& pos, const Eigen::Quaterniond& q,
                            const Vec3d& vel, const Vec3d& bg,
                            double yaw, double pitch, double roll,
                            int match_pts, double res_mean, double res_max) {
    if (!params_.enabled || !legacy_frame_valid_) {
        return;
    }
    std::ostringstream row;
    row << legacy_frame_.id << "," << iter << ","
        << pos.x() << "," << pos.y() << "," << pos.z() << ","
        << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
        << vel.x() << "," << vel.y() << "," << vel.z() << ","
        << bg.x() << "," << bg.y() << "," << bg.z() << ","
        << yaw << "," << pitch << "," << roll << ","
        << match_pts << "," << res_mean << "," << res_max;
    appendGlobalRow(
        "ekf.csv",
        "frame,iter,pos_x,pos_y,pos_z,qw,qx,qy,qz,vel_x,vel_y,vel_z,bg_x,bg_y,bg_z,"
        "yaw_deg,pitch_deg,roll_deg,match_pts,residual_mean,residual_max",
        row.str());
}

void DataCapture::flush() {
    // Files are opened and closed per append to make asynchronous backend rows
    // durable and avoid owning streams across reconfiguration.
}

std::filesystem::path DataCapture::FrontendDirectory(std::uint64_t frame_id) const {
    return output_dir_ / "frontend" / ("frame_" + PaddedId(frame_id));
}

std::filesystem::path DataCapture::BackendDirectory(const std::string& event) const {
    return output_dir_ / "backend" / SafeComponent(event);
}

std::string DataCapture::PaddedId(std::uint64_t id) {
    std::ostringstream stream;
    stream << std::setw(6) << std::setfill('0') << id;
    return stream.str();
}

std::string DataCapture::SafeComponent(const std::string& value) {
    std::string safe;
    safe.reserve(value.size());
    for (unsigned char c : value) {
        if (std::isalnum(c) || c == '_' || c == '-' || c == '.') {
            safe.push_back(static_cast<char>(c));
        } else {
            safe.push_back('_');
        }
    }
    return safe.empty() ? "unnamed" : safe;
}

void DataCapture::SaveCloud(const std::filesystem::path& directory, const std::string& stage,
                            const PointCloudType& cloud, const std::string& coordinate_frame,
                            double lidar_begin_time, double lidar_end_time) {
    try {
        std::filesystem::create_directories(directory);
        const std::string safe_stage = SafeComponent(stage);
        const auto path = directory / (safe_stage + ".pcd");

        const PointCloudType* cloud_to_write = &cloud;
        PointCloudType limited;
        if (params_.max_points_per_cloud > 0 && cloud.size() > params_.max_points_per_cloud) {
            limited.points.assign(cloud.points.begin(),
                                  cloud.points.begin() + params_.max_points_per_cloud);
            limited.width = static_cast<std::uint32_t>(limited.size());
            limited.height = 1;
            limited.is_dense = cloud.is_dense;
            cloud_to_write = &limited;
        }

        const int result = params_.binary_compressed
                               ? pcl::io::savePCDFileBinaryCompressed(path.string(), *cloud_to_write)
                               : pcl::io::savePCDFileBinary(path.string(), *cloud_to_write);
        if (result != 0) {
            LOG(ERROR) << "DataCapture failed to save " << path;
            return;
        }

        std::ostringstream row;
        row << safe_stage << "," << path.filename().string() << "," << SafeComponent(coordinate_frame)
            << "," << cloud.size() << "," << cloud_to_write->size() << ","
            << std::setprecision(15) << lidar_begin_time << "," << lidar_end_time;
        AppendRow(directory / "metadata.csv",
                  "stage,file,coordinate_frame,algorithm_points,saved_points,lidar_begin_time,lidar_end_time",
                  row.str());
    } catch (const std::exception& e) {
        LOG(ERROR) << "DataCapture cloud write failed: " << e.what();
    }
}

void DataCapture::AppendRow(const std::filesystem::path& path, const std::string& header,
                            const std::string& row) {
    try {
        std::filesystem::create_directories(path.parent_path());
        const bool write_header = !std::filesystem::exists(path) || std::filesystem::file_size(path) == 0;
        std::ofstream out(path, std::ios::app);
        if (!out.is_open()) {
            LOG(ERROR) << "DataCapture failed to open " << path;
            return;
        }
        if (write_header && !header.empty()) {
            out << header << "\n";
        }
        out << row;
        if (row.empty() || row.back() != '\n') {
            out << "\n";
        }
    } catch (const std::exception& e) {
        LOG(ERROR) << "DataCapture row write failed: " << e.what();
    }
}

}  // namespace lightning
