#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <tf2_ros/buffer.h>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/options.h"
#include "core/ivox3d/ivox3d.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_processing.hpp"
#include "core/lio/multibody.h"
#include "pointcloud_preprocess.h"

#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

namespace ui {
class PangolinWindow;
}

/**
 * laser mapping
 * 目前有个问题：点云在缓存之后，实际处理的并不是最新的那个点云（通常是buffer里的前一个），这是因为bag里的点云用的开始时间戳，导致
 * 点云的结束时间要比IMU多0.1s左右。为了同步最近的IMU，就只能处理缓冲队列里的那个点云，而不是最新的点云
 */
class LaserMapping {
   public:
    struct Options {
        Options() {}

        bool is_in_slam_mode_ = true;  // 是否在slam模式下

        bool enable_icp_part_ = true;    // 是否添加ICP部分
        double plane_icp_weight_ = 1.0;  // 点面ICP部分的权重
        double icp_weight_ = 100;        // ICP部分的权重

        int min_pts = 300;  // 配准所需的点数

        /// 关键帧阈值
        double kf_dis_th_ = 2.0;
        double kf_angle_th_ = 15 * M_PI / 180.0;

        bool proj_kfs_ = false;
        int max_proj_kfs_ = 5;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping(Options options = Options());
    ~LaserMapping() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "laser mapping deconstruct";
    }

    /// init without ros
    bool Init(const std::string &config_yaml);

    bool Run();

    // callbacks of lidar and imu
    /// 处理ROS2的点云
    void ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    /// 处理livox的点云
    void ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);

    /// 如果已经做了预处理，也可以直接处理点云
    void ProcessPointCloud2(CloudPtr cloud);

    /// 多体模式：处理指定 lidar_id 的点云
    void ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, int lidar_id);

    /// 多体模式：处理指定 imu_id 的 IMU
    void ProcessIMU(const lightning::IMUPtr &msg_in, int imu_id);

    void ProcessIMU(const lightning::IMUPtr &msg_in);

    /// 保存前端的地图
    void SaveMap();

    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) { ui_ = ui; }

    void SetExtrinsic(const Vec3d &translation, const Mat3d &rotation);
    void SetInitialWorldImuRotation(const Mat3d &R_world_imu);

    /// 多体配置（在 Init 之后、Run 之前由 SlamSystem 调用）
    void SetMultiBodyConfig(const MultiBodyConfig &cfg) {
        multibody_cfg_ = cfg;
        const auto* leader_lc = cfg.findLeaderLidar();
        if (leader_lc) leader_lidar_id_ = leader_lc->id;
    }
    void SetTfBuffer(std::shared_ptr<tf2_ros::Buffer> buf) { tf_buffer_ = buf; }
    const Vec3d &GetExtrinsicTranslation() const { return offset_t_lidar_fixed_; }
    const Mat3d &GetExtrinsicRotation() const { return offset_R_lidar_fixed_; }

    /// 获取关键帧
    Keyframe::Ptr GetKeyframe() const { return last_kf_; }

    /// 获取激光的状态
    NavState GetState() const { return state_point_; }

    /// 获取IMU状态
    NavState GetIMUState() const {
        if (p_imu_->IsIMUInited()) {
            return kf_imu_.GetX();
        } else {
            NavState s;
            s.pose_is_ok_ = false;
            return s;
        }
    }

    CloudPtr GetScanUndist() const { return scan_undistort_; }
    CloudPtr GetProjCloud();

    /// 获取最新的点云
    CloudPtr GetRecentCloud();

    std::vector<Keyframe::Ptr> GetAllKeyframes() { return all_keyframes_; }

    /**
     * 计算全局地图
     * @param use_lio_pose
     * @return
     */
    CloudPtr GetGlobalMap(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

    static double ComputeLidarEndTime(double begin_time, const PointCloudType &cloud, double mean_scan_time,
                                      bool input_is_predeskewed);

   private:
    // sync lidar with imu
    bool SyncPackages();
    
    // multi-body sync (multiple lidars + per-body IMUs)
    bool SyncPackagesMultiBody();

    void ObsModel(NavState &s, ESKF::CustomObservationModel &obs);

    inline void PointBodyToWorld(const PointType &pi, PointType &po) {
        Vec3d p_global(state_point_.rot_ *
                           (offset_R_lidar_fixed_ * pi.getVector3fMap().cast<double>() + offset_t_lidar_fixed_) +
                       state_point_.pos_);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();

    /// 多体模式：非 leader body 去畸变 + 跨 body 变换 + 合并
    void ProcessNonLeaderBodies(const NavState& leader_seed, double seed_time);

    bool LoadParamsFromYAML(const std::string &yaml);

    /// 创建关键帧
    void MakeKF();

    /// 将附近的关键帧投影至cloud中
    void ProjectKFs(CloudPtr cloud, int size_limit = 1000);

   private:
    Options options_;

    /// modules
    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    /// local map related
    double filter_size_map_min_ = 0;

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    Mat3d offset_R_lidar_fixed_ = Mat3d::Identity();
    Vec3d offset_t_lidar_fixed_ = Vec3d::Zero();
    std::string map_file_path_;

    std::vector<Keyframe::Ptr> all_keyframes_;
    Keyframe::Ptr last_kf_ = nullptr;
    int kf_id_ = 0;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan

    /// 点面相关
    std::vector<PointVector> nearest_points_;  // nearest points of current scan
    std::vector<Vec4f> corr_pts_;              // inlier pts
    std::vector<Vec4f> corr_norm_;             // inlier plane norms
    std::vector<float> residuals_;             // point-to-plane residuals
    std::vector<char> point_selected_surf_;    // selected points
    std::vector<Vec4f> plane_coef_;            // plane coeffs

    /// 点到点相关
    std::vector<char> point_selected_icp_;  // 点到点的selected points

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;

    /// options
    bool keep_first_imu_estimation_ = false;  // 在没有建立地图前，是否要使用前几帧的IMU状态
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    bool enable_skip_lidar_ = true;  // 雷达是否需要跳帧
    int skip_lidar_num_ = 5;         // 每隔多少帧跳一个雷达
    int skip_lidar_cnt_ = 0;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_surf_ = 0, frame_num_ = 0, effect_feat_icp_ = 0;

    double last_lidar_time_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan

    ESKF kf_;      // 点云时刻的IMU状态
    ESKF kf_imu_;  // imu 最新时刻的eskf状态

    NavState state_point_;  // ekf current state

    bool use_aa_ = false;  // use anderson acceleration?

    std::list<Keyframe::Ptr> proj_kfs_;  // 投影到当前帧的关键帧

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;

    // ---- multi-body members ----
    MultiBodyConfig multibody_cfg_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;

    // Per-lidar buffers (lidar_id → deque)
    std::map<int, std::deque<LidarEntry>> mb_lidar_buffers_;
    // Per-body IMU buffers (imu_id → deque)
    std::map<int, std::deque<lightning::IMUPtr>> mb_imu_buffers_;
    // Per-lidar last timestamp (for loopback detection)
    std::map<int, double> mb_last_lidar_time_;
    // Per-IMU last timestamp
    std::map<int, double> mb_last_imu_time_;

    // Non-leader body state (body_id → state)
    std::vector<NonLeaderBodyState> nonleader_states_;
    // Leader lidar id
    int leader_lidar_id_ = -1;

    // Synced multi-body measurement
    MultiMeasureGroup mb_measures_;
    // Previous scan end time (seed_time for non-leader deskew)
    double prev_lidar_end_time_ = 0;
    // Flag: at least one non-leader body has completed bias init
    bool mb_nonleader_bias_done_ = false;

    // Merged cloud after multi-body deskew + cross-body alignment
    CloudPtr mb_merged_cloud_{new PointCloudType()};
};

}  // namespace lightning

#endif  // FASTER_LIO_LASER_MAPPING_H
