---
title: Lightning 单体 SLAM 数据流与 DataCapture 核对指南
date: 2026-07-31
tags:
  - SLAM
  - Lightning
  - Polka
  - DataCapture
  - 点云
---

# Lightning 单体 SLAM 数据流与 DataCapture 核对指南

## 范围与边界

本文只描述这一条链路：

`Polka 多雷达融合输出 → Lightning 单体 LIO 前端 → 关键帧 → 回环检测/NDT → PGO → 建图输出`

不捕获 Polka 内部各雷达的原始点云、各自去畸变过程和合并中间量。`00_lightning_input_from_polka.pcd` 是 Polka 已经合并完成、刚进入 Lightning 的边界输入。

因此此前讨论的三项应这样归属：

1. ROS/Polka 原始输入点云：Lightning 的入口，数据由 Polka 产生，不是 Polka 内部锚点。
2. Preprocess 过滤后点云：Lightning `PointCloudPreprocess` 内部锚点。
3. IMU 同步后的扫描：Lightning `SyncPackages + ImuProcessing` 内部锚点。

## 全流程

```text
/polka/merged_cloud
  │
  ▼
[00] Lightning ROS 输入
  │ PointCloudPreprocess：stride / range / height
  ▼
[01] 预处理点云
  │ SyncPackages：LiDAR 时间窗 + IMU 覆盖
  ▼
[同步包] scan + imu[]
  │ IMU 初始化 / ESKF Predict / 坐标外参 / 可选逐点去畸变
  ▼
[02] IMU body 点云 + 预测状态
  │ VoxelGrid 降采样 + max_observation_points
  ▼
[03/04] 匹配输入
  │
  ├─ ESKF 迭代
  │    ├─ 点变换到 world
  │    ├─ IVox 最近邻搜索
  │    ├─ 平面拟合
  │    ├─ 残差门限
  │    └─ H、残差、增量、协方差、状态更新
  ▼
[05] 前端最终位姿与世界系点云
  │
  ├─ IVox 增量更新 [06]
  └─ 关键帧判定 [07/08]
         │
         ▼
      回环候选检测
         │
         ▼
      多分辨率 NDT：10 → 5 → 2 → 1 m
         │
         ▼
      回环约束
         │
         ▼
      PGO：运动边 + 回环边 + 高度先验
         │
         ▼
      优化位姿 / 紫线 / 最终地图
```

## 如何运行

先编译：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lightning
```

完整学习模式建议从每 10 帧捕获一次开始，同时强制保留关键帧、回环和 PGO 事件：

```bash
LIGHTNING_CAPTURE_ENABLED=true \
LIGHTNING_CAPTURE_EVERY_N_FRAMES=10 \
LIGHTNING_CAPTURE_ESKF_ITERATION_STRIDE=1 \
LIGHTNING_CAPTURE_IMU_SAMPLE_STRIDE=10 \
LIGHTNING_CAPTURE_DIR=/tmp/lightning_capture \
LIGHTNING_LOOP_CLOSING_ENABLED=true \
bash hw_tests/bag_replay_polka_lightning_slam.sh \
  /home/jarvis/projects/iRail-Truck/bags/slam_debug_20260730_215238_noned \
  base_footprint before
```

配置也可直接写在 `config/articulated_vehicle_map.yaml` 的 `data_capture` 节。

## 频率与磁盘控制

| 配置 | 作用 | 推荐学习值 |
|---|---|---:|
| `enabled` | 总开关 | `true` |
| `every_n_frames` | 每 N 个成功同步并进入处理的帧捕获一次 | `10` |
| `frame_start` / `frame_end` | 仅捕获指定闭区间，`-1` 表示不限制结尾 | `0 / -1` |
| `eskf_iteration_stride` | 每隔 N 次 ESKF 迭代捕获一次 | `1` |
| `imu_sample_stride` | 原始 IMU 每隔 N 个样本写一行 | `10` |
| `capture_all_keyframes` | 即使普通帧未命中采样，也保留关键帧事件 | `true` |
| `capture_all_loop_candidates` | 保留所有回环候选判断和 NDT 数据 | `true` |
| `capture_all_pgo_events` | 保留每次 PGO 前后状态 | `true` |
| `capture_ivox_snapshot` | 每个采样帧保存完整 IVox；非常占空间 | `false` |
| `max_points_per_cloud` | 仅限制写盘副本点数；`0` 为不限制 | `0` |
| `binary_compressed` | PCD 二进制压缩 | `true` |

`every_n_frames` 只影响普通前端帧。关键帧、回环、PGO 使用独立的事件触发，避免因抽样漏掉后端关键节点。

## 输出目录

```text
/tmp/lightning_capture/
├── run.yaml
├── runtime.yaml
├── frames.csv
├── imu.csv
├── imu_initialization.csv
├── input_events.csv
├── sync_events.csv
├── keyframe_decisions.csv
├── keyframes.csv
├── map_outputs.csv
├── frontend/
│   └── frame_000020/
│       ├── 00_lightning_input_from_polka.pcd
│       ├── 01_preprocessed_lidar.pcd
│       ├── 02_imu_body_cloud.pcd
│       ├── 03_voxel_pre_limit.pcd
│       ├── 04_observation_input.pcd
│       ├── 05_state_updated_world.pcd
│       ├── 06_ivox_points_added.pcd
│       ├── 07_keyframe_body.pcd
│       ├── 08_keyframe_world_lio.pcd
│       ├── preprocessing.csv
│       ├── synchronization.csv
│       ├── imu_prediction.csv
│       ├── imu_process_mode.csv
│       ├── downsampling.csv
│       ├── frontend_result.csv
│       ├── ivox_incremental.csv
│       ├── metadata.csv
│       └── iteration_00/
│           ├── scan_world.pcd
│           ├── accepted_source_world.pcd
│           ├── rejected_source_world.pcd
│           ├── accepted_neighbors_world.pcd
│           ├── correspondences.csv
│           └── eskf_iteration.csv
└── backend/
    ├── candidate_detection/candidates.csv
    ├── loop_000080_000120/
    │   ├── source_body.pcd
    │   ├── source_world_initial.pcd
    │   ├── target_submap_world.pcd
    │   ├── ndt_resolution_{10,5,2,1}_*.pcd
    │   ├── submap.csv
    │   ├── ndt.csv
    │   └── result.csv
    ├── pgo_000000/
    │   ├── poses_before.csv
    │   ├── edges.csv
    │   ├── height_priors.csv
    │   ├── loop_edges_after.csv
    │   └── poses_after.csv
    └── map_*/
        ├── assembled_lio.pcd
        ├── assembled_optimized.pcd
        └── selected_global.pcd
```

文件只在对应事件发生时出现。例如非关键帧没有 `07/08`，没有回环时不会产生 `loop_*` 和 `pgo_*`。

## 前端锚点逐项学习

### 00：Lightning 边界输入

`00_lightning_input_from_polka.pcd`

- 坐标系：`base_footprint`。
- 内容：Polka 已融合、已按点补偿到消息头时刻的点云。
- 先核对车体方向、地面方向、量程和明显重影。
- 若这里已经重影，根因在 Polka 或其上游；不要继续归因于 Lightning。

### 01：预处理

`01_preprocessed_lidar.pcd` 与 `preprocessing.csv`

- 依次记录输入点数、stride 淘汰、距离淘汰、高度淘汰和输出点数。
- 对比 00/01 可确认 Lightning 是否误删有效结构。
- `preprocessing.csv` 的各淘汰数之和加输出数应等于输入数。

### 同步与 IMU

`synchronization.csv`、`imu.csv`、`imu_initialization.csv`

- LiDAR 时间窗必须被 IMU 首尾时间覆盖。
- `sync_events.csv` 记录空缓存或 IMU 覆盖不足等等待原因。
- `imu.csv` 是进入 Lightning 的原始 FLU、m/s²、rad/s 数据，不受前端帧采样控制，只受 `imu_sample_stride` 控制。
- 静止初始化时，加速度模长应约为 `9.81 m/s²`，角速度应接近零。
- `runtime.yaml` 记录实际外参及来源，可用来确认运行时采用 TF 还是 YAML fallback。

### 02：IMU 预测与扫描处理

`02_imu_body_cloud.pcd`、`imu_prediction.csv`、`imu_process_mode.csv`

- 当前 Polka 输入标记为 `polka_predeskewed`，Lightning 不应再次逐点去畸变。
- 点云从 LiDAR 帧变换至 `rear_lidar_imu`，用于单体 LIO。
- `imu_prediction.csv` 同时记录 IMU 处理前后状态和协方差对角线。
- 若 01 正常、02 整体姿态错误，优先核对 IMU 数据轴向与 `T(rear_lidar_imu ← base_footprint)`。

### 03/04：降采样与观测输入

`03_voxel_pre_limit.pcd`、`04_observation_input.pcd`、`downsampling.csv`

- 03 是体素降采样结果、尚未执行最大观测点数限制。
- 04 是实际送入扫描匹配的点集。
- `max_observation_points=0` 时 03 和 04 点数应一致。

### ESKF 每次迭代

每次迭代位于 `iteration_XX/`：

- `scan_world.pcd`：用该次迭代状态变换后的全部观测点。
- `accepted_source_world.pcd`：通过邻域、平面和残差门限的点。
- `rejected_source_world.pcd`：被拒绝的点。
- `accepted_neighbors_world.pcd`：有效点从 IVox 取到的邻域点。
- `correspondences.csv`：逐点 body/world 坐标、平面参数、残差和拒绝原因。
- `eskf_iteration.csv`：有效性、是否接受、是否收敛、特征数、残差统计、特征值、秩、`HTH/HTr`、状态增量和更新前后状态。

`correspondences.csv` 的拒绝码：

| 值 | 含义 |
|---:|---|
| `0` | 接受 |
| `1` | IVox 邻居不足 |
| `2` | 平面拟合失败 |
| `3` | 点到平面残差门限失败 |

学习时先看接受点是否落在稳定平面上，再看残差分布和矩阵秩，最后看状态增量是否合理。大量邻居不足通常指向地图、坐标变换或初值问题；大量残差门限失败通常指向预测漂移或场景不匹配。

### 05/06：状态结果与局部地图

- `05_state_updated_world.pcd`：本帧 ESKF 更新后的最终世界系点云。
- `frontend_result.csv`：预测状态、更新状态、迭代次数、最终残差和有效特征数。
- `06_ivox_points_added.pcd`：本帧实际加入 IVox 的点，而不是候选全集。
- `ivox_incremental.csv`：两类加入路径和当前有效栅格数。
- `06_ivox_full_snapshot.pcd`：只有打开 `capture_ivox_snapshot` 才生成。

### 07/08：关键帧

- `keyframe_decisions.csv`：每帧位移、转角、阈值、是否成关键帧及原因。
- `07_keyframe_body.pcd`：关键帧本体点云。
- `08_keyframe_world_lio.pcd`：按当时 LIO 位姿放入世界系的点云。
- `keyframes.csv`：关键帧创建时的 LIO 位姿和初始优化位姿。

## 后端锚点逐项学习

### 回环候选

`backend/candidate_detection/candidates.csv`

每个历史关键帧都记录接受或拒绝原因，包括初始化、与上次回环间隔不足、同轨迹 ID 间隔不足、距离范围内或范围外。先确认候选选择逻辑，再看 NDT；不要只看最终是否回环。

### 多分辨率 NDT

`backend/loop_<历史KF>_<当前KF>/`

- `source_body.pcd`：当前关键帧本体系点云。
- `source_world_initial.pcd`：按 PGO 当前初值放到世界系的源点云。
- `target_submap_world.pcd`：历史关键帧周边构建的世界系子地图。
- 每个分辨率均保存 target voxel、source voxel、aligned world。
- `ndt.csv` 保存收敛状态、迭代数、分数和逐级变换。
- `result.csv` 保存约束是否接受、原因、分数阈值和最终相对位姿。

视觉核对顺序：初始 source/target → 10 m 粗配准 → 5 m → 2 m → 1 m。第一次发生错误跳变的分辨率就是重点。

### PGO

`backend/pgo_NNNNNN/`

- `poses_before.csv`：优化前紫线的输入状态。
- `edges.csv`：运动边与回环边的测量、信息矩阵和鲁棒核。
- `height_priors.csv`：高度约束。
- `loop_edges_after.csv`：优化后回环边 chi²、level 和离群判定。
- `poses_after.csv`：优化后位姿，以及每个关键帧的修正量。

若紫线大幅偏离，先在 `poses_after.csv` 找到修正量开始突增的关键帧，再反查对应回环事件的 NDT 和前端关键帧。

### 地图输出

- `assembled_lio.pcd`：所有关键帧按 LIO 位姿拼接。
- `assembled_optimized.pcd`：按 PGO 优化位姿拼接。
- `selected_global.pcd`：实际选中保存的全局地图。

对比前两者可以直接区分重影主要来自前端累计漂移，还是后端错误约束。

## 推荐人工核对顺序

1. 用 CloudCompare 或 PCL Viewer 打开同一帧的 00、01、02，确认过滤和外参。
2. 检查 `synchronization.csv`、`imu_initialization.csv`、`imu_prediction.csv`。
3. 对比 03/04，确认实际观测输入。
4. 逐个打开 `iteration_XX`，检查有效点、拒绝点、邻域点和残差。
5. 对比 05 与 06，理解状态更新和 IVox 增量。
6. 查看关键帧判定以及 07/08。
7. 开启回环后检查候选表，再逐级检查 NDT。
8. 最后查看 PGO 前后位姿和三份地图。

## 快速质量检查

检查所有 CSV 是否存在空记录或列数变化：

```bash
find /tmp/lightning_capture -name '*.csv' -print0 |
while IFS= read -r -d '' file; do
  awk -F, 'NR==1 { n=NF } NF!=n { print FILENAME \":\" NR \": columns=\" NF \", expected=\" n; exit 1 }' "$file"
done
```

检查一个 PCD 是否可被 PCL 读取：

```bash
pcl_pcd2ply \
  /tmp/lightning_capture/frontend/frame_000020/00_lightning_input_from_polka.pcd \
  /tmp/lightning_capture_check.ply
```

## 使用注意

- DataCapture 只复制数据写盘，不改变算法输入点云。
- `max_points_per_cloud` 只对写盘副本采样，不改变 SLAM。
- PCD 写盘会增加 CPU、磁盘和内存压力；完整 IVox 快照尤其昂贵。
- 每次实验使用新的 `output_dir`，避免多个运行结果追加到相同 CSV。
- `processed_frame_id` 在 `SyncPackages` 成功后分配，所以它代表真正进入处理流程的帧，而不是 ROS 消息序号。
- 所有 PCD 的坐标系和原始/保存点数都记录在同目录 `metadata.csv`，人工比较前先看该文件。
