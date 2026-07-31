# Lightning 单体 SLAM 全流程 DataCapture 设计

## 目标

为当前实际运行的 `POLKA_MERGED` 单体 LIO 链路建立统一、可配置、可人工核对的数据捕获系统。捕获范围从 Polka 合并点云进入 Lightning 开始，覆盖点云预处理、IMU 同步与预测、点云坐标变换、迭代 ESKF 配准、IVox 增量建图、关键帧、回环 NDT、PGO 和最终地图拼接。

本设计不进入 Polka 内部，也不覆盖 Lightning 未启用的 multibody 分支。

## 设计原则

1. 只在数据语义、坐标系或算法状态发生变化的边界设置锚点。
2. 点云节点保存 PCD；状态、矩阵和决策节点保存 CSV；运行参数与外参保存 YAML。
3. 每个成功完成 `SyncPackages()`、进入单体 LIO 的扫描获得稳定 `processed_frame_id`。
4. 普通前端帧按频率采样；关键帧、回环候选和 PGO 事件可独立强制捕获。
5. 前端和异步后端使用不可变 ID 路径，所有文件写入均线程安全。
6. 每份 PCD 必须在清单中标明坐标系和数据含义。
7. DataCapture 不改变 SLAM 算法分支、状态或随机行为。

## 捕获目录

```text
capture/
├── run.yaml
├── frames.csv
├── imu.csv
├── imu_initialization.csv
├── frontend/
│   └── frame_000120/
│       ├── metadata.csv
│       ├── 00_lightning_input_from_polka.pcd
│       ├── 01_preprocessed_lidar.pcd
│       ├── 02_imu_body_cloud.pcd
│       ├── 03_voxel_pre_limit.pcd
│       ├── 04_observation_input.pcd
│       ├── iteration_00/
│       │   ├── scan_world.pcd
│       │   ├── accepted_source_world.pcd
│       │   ├── accepted_neighbors_world.pcd
│       │   ├── rejected_source_world.pcd
│       │   └── observation.csv
│       ├── state_iterations.csv
│       ├── 05_state_updated_world.pcd
│       ├── 06_ivox_points_added.pcd
│       └── 07_keyframe_world_lio.pcd
├── backend/
│   ├── candidates.csv
│   ├── loop_000080_000120/
│   │   ├── source_body.pcd
│   │   ├── source_world_initial.pcd
│   │   ├── target_submap_world.pcd
│   │   ├── ndt_resolution_10.pcd
│   │   ├── ndt_resolution_5.pcd
│   │   ├── ndt_resolution_2.pcd
│   │   ├── ndt_resolution_1.pcd
│   │   └── ndt.csv
│   └── pgo_000003/
│       ├── poses_before.csv
│       ├── edges.csv
│       └── poses_after.csv
└── map_output/
    ├── assembled_lio.pcd
    ├── assembled_optimized.pcd
    └── selected_global.pcd
```

文件仅在对应流程实际发生时存在。例如非关键帧没有 `07_keyframe_world_lio.pcd`，无回环时不会生成 PGO 目录。

## 配置

```yaml
data_capture:
  enabled: false
  output_dir: "/tmp/lightning_capture"

  every_n_frames: 10
  frame_start: 0
  frame_end: -1
  eskf_iteration_stride: 1
  imu_sample_stride: 1

  frontend_enabled: true
  frontend_iterations_enabled: true
  backend_enabled: true
  map_output_enabled: true

  capture_all_keyframes: true
  capture_all_loop_candidates: true
  capture_all_pgo_events: true
  capture_ivox_snapshot: false

  binary_compressed: true
  max_points_per_cloud: 0
```

`every_n_frames=1` 表示捕获每个成功同步帧。`frame_start` 和 `frame_end` 使用 `processed_frame_id`，`frame_end=-1` 表示不限。事件级开关不改变 SLAM 自身是否启用回环，只决定已发生事件是否写盘。

## 前端锚点

### 输入、预处理和同步

- ROS 回调边界保存 Polka 已合并的 XYZ/I 点云，坐标系为配置的 LiDAR frame。
- `PointCloudPreprocess::Process()` 后保存 Lightning 过滤后的标准点云，并记录输入/输出点数。
- `SyncPackages()` 不复制未变化点云；在帧元数据中记录 LiDAR 起止时间、匹配 IMU 起止时间、IMU 数量及缓存长度。
- 重复时间戳、乱序、IMU 覆盖不足等情况写入事件 CSV。

### IMU

- `imu.csv` 按 `imu_sample_stride` 保存 Lightning 实际收到的 FLU/m/s² 与 rad/s 数据。
- 初始化阶段记录均值加速度、均值角速度、重力、陀螺仪偏置和初始化完成标志。
- 每个捕获帧记录 ESKF 预测前后状态和协方差对角线。
- 对 Polka 预去畸变输入明确记录 `pointwise_deskew=false`，同时保存经 `T(imu <- lidar)` 转换后的 IMU/body 点云。

### ESKF 与观测模型

- 降采样前、降采样后以及最大观测点限制后的点云分别设置锚点。
- 每次 `ObsModel()` 调用获得该帧内的迭代号。
- 每轮保存当前状态变换后的 world 点云、接受点、拒绝点、接受点对应的 IVox 邻域点。
- 每轮 CSV 保存有效点数、点面残差统计、`HTH`、`HTr`、信息矩阵特征值、可观测秩、更新增量及更新前后状态。
- 最终保存 ESKF 更新后的 world 点云和状态。

### IVox 与关键帧

- 第一帧记录初始 world 点云和 IVox 初始化事件。
- `MapIncremental()` 保存实际加入 IVox 的点，包括普通新增点和无需降采样点。
- 记录关键帧距离/角度阈值、计算值、判定结果。
- 关键帧保存 body 点云、LIO world 点云、LIO pose 和初始 OptPose。
- `capture_ivox_snapshot` 默认关闭；开启时仅在捕获帧导出全量 IVox 快照，避免默认产生不可控数据量。

## 后端锚点

### 回环候选

- 每次候选检查记录当前关键帧、历史关键帧、ID 间隔、二维距离及接受/拒绝原因。
- 强制捕获的回环事件以关键帧 ID 命名，不依赖前端当前帧目录。

### 多分辨率 NDT

- 保存 source body 点云、按当前 OptPose 投影的初始 source world 点云和历史 target world 子地图。
- 10、5、2、1 米每一级分别保存对齐输出。
- 每级记录输入点数、voxel 后点数、初始/最终变换、迭代数、收敛状态和 transformation probability。
- 最终记录阈值和约束接受/拒绝结果。

### PGO

- 优化前保存所有顶点位姿。
- 保存运动边、回环边和高度先验，包含测量、信息矩阵、鲁棒核、χ² 和 level。
- 优化后保存所有顶点位姿与相对优化前的修正量。
- 对被判为异常的回环边记录 outlier 状态。

## 地图输出

保存地图时分别构造并捕获：

1. 所有关键帧按 LIOPose 拼接并 voxel 后的地图。
2. 所有关键帧按 OptPose 拼接并 voxel 后的地图。
3. 当前运行配置最终选择并写入 `global.pcd` 的地图。

轨迹 CSV 同时提供 LIOPose 和 OptPose，供红、绿、紫轨迹来源对照。

## 运行安全与性能

- PCD 默认使用 binary compressed。
- `max_points_per_cloud=0` 表示不限制；非零时仅影响写盘副本，不修改算法点云。
- 捕获失败只记录错误并继续 SLAM，不允许写盘异常中断建图。
- 前端普通帧的采样决策在获得稳定帧号后一次确定；该帧所有前端节点共享同一决策。
- 后端目录和 CSV 由互斥锁保护，避免在线异步回环线程与前端竞争。
- 输出目录内含 schema/version，便于后续工具兼容。

## 验收标准

1. 禁用 DataCapture 时不创建文件，现有 SLAM 路径不变。
2. `every_n_frames=2` 时只为帧 0、2、4 等建立普通前端完整目录。
3. 被捕获帧可按清单逐节点确认点数、坐标系和时间范围。
4. 每次 ObsModel 调用都有连续迭代号，迭代点云和状态行可互相对应。
5. 回环发生时可逐级比较 NDT source/target/output。
6. PGO 前后位姿和异常边可以从 CSV 独立复现修正方向。
7. 地图输出目录同时存在 LIO、优化后和最终选择三份地图。
8. Lightning 包编译通过，已有测试和新增 DataCapture 测试通过。
