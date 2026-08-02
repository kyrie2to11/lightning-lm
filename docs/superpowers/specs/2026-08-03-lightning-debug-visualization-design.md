# Lightning 调试可视化与离线教学回放设计

日期：2026-08-03

## 1. 目标

在不改变 Lightning SLAM 算法输入和生产运行行为的前提下，建立一套可重复学习、可逐节点核对的数据可视化体系：

- DataCapture 持久化完整前后端锚点；
- Lichtblick 在线观察 Lightning 内部阶段点云、匹配过程与轨迹；
- PlotJuggler 在线观察 IMU、ESKF、IVox、NDT 和 PGO 时序指标；
- bag 处理结束后，可从 DataCapture 离线逐帧回放全部阶段；
- 自动生成静态 PNG、交互式 Plotly HTML 和 Obsidian 图文学习报告。

本设计只覆盖 Polka 完成多雷达融合后进入 Lightning 的单体 LIO，以及 Lightning 后端关键帧、回环、NDT、PGO 和地图输出。Polka 内部各雷达的去畸变与融合中间量不在本期范围内。

## 2. 设计原则

1. **生产默认关闭**：四项调试能力默认均为 `false`。
2. **关闭时零重活**：不开启时不创建调试 publisher、不复制点云、不构造大型消息、不写文件、不启动离线回放器。
3. **算法无副作用**：采样、可视化和落盘只读取算法已有结果，不改变点云、状态、匹配或优化输入。
4. **独立开关**：磁盘捕获、在线点云、在线时序、离线回放互不隐式启用。
5. **事件不漏采**：普通前端按帧采样；关键帧、回环和 PGO 可独立强制保留。
6. **坐标系明确**：每个点云 topic 和文件都携带正确 frame，禁止靠观察者猜测坐标系。

## 3. 四项独立开关

现有 `data_capture.enabled` 保持不变，新增可视化和回放配置：

```yaml
data_capture:
  enabled: false

debug_visualization:
  live_cloud_enabled: false
  live_cloud_every_n_frames: 10
  live_timeseries_enabled: false
  live_timeseries_every_n_frames: 1

stage_replay:
  enabled: false
```

开关语义：

| 开关 | 所属进程 | 行为 |
|---|---|---|
| `data_capture.enabled` | Lightning | 写 PCD、CSV 和运行元数据 |
| `live_cloud_enabled` | Lightning | 发布供 Lichtblick 使用的内部阶段点云、轨迹和 Marker |
| `live_timeseries_enabled` | Lightning | 发布供 PlotJuggler 使用的数值指标 |
| `stage_replay.enabled` | 独立回放器 | 从指定 DataCapture 目录逐帧发布离线数据 |

生产配置中四项全部关闭。离线回放器不是 Lightning 子线程，只有用户显式启动辅助脚本时才运行；其配置默认值仍为关闭，防止教学脚本被误启动。

## 4. 架构选择

采用“轻量调试发布器 + 现有 DataCapture + 独立离线回放器”。

```text
Lightning 算法节点
  │
  ├── DataCapture（enabled 时落盘）
  │
  ├── DebugCloudPublisher（live_cloud_enabled 时发布）
  │       └── foxglove_bridge ──> Lichtblick
  │
  └── DebugMetricsPublisher（live_timeseries_enabled 时发布）
          └── PlotJuggler ROS2 Subscriber

DataCapture 目录
  ├── CaptureAnalyzer ──> PNG + Plotly HTML + 汇总 CSV + Markdown
  └── StageReplayNode ──> ROS topics ──> Lichtblick / PlotJuggler
```

不采用“旁路实时扫描正在写入的文件”，因为它存在半写入竞争、延迟和事件丢失风险。也不在每个算法函数中直接创建 publisher；所有 ROS 发布和采样判断集中在调试发布器，算法节点只在开关开启后传递现成结果。

## 5. 在线 Lichtblick 数据

主题统一使用 `/lightning/debug` 前缀：

### 5.1 前端点云

- `/lightning/debug/frontend/input`
- `/lightning/debug/frontend/preprocessed`
- `/lightning/debug/frontend/imu_body`
- `/lightning/debug/frontend/voxel_pre_limit`
- `/lightning/debug/frontend/observation_input`
- `/lightning/debug/frontend/state_updated`
- `/lightning/debug/frontend/ivox_added`
- `/lightning/debug/frontend/keyframe_body`
- `/lightning/debug/frontend/keyframe_world`

### 5.2 ESKF 迭代

- `/lightning/debug/iteration/scan_world`
- `/lightning/debug/iteration/accepted`
- `/lightning/debug/iteration/rejected`
- `/lightning/debug/iteration/neighbors`
- `/lightning/debug/iteration/correspondences`

`correspondences` 使用 `visualization_msgs/MarkerArray` 或等价紧凑 Marker 表示匹配线。点数必须受独立上限控制，避免一次迭代发布数万条线。

### 5.3 后端与轨迹

- `/lightning/debug/backend/ndt/source`
- `/lightning/debug/backend/ndt/target`
- `/lightning/debug/backend/ndt/aligned`
- `/lightning/debug/path/lio`
- `/lightning/debug/path/optimized`
- `/lightning/debug/backend/loop_constraints`

点云使用 `sensor_msgs/msg/PointCloud2`，轨迹使用 `nav_msgs/msg/Path`，约束关系使用 `visualization_msgs/msg/MarkerArray`。在线主题采用适合可视化的 QoS，并在首次发布时记录 frame 与 stage 元数据。

## 6. 在线 PlotJuggler 数据

使用已安装的 `plotjuggler_msgs/msg/Dictionary` 和 `plotjuggler_msgs/msg/DataPoints`，避免为调试指标新增大量自定义 ROS 消息。字段名稳定、分组明确：

### 6.1 IMU 与预测

- 原始加速度和角速度；
- 初始化均值、方差、样本数和完成状态；
- 预测前后位置、姿态、速度、陀螺 bias、加速度 bias、gravity；
- 协方差对角线。

### 6.2 ESKF 匹配

- 迭代编号、有效性、是否接受、是否收敛；
- 有效特征数、接受/拒绝点数；
- 平均/最大残差；
- 观测矩阵秩和特征值；
- 位姿、速度与 bias 增量。

### 6.3 地图与后端

- IVox 新增点数和有效栅格数；
- 关键帧位移、转角、判定结果；
- NDT 分辨率、score、迭代数、收敛状态；
- PGO 修正平移、修正转角、回环 chi² 和离群标记。

字典只在字段集合变化时发布，数值消息使用 bag/算法时间戳而不是系统墙钟时间，使在线和离线曲线能够对齐。

## 7. 离线阶段回放器

`StageReplayNode` 读取一个完成的 DataCapture 目录，并复用在线主题命名。功能包括：

- 按 `processed_frame_id` 顺序播放；
- 指定开始帧、结束帧、播放倍率和循环；
- 暂停、继续、单步前进、单步后退和跳转到指定帧；
- 在同一帧内按阶段切换，或同时发布多个阶段用于叠加比较；
- 发布 `/clock`，保证 Lichtblick、TF、Path 和 PlotJuggler 使用一致时间轴；
- 回放前验证 PCD/CSV 完整性，遇到缺失事件时发布状态而不是伪造数据；
- 关键帧、回环、PGO 使用独立事件索引，不假设每个前端帧都有后端数据。

离线回放器的 `enabled=false` 是显式防误启动门；命令行必须同时提供捕获目录和启用参数才进入发布循环。

## 8. 静态与交互式分析

`CaptureAnalyzer` 读取同一 DataCapture 目录，输出到独立报告目录：

```text
report/
├── index.html
├── summary.json
├── timeseries.csv
├── images/
│   ├── frontend_stage_overview.png
│   ├── imu_and_prediction.png
│   ├── eskf_iterations.png
│   ├── ivox_and_keyframes.png
│   ├── ndt_pyramid.png
│   └── lio_vs_pgo.png
├── interactive/
│   ├── frontend_stages.html
│   ├── eskf_correspondences.html
│   ├── ndt_alignment.html
│   └── trajectories.html
├── layouts/
│   ├── lichtblick_lightning_learning.json
│   └── plotjuggler_lightning_learning.xml
└── Lightning_SLAM_数据流学习报告.md
```

PNG 面向 Obsidian 顺序阅读，Plotly HTML 面向点云旋转、缩放、开关图层。点云显示副本允许确定性降采样，但报告必须标明原始点数与显示点数。

## 9. 第一次完整回放参数

使用 bag：

`/home/jarvis/projects/iRail-Truck/bags/slam_debug_20260730_215238_noned`

参数：

- `data_capture.enabled=true`
- `every_n_frames=10`
- `eskf_iteration_stride=1`
- `imu_sample_stride=10`
- `capture_all_keyframes=true`
- `capture_all_loop_candidates=true`
- `capture_all_pgo_events=true`
- `capture_ivox_snapshot=false`
- `with_loop_closing=true`
- 在线点云和时序发布在教学运行中开启；生产默认保持关闭。

回放输出使用新的时间戳目录，不覆盖已有烟测结果。回放完成后恢复仓库 YAML 默认值，脚本不得把临时启用值留在工作树。

## 10. 教学分析顺序

报告按以下顺序带读：

1. Polka 与 Lightning 的责任边界；
2. 入口点云和预处理过滤；
3. LiDAR/IMU 时间同步；
4. IMU 初始化、预测与坐标变换；
5. 体素降采样和最终观测输入；
6. IVox 邻域、平面拟合、残差门限；
7. ESKF 迭代状态更新；
8. IVox 地图增量和关键帧生成；
9. 回环候选与多分辨率 NDT；
10. PGO 前后轨迹修正；
11. LIO 地图、优化地图和最终地图对比；
12. 从异常现象反查对应锚点的方法。

每一节至少回答：输入是什么、处理做了什么、输出是什么、使用哪个坐标系、正常结果应长什么样、异常时应回查哪里。

## 11. 错误处理

- 配置频率小于 1 时拒绝启动调试发布器并打印明确错误；
- 输出目录已存在时使用新运行目录，禁止静默混写；
- PCD 或 CSV 缺失时报告具体路径和事件，不导致其他可用阶段无法分析；
- ROS 发布失败不影响 SLAM 主循环；错误按频率限制记录；
- 点云或 Marker 超过显示上限时只限制调试副本，并记录截断前后数量；
- 关闭任何开关后，其对应输出目录、publisher 和后台线程均不得出现。

## 12. 验收标准

1. 四项开关默认均关闭，普通 bag replay 不产生调试目录和 `/lightning/debug/*` topic。
2. 每项开关可以单独开启，不隐式开启其他三项。
3. 在线主题在 Lichtblick 中能叠加展示前端阶段、迭代点集、NDT 和两条轨迹。
4. PlotJuggler 能加载预置布局并显示 IMU、ESKF、IVox、关键帧、NDT、PGO 曲线。
5. 离线回放器支持暂停、单步、跳转和重放，发布结果与捕获目录一致。
6. 完整 bag 回放成功产生前端、关键帧、回环、NDT、PGO 和地图数据。
7. 所有 CSV 列数一致，选取的 PCD 可被 PCL/Open3D 读取。
8. 静态 PNG、交互式 HTML、布局文件和 Obsidian 报告均可打开。
9. 开启调试不会改变相同输入下的 SLAM 算法点数、关键帧判定和最终位姿结果；允许因线程调度产生运行耗时差异。

## 13. 非目标

- 不修改 Polka 内部算法或捕获其内部各雷达阶段；
- 不把调试 topic 作为稳定生产 API；
- 不在生产启动脚本中自动打开 Lichtblick 或 PlotJuggler；
- 不为了可视化修改 ESKF、NDT 或 PGO 数学逻辑；
- 不默认保存完整 IVox 每帧快照。
