# Lightning 关键帧航向对照调试设计

创建日期：2026-08-03  
标签：#SLAM #Lightning #PlotJuggler #DataCapture #Debug

## 1. 目标

为 Polka 合并点云后的 Lightning 单体 SLAM 增加一条默认关闭的旁路诊断链，在不让底盘里程计参与 SLAM 估计的前提下，逐关键帧比较：

1. IMU 传播得到的 Lightning 预测姿态；
2. 点云观测更新后的 Lightning 姿态；
3. 底盘 `/articulated_steering_controller/odom` 航向；
4. 回环 NDT 与 PGO 对关键帧姿态的后端修正。

回放期间自动扫描全部关键帧。当 Lightning 车体相对航向与底盘 odom 相对航向首次超过配置阈值时，诊断节点暂停 rosbag player，并保留异常关键帧及其前后关键帧的可复核证据。

本功能只用于学习和故障定位。生产默认不创建诊断节点、不回放参考 odom、不发布额外姿态指标，也不写诊断 CSV。

## 2. 约束与非目标

- 底盘 odom 只能作为独立参考，不能作为 Lightning 的状态观测或 PGO 约束。
- `data_capture.enabled`、在线点云、在线时序和关键帧自动诊断保持相互独立。
- Lightning 构建并行 worker 上限固定为 4，避免 OOM。
- 不在生产启动脚本中自动打开 PlotJuggler 或 Lichtblick。
- 本轮不修正外参、IMU轴向、ESKF或PGO算法；只增加能够定位首个错误节点的证据。
- 不把轮速 odom 描述为真值。结论必须保留轮滑、绞接模型误差和时间同步误差的限制。

## 3. 方案选择

### 3.1 采用：旁路关键帧诊断节点

Lightning 只发布自身已有状态推导出的姿态指标。独立 Python 诊断节点订阅 Lightning 指标和底盘 odom，在关键帧时刻做时间匹配、角度解包、归零比较、阈值判断和暂停控制。

优点：

- 不把参考 odom 耦合进 SLAM主进程；
- 诊断节点关闭时不改变 Lightning 的订阅、调度和估计；
- 可以单独测试时间匹配与角度逻辑；
- 实时 PlotJuggler 与逐关键帧 CSV 使用同一组比较结果。

### 3.2 不采用：Lightning 直接订阅底盘 odom

虽然可以在一个进程内完成时间对齐，但会把仅用于验证的参考传感器引入算法进程，增加调度和错误耦合，不利于证明诊断对 SLAM 无影响。

### 3.3 不采用：仅在回放结束后离线计算

离线处理风险最低，但不能在首个异常关键帧暂停，也无法在现场同步观察点云、ESKF和参考航向，因此只适合作为结果复核，不作为主流程。

## 4. 数据流

```text
bag /imu + raw clouds
        │
        ▼
Polka ──> Lightning LIO ──> compact PlotJuggler metrics
                              │
                              ├── predicted IMU/base attitude
                              ├── updated IMU/base attitude
                              ├── ESKF metrics
                              └── keyframe event

bag /articulated_steering_controller/odom
        │
        ▼
KeyframeYawMonitor <──────── Lightning metrics
        │
        ├── nearest-time odom match
        ├── unwrap + normalize at first valid keyframe
        ├── yaw error threshold
        ├── stamped comparison topic ──> PlotJuggler
        ├── one-row-per-keyframe CSV
        └── first threshold crossing ──> rosbag player pause service
```

## 5. Lightning 姿态指标

### 5.1 坐标约定

Lightning `NavState::rot_` 表示 `R_world_imu`。配置与运行时 TF 提供：

```text
p_imu = R_imu_base · p_base + t_imu_base
T_imu_base = T(rear_lidar_imu ← base_footprint)
```

因此车体姿态必须通过以下合成得到：

```text
R_world_base = R_world_imu · R_imu_base
```

不得直接把 `R_world_imu` 的 yaw 当作车辆 yaw。

### 5.2 在线字段

预测状态和点云更新后状态各发布：

```text
state/predicted/imu_roll_deg
state/predicted/imu_pitch_deg
state/predicted/imu_yaw_deg
state/predicted/base_roll_deg
state/predicted/base_pitch_deg
state/predicted/base_yaw_deg

state/updated/imu_roll_deg
state/updated/imu_pitch_deg
state/updated/imu_yaw_deg
state/updated/base_roll_deg
state/updated/base_pitch_deg
state/updated/base_yaw_deg
```

角度按右手系 ZYX yaw-pitch-roll 输出，范围为 `[-180°, 180°]`。解包和相对归零由诊断节点完成，避免在线发布器保存跨帧隐式状态。

关键帧事件继续使用已有：

```text
keyframe/id
keyframe/created
```

ESKF、NDT和PGO沿用现有指标，不另造重复字段。

## 6. 关键帧诊断节点

### 6.1 输入

- `/lightning/debug/metrics/dictionary`
- `/lightning/debug/metrics/data`
- `/articulated_steering_controller/odom`

节点解析 compact dictionary/data 消息，并缓存：

- 最新的 predicted/updated base yaw；
- 指标时间戳；
- 有界 odom 时间序列；
- 首个有效关键帧的 Lightning 和 odom 航向零点。

### 6.2 时间匹配

每次 `keyframe/created == 1` 时，以同一指标消息的时间戳为关键帧时间。诊断节点选择绝对时间差最小的 odom 样本。

默认最大允许差：

```text
100 ms
```

超过该值时：

- 本关键帧写入 `matched=false`；
- 不更新角度解包状态；
- 不参与阈值判断；
- 控制台打印同步缺失原因；
- 继续处理后续关键帧，不暂停回放。

### 6.3 航向解包与归一化

Lightning 和 odom 分别按相邻有效关键帧做最短角差解包：

```text
delta = wrap_to_180(current - previous)
unwrapped += delta
relative = unwrapped - first_unwrapped
```

比较量为：

```text
yaw_error_deg = wrap_to_180(
    lightning_updated_base_relative_yaw_deg - odom_relative_yaw_deg)
```

预测姿态单独记录和绘图，但阈值使用 updated base yaw，因为它是进入关键帧和后端的前端最终状态。

### 6.4 自动暂停

默认阈值：

```text
abs(yaw_error_deg) >= 10°
```

只在第一次跨越阈值时调用 rosbag player 的 pause 服务。重复异常只记录，不重复调用服务。暂停信息必须包含：

- keyframe ID；
- keyframe timestamp；
- predicted/updated Lightning base相对 yaw；
- odom 相对 yaw；
- yaw error；
- odom 同步时间差。

服务不可用时不终止节点：记录 `pause_requested=true, pause_succeeded=false`，继续写证据并打印明确错误。

### 6.5 单步复核

自动暂停锁定首次异常关键帧后，DataCapture 中所有关键帧事件仍强制保存。离线 stage replay 用 `seek_frame` 定位该关键帧，再检查前后至少 3 个已捕获关键帧。stage replay 不负责重放时序指标；PlotJuggler使用本次在线指标或录制的调试 rosbag。

## 7. 诊断输出

### 7.1 PlotJuggler 对照消息

诊断节点发布带原始关键帧时间戳的标准 ROS stamped 消息，至少包含：

```text
Lightning predicted base relative yaw
Lightning updated base relative yaw
odom relative yaw
yaw error
keyframe ID
odom sync dt
```

使用标准 ROS消息而不是与 Lightning 共用 compact dictionary UUID，避免两个独立发布器的字典冲突。

PlotJuggler布局新增 `Yaw Debug` 页，并同时展示：

- 上述逐关键帧对照曲线；
- `eskf/dx_rotation`；
- `eskf/residual_mean`、`eskf/residual_max`；
- `pgo/max_rotation_correction_deg`。

### 7.2 逐关键帧 CSV

只在关键帧自动诊断启用时写入：

```text
<capture_dir>/diagnostics/keyframe_yaw_comparison.csv
```

字段固定为：

```text
keyframe_id
keyframe_timestamp
odom_timestamp
matched
sync_dt_ms
predicted_base_yaw_deg
updated_base_yaw_deg
odom_yaw_deg
predicted_base_relative_yaw_deg
updated_base_relative_yaw_deg
odom_relative_yaw_deg
yaw_error_deg
threshold_triggered
pause_requested
pause_succeeded
```

CSV每个关键帧一行，不复制点云、原始 IMU或完整 odom。文件采用启动时覆盖模式，避免不同回放混在同一时间轴。

## 8. 配置与默认行为

主回放脚本支持：

```text
LIGHTNING_KEYFRAME_DEBUG_ENABLED=false
LIGHTNING_KEYFRAME_YAW_THRESHOLD_DEG=10
LIGHTNING_KEYFRAME_ODOM_MAX_DT_MS=100
```

当 `LIGHTNING_KEYFRAME_DEBUG_ENABLED=false`：

- 不启动诊断节点；
- bag replay 不额外回放底盘 odom；
- 不生成诊断 CSV；
- 不发布关键帧对照 topic；
- 不改变 Lightning 当前生产行为。

当该开关为 `true`，脚本必须同时开启 Lightning live timeseries；如果用户显式给出冲突配置，脚本以清晰错误退出，而不是静默运行一条没有数据的诊断链。

## 9. 测试策略

### 9.1 C++单元测试

- identity外参下 IMU/base欧拉角一致；
- 使用已知 `R_imu_base` 合成 `R_world_base`，验证不能退化为 IMU yaw；
- 输出角度范围和单位正确；
- 在线调试主开关关闭时不创建发布器的既有行为保持不变。

### 9.2 Python单元测试

- `179° → -179°` 解包为 `+2°`，而不是 `-358°`；
- 最近 odom 匹配选择正确；
- 超过 `max_dt` 时返回 unmatched；
- 首个有效关键帧归零；
- `9.9°` 不触发、`10.0°` 触发；
- 只在首次跨阈值时请求暂停；
- pause服务失败仍写完整 CSV行。

### 9.3 脚本测试

- 默认开关关闭且不回放 odom；
- 开启时回放 odom并启动诊断节点；
- 阈值和同步窗口环境变量传递正确；
- 诊断开启但 live timeseries关闭时快速失败；
- cleanup能终止诊断节点并恢复临时配置。

### 9.4 集成验证

使用 `slam_debug_20260730_215238_noned`：

1. `--parallel-workers 4` 构建 Lightning；
2. 开启 DataCapture、live timeseries和关键帧诊断；
3. 确认字典、数据、odom和关键帧对照 topic存在；
4. 确认 CSV逐关键帧增长；
5. 确认首次 `|yaw_error| >= 10°` 时回放暂停；
6. 记录首个异常关键帧及前后至少3个关键帧的姿态、ESKF残差和点云；
7. 不把诊断成功误报为 SLAM根因已修复。

## 10. 成功标准

- 不改动任何 SLAM估计输入即可得到逐关键帧三方航向对照；
- 能自动定位并暂停在第一个超过阈值的关键帧；
- PlotJuggler能同步显示预测、更新、odom、ESKF和PGO旋转指标；
- 单个CSV完整记录自动判断依据；
- 所有新功能默认关闭；
- 单元测试、脚本测试和 worker=4构建通过；
- 实际回放产出可复核的“首次分叉关键帧”，而不是仅凭最终重影地图猜测根因。
