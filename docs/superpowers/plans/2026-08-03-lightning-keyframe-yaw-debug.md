# Lightning Keyframe Yaw Debug Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 增加默认关闭的 Lightning/底盘 odom 逐关键帧航向对照，在首次航向误差超过阈值时自动暂停 bag 回放并保留 PlotJuggler 与 CSV 证据。

**Architecture:** Lightning 仅从自身 `NavState` 和实际运行时 `R_imu_base` 发布 IMU/base欧拉角；独立 Python 节点解析 compact PlotJuggler metrics、匹配 odom并做解包、归零、阈值判断。主回放脚本只在显式开关打开时回放参考 odom并启动诊断节点，参考数据不进入 SLAM估计。

**Tech Stack:** C++17、Eigen/Sophus、ROS 2 Jazzy、plotjuggler_msgs、nav_msgs、geometry_msgs、rclpy、pytest、Bash。

## Global Constraints

- 底盘 odom 只用于诊断，不得成为 Lightning 状态观测或PGO约束。
- 所有新增功能生产默认关闭。
- Lightning 构建限制 `--parallel-workers 4`。
- 使用 TDD：每项生产代码必须先有能因缺失功能而失败的测试。
- 不改动现有 `src/rslidar_sdk` 工作树状态。
- 实际回放使用 `/home/jarvis/projects/iRail-Truck/bags/slam_debug_20260730_215238_noned`。

---

### Task 1: Lightning姿态指标

**Files:**
- Modify: `src/common/debug_visualization.h`
- Modify: `src/common/debug_visualization.cc`
- Modify: `src/core/lio/laser_mapping.cc`
- Modify: `test/test_debug_visualization.cc`

**Interfaces:**
- Consumes: `NavState::rot_` as `R_world_imu`; `LaserMapping::offset_R_lidar_fixed_` as runtime `R_imu_base`.
- Produces: `DebugVisualization::AttitudeDegrees`, `ToAttitudeDegrees(R_world_imu, R_imu_base)`, and twelve stable `state/{predicted,updated}/{imu,base}_{roll,pitch,yaw}_deg` metrics.

- [ ] **Step 1: 写姿态合成失败测试**

在 `test_debug_visualization.cc` 增加 identity和非identity外参测试：

```cpp
TEST(DebugVisualizationAttitude, ComposesRuntimeImuFromBaseRotation) {
    const Mat3d R_world_imu =
        Eigen::AngleAxisd(30.0 * M_PI / 180.0, Vec3d::UnitZ()).toRotationMatrix();
    const Mat3d R_imu_base =
        Eigen::AngleAxisd(20.0 * M_PI / 180.0, Vec3d::UnitZ()).toRotationMatrix();

    const auto attitude = DebugVisualization::ToAttitudeDegrees(R_world_imu, R_imu_base);

    EXPECT_NEAR(attitude.imu_yaw_deg, 30.0, 1e-9);
    EXPECT_NEAR(attitude.base_yaw_deg, 50.0, 1e-9);
}
```

- [ ] **Step 2: 运行测试确认RED**

Run:

```bash
unset COLCON_CURRENT_PREFIX
source /opt/ros/jazzy/setup.bash
colcon test --packages-select lightning --ctest-args -R test_debug_visualization --event-handlers console_direct+
```

Expected: FAIL to compile because `AttitudeDegrees`/`ToAttitudeDegrees` do not exist.

- [ ] **Step 3: 实现最小姿态转换API**

在 `debug_visualization.h` 定义：

```cpp
struct AttitudeDegrees {
    double imu_roll_deg = 0.0;
    double imu_pitch_deg = 0.0;
    double imu_yaw_deg = 0.0;
    double base_roll_deg = 0.0;
    double base_pitch_deg = 0.0;
    double base_yaw_deg = 0.0;
};

static AttitudeDegrees ToAttitudeDegrees(
    const Mat3d& R_world_imu, const Mat3d& R_imu_base);
static Metrics AttitudeMetrics(
    const std::string& phase, const Mat3d& R_world_imu, const Mat3d& R_imu_base);
```

在 `.cc` 用显式 ZYX 公式计算并转换为 degree，避免 Eigen欧拉角在等价分支间跳变：

```cpp
yaw = std::atan2(R(1, 0), R(0, 0));
pitch = std::atan2(-R(2, 0), std::hypot(R(0, 0), R(1, 0)));
roll = std::atan2(R(2, 1), R(2, 2));
```

base矩阵严格使用：

```cpp
const Mat3d R_world_base = R_world_imu * R_imu_base;
```

- [ ] **Step 4: 接入 predicted和updated指标**

在两个既有 `publishMetrics` 调用中合并 `AttitudeMetrics` 返回值，使用同一帧时间戳和 `debug_frame_id_`，不增加新的发布频率或ROS topic。

- [ ] **Step 5: 运行测试确认GREEN**

Run Task 1 Step 2 command and `colcon test-result --verbose`.

Expected: `test_debug_visualization` passes with zero failures.

- [ ] **Step 6: 提交Task 1**

```bash
git add src/common/debug_visualization.h src/common/debug_visualization.cc \
        src/core/lio/laser_mapping.cc test/test_debug_visualization.cc
git commit -m "feat: 发布 Lightning 车体姿态调试指标" \
  -m "- 按运行时 IMU 到 base 外参合成预测与更新姿态
- 为 PlotJuggler 发布稳定的 IMU 和 base 欧拉角字段"
```

---

### Task 2: 纯Python关键帧航向分析核心

**Files:**
- Create: `hw_tests/lightning_capture_visualizer/keyframe_yaw_monitor.py`（主仓库）
- Create: `hw_tests/lightning_capture_visualizer/test_keyframe_yaw_monitor.py`（主仓库）

**Interfaces:**
- Consumes: timestamped Lightning metrics, timestamped odom yaw, keyframe event.
- Produces: `wrap_degrees`, `YawUnwrapper`, `TimedOdomBuffer`, `KeyframeYawAnalyzer`, immutable per-keyframe result row.

- [ ] **Step 1: 写角度解包和时间匹配失败测试**

测试必须覆盖：

```python
def test_unwrap_crosses_positive_180_without_a_full_turn():
    unwrap = YawUnwrapper()
    assert unwrap.update(179.0) == pytest.approx(179.0)
    assert unwrap.update(-179.0) == pytest.approx(181.0)

def test_odom_match_rejects_sample_outside_window():
    buffer = TimedOdomBuffer(max_samples=10)
    buffer.append(1.0, 5.0)
    assert buffer.nearest(1.2, max_dt_seconds=0.1) is None
```

- [ ] **Step 2: 运行pytest确认RED**

Run:

```bash
python3 -m pytest hw_tests/lightning_capture_visualizer/test_keyframe_yaw_monitor.py -q
```

Expected: FAIL because module/API is missing.

- [ ] **Step 3: 实现纯分析核心**

实现：

```python
def wrap_degrees(angle: float) -> float: ...

class YawUnwrapper:
    def update(self, wrapped_degrees: float) -> float: ...

class TimedOdomBuffer:
    def append(self, timestamp: float, yaw_degrees: float) -> None: ...
    def nearest(self, timestamp: float, max_dt_seconds: float) -> tuple[float, float] | None: ...

class KeyframeYawAnalyzer:
    def process_keyframe(
        self, keyframe_id: int, timestamp: float,
        predicted_base_yaw_deg: float, updated_base_yaw_deg: float,
    ) -> KeyframeYawResult: ...
```

只有 matched结果才推进三个解包器和归零状态。`threshold_triggered` 只在首次 `abs(error) >= threshold` 时为 true。

- [ ] **Step 4: 增加归零、阈值和单次触发测试**

覆盖首帧全为0、`9.9°`不触发、`10.0°`触发、后续异常不重复触发、unmatched不改变状态。

- [ ] **Step 5: 运行pytest确认GREEN**

Run Task 2 Step 2 command.

Expected: all monitor unit tests pass.

---

### Task 3: ROS监视器、CSV和自动暂停

**Files:**
- Modify: `hw_tests/lightning_capture_visualizer/keyframe_yaw_monitor.py`（主仓库）
- Modify: `hw_tests/lightning_capture_visualizer/test_keyframe_yaw_monitor.py`（主仓库）
- Modify: `hw_tests/lightning_capture_visualizer/layouts/plotjuggler_lightning_learning.xml`（主仓库）
- Modify: `hw_tests/lightning_capture_visualizer/README.md`（主仓库）

**Interfaces:**
- Consumes: `/lightning/debug/metrics/dictionary`, `/lightning/debug/metrics/data`, `/articulated_steering_controller/odom`, `/rosbag2_player/pause`.
- Produces: `/lightning/debug/keyframe_yaw`, `/lightning/debug/keyframe_yaw_error`, and `<capture_dir>/diagnostics/keyframe_yaw_comparison.csv`.

- [ ] **Step 1: 写CSV格式和compact metrics解析失败测试**

测试固定CSV字段顺序，并验证 dictionary UUID变化后按新字典解析 `state/updated/base_yaw_deg` 与 `keyframe/id`。

- [ ] **Step 2: 运行pytest确认RED**

Run Task 2 Step 2 command.

Expected: FAIL because parser/CSV writer is missing.

- [ ] **Step 3: 实现ROS wrapper**

`main()` 接受：

```text
--capture-dir
--threshold-deg
--odom-max-dt-ms
--pause-service /rosbag2_player/pause
```

发布两个 `geometry_msgs/msg/Vector3Stamped`：

```text
/lightning/debug/keyframe_yaw
  x = predicted base relative yaw
  y = updated base relative yaw
  z = odom relative yaw

/lightning/debug/keyframe_yaw_error
  x = yaw error
  y = keyframe ID
  z = odom sync dt ms
```

首次触发时异步调用 `std_srvs/srv/Trigger` pause服务。服务不可用或失败只更新CSV状态并打印错误，不中止节点。

- [ ] **Step 4: 实现CSV覆盖写入**

节点启动时创建 `diagnostics` 并以 `w` 模式写固定header；每个关键帧立即 `flush()`，保证自动暂停或Ctrl-C后证据完整。

- [ ] **Step 5: 更新PlotJuggler布局与README**

新增 `Yaw Debug` 页，包含标准消息曲线和既有 `eskf/dx_rotation`、残差、`pgo/max_rotation_correction_deg`；README加入启动、阈值、暂停和CSV说明。

- [ ] **Step 6: 运行pytest确认GREEN**

Run Task 2 Step 2 command and existing visualizer tests.

Expected: all tests pass.

- [ ] **Step 7: 提交Tasks 2-3主仓库文件**

主仓库提交在Task 4脚本接入后统一完成，避免提交一个不可启动的监视器。

---

### Task 4: Bag replay显式开关接入

**Files:**
- Modify: `hw_tests/bag_replay_polka_lightning_slam.sh`（主仓库）
- Create: `hw_tests/lightning_capture_visualizer/test_keyframe_debug_options.sh`（主仓库）

**Interfaces:**
- Consumes: `LIGHTNING_KEYFRAME_DEBUG_ENABLED`, `LIGHTNING_KEYFRAME_YAW_THRESHOLD_DEG`, `LIGHTNING_KEYFRAME_ODOM_MAX_DT_MS`.
- Produces: conditional odom replay, monitor process lifecycle, config validation and logs.

- [ ] **Step 1: 写脚本失败测试**

测试静态/隔离运行行为：默认不含参考 odom；开启时加入 odom topic和监视器命令；阈值与同步窗口传参；cleanup包含 monitor PID；显式关闭 live timeseries与debug开启冲突时退出。

- [ ] **Step 2: 运行测试确认RED**

Run:

```bash
bash hw_tests/lightning_capture_visualizer/test_keyframe_debug_options.sh
```

Expected: FAIL because keyframe debug options are absent.

- [ ] **Step 3: 实现环境变量与验证**

默认：

```bash
KEYFRAME_DEBUG_ENABLED=false
KEYFRAME_YAW_THRESHOLD_DEG=10
KEYFRAME_ODOM_MAX_DT_MS=100
```

debug开启且 `LIGHTNING_LIVE_TIMESERIES_ENABLED` 未设置时自动设为true；若用户显式设置false则快速失败。

- [ ] **Step 4: 条件回放odom并管理监视器**

用数组构建额外topics，避免复制整段 `ros2 bag play`；Lightning启动后启动监视器，保存 `KEYFRAME_DEBUG_PID`，cleanup中先发INT再兜底kill。

- [ ] **Step 5: 运行脚本测试确认GREEN**

Run Task 4 Step 2 command plus `test_replay_options.sh`.

Expected: both scripts exit 0.

- [ ] **Step 6: 提交主仓库实现**

```bash
git add hw_tests/bag_replay_polka_lightning_slam.sh \
        hw_tests/lightning_capture_visualizer/keyframe_yaw_monitor.py \
        hw_tests/lightning_capture_visualizer/test_keyframe_yaw_monitor.py \
        hw_tests/lightning_capture_visualizer/test_keyframe_debug_options.sh \
        hw_tests/lightning_capture_visualizer/layouts/plotjuggler_lightning_learning.xml \
        hw_tests/lightning_capture_visualizer/README.md
git commit -m "feat: 添加关键帧航向自动诊断" \
  -m "- 对齐 Lightning 与底盘 odom 航向并保存逐关键帧证据
- 首次超过阈值时暂停 bag 回放并扩展 PlotJuggler 布局"
```

---

### Task 5: 构建与实际回放

**Files:**
- Verify: Lightning and main workspace outputs under `install/`, `/tmp/lightning_capture_keyframe_debug`, `/tmp/polka_lightning_logs`.

**Interfaces:**
- Consumes: all prior tasks.
- Produces: first divergent keyframe evidence and a paused replay session.

- [ ] **Step 1: 完整测试**

Run:

```bash
python3 -m pytest hw_tests/lightning_capture_visualizer -q
bash hw_tests/lightning_capture_visualizer/test_replay_options.sh
bash hw_tests/lightning_capture_visualizer/test_keyframe_debug_options.sh
```

Expected: zero failures.

- [ ] **Step 2: worker=4构建Lightning**

Run:

```bash
unset COLCON_CURRENT_PREFIX
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lightning --parallel-workers 4
```

Expected: package `lightning` finishes successfully without OOM.

- [ ] **Step 3: 运行Lightning测试**

Run:

```bash
source install/setup.bash
colcon test --packages-select lightning --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests.

- [ ] **Step 4: 开启关键帧诊断回放**

Run:

```bash
LIGHTNING_CAPTURE_ENABLED=true \
LIGHTNING_CAPTURE_DIR=/tmp/lightning_capture_keyframe_debug \
LIGHTNING_CAPTURE_EVERY_N_FRAMES=10 \
LIGHTNING_LIVE_CLOUD_ENABLED=true \
LIGHTNING_LIVE_CLOUD_EVERY_N_FRAMES=10 \
LIGHTNING_KEYFRAME_DEBUG_ENABLED=true \
LIGHTNING_KEYFRAME_YAW_THRESHOLD_DEG=10 \
LIGHTNING_KEYFRAME_ODOM_MAX_DT_MS=100 \
LIGHTNING_LOOP_CLOSING_ENABLED=true \
./hw_tests/bag_replay_polka_lightning_slam.sh \
  /home/jarvis/projects/iRail-Truck/bags/slam_debug_20260730_215238_noned \
  base_footprint before
```

Expected: monitor logs each matched keyframe and pauses rosbag player at first `|yaw_error| >= 10°`.

- [ ] **Step 5: 验证运行证据**

检查topics、pause服务、monitor日志和CSV。报告首个异常关键帧、前后3个关键帧的预测/更新/odom yaw、同步差、ESKF旋转修正和残差；不得在证据不足时宣布根因已修复。

- [ ] **Step 6: 新鲜验证与工作树审计**

Run:

```bash
git diff --check
git status --short
git -C src/lightning-lm diff --check
git -C src/lightning-lm status --short
```

Expected: only intended commits/submodule pointer plus pre-existing `src/rslidar_sdk` state remain.
