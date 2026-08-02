# Lightning Debug Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 Lightning 增加默认关闭的在线点云/时序调试发布，并提供 DataCapture 离线阶段回放、静态 PNG、Plotly HTML、Lichtblick/PlotJuggler 布局和 Obsidian 教学报告。

**Architecture:** Lightning 内新增单一 `DebugVisualization` ROS 输出组件，前端和后端只提交已有结果；组件按独立采样开关发布 `PointCloud2`、`Path`、`MarkerArray` 和 PlotJuggler `Dictionary/DataPoints`。离线工具位于 `hw_tests/lightning_capture_visualizer`，读取 DataCapture 而不链接 Lightning 算法库。

**Tech Stack:** C++17、ROS 2 Jazzy、PCL、plotjuggler_msgs、Python 3、rclpy、Open3D、NumPy、Pandas、Matplotlib、Plotly、pytest。

## Global Constraints

- `data_capture.enabled`、在线点云、在线时序、离线回放四项开关必须独立且默认关闭。
- 所有调试开关关闭时不创建调试 publisher、不复制点云、不写调试文件、不启动线程。
- 调试路径不得修改 ESKF、IVox、NDT、PGO 的输入或数学逻辑。
- 普通帧按配置频率发布；关键帧、回环和 PGO 允许强制发布。
- 生产启动脚本不得自动打开 Lichtblick 或 PlotJuggler。
- 不引入新的 Python pip 依赖；使用机器已有 Open3D、Matplotlib、Plotly、Pandas。

---

### Task 1: 配置和在线发布器骨架

**Files:**
- Create: `src/common/debug_visualization.h`
- Create: `src/common/debug_visualization.cc`
- Create: `test/test_debug_visualization.cc`
- Modify: `src/CMakeLists.txt`
- Modify: `package.xml`
- Modify: `config/articulated_vehicle_map.yaml`
- Modify: `src/core/system/slam.h`
- Modify: `src/core/system/slam.cc`

**Interfaces:**
- Consumes: `rclcpp::Node::SharedPtr`、YAML `debug_visualization` 配置。
- Produces: `DebugVisualization::Params`、`enabled()`、`cloudEnabled(frame_id, force)`、`timeseriesEnabled(frame_id, force)`、`publishCloud(...)`、`publishMetrics(...)`。

- [ ] **Step 1: 写失败测试**

测试默认参数全部关闭、频率小于 1 抛出异常、帧 0/10 命中十帧采样、强制事件绕过采样但不绕过总开关。

- [ ] **Step 2: 运行 RED**

Run: `source /opt/ros/jazzy/setup.bash && colcon build --packages-select lightning && colcon test --packages-select lightning --ctest-args -R test_debug_visualization --output-on-failure`

Expected: 编译失败，提示 `common/debug_visualization.h` 不存在。

- [ ] **Step 3: 实现最小发布器**

`Params` 包含两个开关、两个频率、点云/Marker 显示点数上限。构造函数只在对应开关打开时创建 publisher。`publishCloud` 使用 `pcl::toROSMsg`，`publishMetrics` 维护稳定字段字典并发布 `plotjuggler_msgs::msg::Dictionary/DataPoints`。

- [ ] **Step 4: 接入 SlamSystem 配置**

先创建 ROS node，再构造调试发布器；离线 `SlamSystem` 不创建在线 publisher。YAML 默认：

```yaml
debug_visualization:
  live_cloud_enabled: false
  live_cloud_every_n_frames: 10
  live_timeseries_enabled: false
  live_timeseries_every_n_frames: 1
  max_cloud_points: 50000
  max_correspondence_markers: 2000

stage_replay:
  enabled: false
```

- [ ] **Step 5: 运行 GREEN 和全量测试**

Run: `source /opt/ros/jazzy/setup.bash && colcon build --packages-select lightning && colcon test --packages-select lightning && colcon test-result --verbose`

Expected: 0 failures。

- [ ] **Step 6: 提交**

```bash
git add src/common/debug_visualization.* test/test_debug_visualization.cc src/CMakeLists.txt package.xml config/articulated_vehicle_map.yaml src/core/system/slam.*
git commit -m "feat: 添加 Lightning 在线调试发布器"
```

### Task 2: 接入前端、关键帧和后端锚点

**Files:**
- Modify: `src/core/lio/laser_mapping.h`
- Modify: `src/core/lio/laser_mapping.cc`
- Modify: `src/core/loop_closing/loop_closing.h`
- Modify: `src/core/loop_closing/loop_closing.cc`
- Modify: `src/core/system/slam.cc`
- Modify: `test/test_debug_visualization.cc`

**Interfaces:**
- Consumes: Task 1 的 `std::shared_ptr<DebugVisualization>`。
- Produces: `/lightning/debug/frontend/*`、`/iteration/*`、`/backend/*`、`/path/*` 和 PlotJuggler 指标。

- [ ] **Step 1: 写失败测试**

使用发布记录回调验证：关闭时不接收事件；采样帧接收前端 stage；关键帧 `force=true` 发布；指标名称保持唯一索引。

- [ ] **Step 2: 运行 RED**

Run targeted test; Expected: 缺少事件提交或 topic 映射而失败。

- [ ] **Step 3: 接入前端**

在现有 DataCapture 锚点附近发布入口、预处理、IMU body、降采样、观测输入、迭代接受/拒绝/邻域、最终状态、IVox 增量和关键帧。指标覆盖 IMU、ESKF 结果、残差、特征数、状态增量、IVox 和关键帧判定。

- [ ] **Step 4: 接入后端**

发布 NDT source/target/aligned、NDT score/迭代/收敛、LIO/优化 Path、回环约束与 PGO 修正指标。后端事件使用 `force=true`，但仍受对应总开关控制。

- [ ] **Step 5: 验证**

Run targeted and full Lightning tests; Expected: 0 failures。

- [ ] **Step 6: 提交**

```bash
git add src/core/lio/laser_mapping.* src/core/loop_closing/loop_closing.* src/core/system/slam.cc test/test_debug_visualization.cc
git commit -m "feat: 发布 Lightning 前后端调试数据"
```

### Task 3: DataCapture 离线索引与 ROS 阶段回放器

**Files:**
- Create: `hw_tests/lightning_capture_visualizer/capture_model.py`
- Create: `hw_tests/lightning_capture_visualizer/replay_capture_ros.py`
- Create: `hw_tests/lightning_capture_visualizer/stage_replay.yaml`
- Create: `hw_tests/lightning_capture_visualizer/test_capture_model.py`

**Interfaces:**
- Consumes: DataCapture 根目录。
- Produces: `CaptureRun.frames`、`CaptureRun.loop_events`、`CaptureRun.pgo_events` 和与在线相同的 ROS topic。

- [ ] **Step 1: 写最小 fixture 和失败测试**

测试按数字排序 `frame_2/frame_10`、缺失可选阶段、回环/PGO 独立索引、默认 `stage_replay.enabled=false`、未显式 `--enable` 时拒绝发布。

- [ ] **Step 2: 运行 RED**

Run: `python3 -m pytest hw_tests/lightning_capture_visualizer/test_capture_model.py -q`

Expected: `capture_model` 导入失败。

- [ ] **Step 3: 实现只读目录模型**

只解析目录、metadata 和 CSV；不修改捕获目录。对缺失必需文件给出路径化错误，对可选事件返回空集合。

- [ ] **Step 4: 实现 ROS 回放**

支持 `--enable --capture-dir --start-frame --end-frame --rate --loop`；发布 `/clock`、stage 点云、Path 和离线指标；提供 pause/resume/step-next/step-prev/seek 服务。

- [ ] **Step 5: 运行测试和 `--help`**

Expected: pytest 通过，未带 `--enable` 时退出且不创建 ROS node。

- [ ] **Step 6: 主仓库提交**

```bash
git add hw_tests/lightning_capture_visualizer
git commit -m "feat: 添加 Lightning 数据捕获离线回放器"
```

### Task 4: 静态和交互式分析器

**Files:**
- Create: `hw_tests/lightning_capture_visualizer/analyze_capture.py`
- Create: `hw_tests/lightning_capture_visualizer/plotting.py`
- Create: `hw_tests/lightning_capture_visualizer/test_plotting.py`
- Create: `hw_tests/lightning_capture_visualizer/layouts/lichtblick_lightning_learning.json`
- Create: `hw_tests/lightning_capture_visualizer/layouts/plotjuggler_lightning_learning.xml`

**Interfaces:**
- Consumes: Task 3 的 `CaptureRun`。
- Produces: `summary.json`、`timeseries.csv`、PNG、Plotly HTML、Markdown 和两套布局。

- [ ] **Step 1: 写失败测试**

用小型 fixture 验证代表帧选择、显示点数确定性限制、轨迹修正统计、报告链接存在且布局文件可解析。

- [ ] **Step 2: 运行 RED**

Expected: `plotting` 导入失败。

- [ ] **Step 3: 实现汇总和代表事件选择**

选择初始化后首帧、有效特征最低帧、残差最高帧、一个关键帧、一次接受回环、PGO 最大修正事件；在 `summary.json` 记录选择原因。

- [ ] **Step 4: 实现图表**

生成前端阶段俯视/侧视、IMU/状态、ESKF 迭代、IVox/关键帧、NDT 金字塔、LIO/PGO 轨迹 PNG；生成相同主题的 Plotly HTML。

- [ ] **Step 5: 生成布局与 Markdown**

布局预置在线/离线统一 topic；报告逐节点解释输入、处理、输出、坐标系、正常形态和异常回查点。

- [ ] **Step 6: 测试并提交**

Run pytest and JSON/XML parse checks; Expected: 0 failures。

### Task 5: 回放脚本四开关和 GUI 辅助启动

**Files:**
- Modify: `hw_tests/bag_replay_polka_lightning_slam.sh`
- Create: `hw_tests/lightning_capture_visualizer/open_learning_tools.sh`
- Create: `hw_tests/lightning_capture_visualizer/test_replay_options.sh`

**Interfaces:**
- Consumes: 环境变量 `LIGHTNING_CAPTURE_ENABLED`、`LIGHTNING_LIVE_CLOUD_ENABLED`、`LIGHTNING_LIVE_TIMESERIES_ENABLED`、`LIGHTNING_STAGE_REPLAY_ENABLED`。
- Produces: 对 YAML 的作用域安全替换；显式辅助脚本启动 Lichtblick/PlotJuggler。

- [ ] **Step 1: 写 shell 失败测试**

复制 YAML 到临时目录，验证四开关默认 false、分别开启只改变自己的字段、路径包含空格时保持有效 YAML。

- [ ] **Step 2: 运行 RED**

Expected: 新环境变量未生效。

- [ ] **Step 3: 实现脚本开关**

只修改对应 YAML 块，trap 保存并恢复原文件内容。生产默认不启动 GUI；`open_learning_tools.sh` 必须由用户显式运行。

- [ ] **Step 4: 验证并提交**

Run: `bash -n`、shell fixture test、YAML parse。Expected: 0 failures。

### Task 6: 完整 bag 教学运行、分析和 Obsidian 输出

**Files:**
- Generate: `/tmp/lightning_capture_20260803_learning/`
- Generate: `/tmp/lightning_capture_20260803_learning_report/`
- Create outside repo: `/home/jarvis/文档/Obsidian Vault/Cache/Daily Notes/20260803 Lightning SLAM 全流程数据流学习.md`

**Interfaces:**
- Consumes: 前五项实现与 `slam_debug_20260730_215238_noned`。
- Produces: 一次完整捕获、在线 topic 证据、静态/交互式报告和 Obsidian 笔记。

- [ ] **Step 1: 构建并跑完整测试**

Run Lightning build/test、Python pytest、shell tests。Expected: 全部通过。

- [ ] **Step 2: 运行完整 bag**

开启 DataCapture、在线点云、在线时序和回环；前端每 10 帧、ESKF 每次迭代、IMU 每 10 样本。记录 topic list、topic hz 和运行日志。

- [ ] **Step 3: 验证捕获**

检查前端、关键帧、候选、NDT、PGO、地图目录；所有 CSV 列数一致；抽样 PCD 可由 PCL/Open3D 读取。

- [ ] **Step 4: 生成报告**

运行 `analyze_capture.py`，验证所有 PNG、HTML、布局和 Markdown 非空且内部链接有效。

- [ ] **Step 5: 启动离线回放和 GUI 烟测**

显式开启 stage replay；验证 ROS topic、暂停/单步/seek；启动 Lichtblick 和 PlotJuggler 加载布局，记录截图。

- [ ] **Step 6: 写入 Obsidian 并恢复默认配置**

复制 Markdown 与图片引用到 Daily Notes；确认四项开关恢复 false，工作树不残留运行时 YAML 改写。

- [ ] **Step 7: 最终验证与提交**

Run `git diff --check`、全量测试、报告完整性检查；只提交源码、测试、布局和文档，不提交 `/tmp` 捕获数据。
