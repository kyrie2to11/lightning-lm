# Lightning Single-Body DataCapture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a configurable capture system that exposes every semantic data boundary in Lightning's active Polka merged single-body LIO frontend and loop/NDT/PGO backend.

**Architecture:** Replace the mutable, frame-global header-only recorder with a thread-safe session recorder whose methods take immutable frame or backend event IDs. LaserMapping owns the recorder and LoopClosing receives a non-owning shared lifetime-safe reference from SlamSystem. Point-cloud transitions write compressed PCD while state, synchronization, decisions, transforms and graph data write append-only CSV/YAML manifests.

**Tech Stack:** C++17, ROS 2, PCL, Eigen, yaml-cpp, GoogleTest, Lightning ESKF/IVox/Miao optimizer.

## Global Constraints

- Scope begins at the Polka merged cloud received by Lightning and excludes Polka internals.
- Scope covers only the active single-body LIO path and excludes the unused multibody branch.
- Capturing must not change point order, filter decisions, estimator state, loop decisions or map output.
- Ordinary frontend capture frequency is configured by `every_n_frames`; keyframe, loop and PGO events have independent force-capture switches.
- Every cloud artifact declares its coordinate frame and semantic stage in metadata.
- Capture I/O failures must not abort SLAM.

---

### Task 1: Session recorder and sampling policy

**Files:**
- Create: `src/common/data_capture.cc`
- Modify: `src/common/data_capture.h`
- Modify: `src/CMakeLists.txt`
- Create: `test/test_data_capture.cc`
- Modify: `config/articulated_vehicle_map.yaml`

**Interfaces:**
- Produces: `DataCapture::Configure`, `BeginProcessedFrame`, `ShouldCaptureFrame`, `SaveFrontendCloud`, `AppendFrameEvent`, `AppendImu`, `BeginLoopEvent`, `SaveBackendCloud`, `AppendBackendRow`, `SaveMapCloud`.
- Produces: immutable `FrameContext {id, lidar_begin_time, lidar_end_time, sampled}`.

- [ ] Write `test_data_capture.cc` tests that assert disabled capture creates nothing, frame stride selects 0/2/4, frame bounds are inclusive, cloud paths use six-digit frame IDs, and backend events do not depend on current frontend frame.
- [ ] Build the test and verify it fails because the new API does not exist.
- [ ] Implement parameter validation, directory creation, CSV headers, compressed/binary PCD writing, optional write-copy point cap and exception-safe logging.
- [ ] Parse every documented YAML option with defaults preserving disabled behavior.
- [ ] Run `test_data_capture` and verify all assertions pass.

### Task 2: Frontend input, sync, IMU and preprocessing anchors

**Files:**
- Modify: `src/core/system/slam.cc`
- Modify: `src/core/lio/laser_mapping.h`
- Modify: `src/core/lio/laser_mapping.cc`
- Modify: `src/core/lio/imu_processing.hpp`

**Interfaces:**
- Consumes: Task 1 frame/event APIs.
- Produces: stable processed frame IDs assigned after successful `SyncPackages`.
- Produces: input/preprocess/body clouds plus synchronization and IMU state rows.

- [ ] Add a testable `DataCapture::ShouldCaptureFrame`-based frame lifecycle and verify callback count is not used as processed frame identity.
- [ ] Capture the converted Polka input before Lightning filtering and the standardized preprocess output without changing the algorithm cloud.
- [ ] Record lidar/IMU buffer events and successful synchronization ranges/counts.
- [ ] Record raw IMU according to `imu_sample_stride`.
- [ ] Expose ImuProcess initialization summary and per-frame prediction summary without changing its estimator calculations.
- [ ] Save the IMU/body output and record that pointwise deskew is bypassed for Polka input.
- [ ] Build and run existing pointcloud and IMU tests.

### Task 3: Iterative ESKF observation and IVox anchors

**Files:**
- Modify: `src/core/lio/eskf.hpp`
- Modify: `src/core/lio/eskf.cc`
- Modify: `src/core/lio/laser_mapping.h`
- Modify: `src/core/lio/laser_mapping.cc`

**Interfaces:**
- Consumes: current sampled `FrameContext`.
- Produces: ESKF iteration callback containing iteration, states, residuals, eigenvalues, rank, increment and validity.
- Produces: per-iteration observation clouds and IVox insertion clouds.

- [ ] Add an ESKF iteration callback test fixture that verifies one callback row per executed observation update.
- [ ] Add the callback payload and invoke it for valid, rejected and converged iterations.
- [ ] Capture voxel pre-limit and final observation inputs.
- [ ] In `ObsModel`, build capture-only world clouds for accepted sources, rejected sources and accepted IVox neighbor points.
- [ ] Save per-iteration point clouds using `eskf_iteration_stride`.
- [ ] Save final updated world cloud, first-map initialization cloud and MapIncremental insertion clouds.
- [ ] Run ESKF/DataCapture tests and build Lightning.

### Task 4: Keyframe decisions and keyframe artifacts

**Files:**
- Modify: `src/core/lio/laser_mapping.cc`

**Interfaces:**
- Consumes: frame and estimator capture data from Tasks 1–3.
- Produces: `keyframe_decisions.csv`, keyframe body/world clouds and LIO/Opt pose rows.

- [ ] Record distance/angle values, thresholds and decision result at every decision branch.
- [ ] Ensure `capture_all_keyframes` writes keyframe-specific artifacts even when the ordinary frame is outside the stride.
- [ ] Save immutable keyframe body cloud and an explicitly transformed LIO world cloud.
- [ ] Verify non-keyframes do not create keyframe PCD files.

### Task 5: Loop candidate and multiresolution NDT anchors

**Files:**
- Modify: `src/core/loop_closing/loop_closing.h`
- Modify: `src/core/loop_closing/loop_closing.cc`
- Modify: `src/core/system/slam.cc`

**Interfaces:**
- Consumes: `DataCapture*` set before LoopClosing's worker starts.
- Produces: candidate decision rows and per-candidate source/target/multiresolution output artifacts.

- [ ] Inject the LaserMapping-owned recorder into LoopClosing before starting online processing.
- [ ] Record candidate acceptance and rejection reasons without changing candidate selection.
- [ ] Save source body, source world initial and target submap world clouds.
- [ ] At each NDT resolution save output and append convergence, iteration, probability and transform.
- [ ] Record final threshold acceptance and the resulting `Tij`.
- [ ] Build Lightning and verify disabled capture keeps the original loop behavior.

### Task 6: PGO graph and map output anchors

**Files:**
- Modify: `src/core/loop_closing/loop_closing.cc`
- Modify: `src/core/lio/laser_mapping.h`
- Modify: `src/core/lio/laser_mapping.cc`
- Modify: `src/core/system/slam.cc`

**Interfaces:**
- Produces: PGO before/edge/after CSV files and LIO/optimized/selected global PCD files.

- [ ] Snapshot all optimizer vertices before optimization.
- [ ] Record motion, height and loop edges with measurement, information, robust kernel, χ² and final level.
- [ ] Snapshot optimized poses and per-keyframe pose corrections.
- [ ] During SaveMap build both LIOPose and OptPose maps and save both capture copies plus the selected map.
- [ ] Verify the capture map copies are made from independent outputs and never replace the production map.

### Task 7: Documentation and end-to-end verification

**Files:**
- Modify: `README_CN.md`
- Create outside repository: `/home/jarvis/文档/Obsidian Vault/Cache/Daily Notes/20260731 Lightning 单体 SLAM 数据流与 DataCapture 核对指南.md`

**Interfaces:**
- Consumes: final filenames, schemas and configuration from Tasks 1–6.
- Produces: a module-by-module learning guide and manual CloudCompare/PCL inspection checklist.

- [ ] Document coordinate frames and point meaning for every stage.
- [ ] Document recommended learning configuration and targeted full-rate rerun configuration.
- [ ] Document how to correlate frame, ESKF iteration, keyframe, loop and PGO IDs.
- [ ] Run `git diff --check`, DataCapture tests, existing Lightning tests and package build.
- [ ] Perform a configuration-disabled smoke check and, if the bag runtime is available, a short capture-enabled replay check.
- [ ] Inspect generated directory names, CSV headers, PCD readability and point counts.

## Self-review

- Spec coverage: Tasks 2–4 cover every frontend semantic boundary; Tasks 5–6 cover loop, NDT, PGO and map assembly; Task 7 covers learning and manual inspection.
- Placeholder scan: all tasks identify exact files, interfaces, checks and expected behavior; no deferred functional requirement remains.
- Type consistency: all modules use the single `DataCapture` recorder and immutable IDs; the backend never reads mutable frontend path state.
