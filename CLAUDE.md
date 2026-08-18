# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

@import .claude/rules/code-edit-markers.md

## Build Commands

### Prerequisites (one-time)

```bash
# Livox-SDK2 — builds liblivox_lidar_sdk_shared.so (required by livox_ros_driver2)
cd supports/Livox-SDK2 && mkdir -p build && cd build && cmake .. && make -j && sudo make install

# livox_ros_driver2 — Livox ROS 2 driver (required by SLAM packages)
cd supports/ws_livox/src/livox_ros_driver2 && ./build.sh humble
source supports/ws_livox/install/setup.bash
```

### Workspaces (build in this order)

```bash
# 1. robosense_ws — LiDAR driver
cd robosense_ws && colcon build --symlink-install

# 2. rs_converter_ws — format converter
cd rs_converter_ws && colcon build --symlink-install

# 3. SLAM — basic BEFORE super_lio, then odin_ros_driver
cd SLAM
colcon build --symlink-install --packages-select basic
colcon build --symlink-install --packages-select super_lio
colcon build --symlink-install --packages-select odin_ros_driver

# 4. cmu_planner — planning stack
cd cmu_planner
colcon build --symlink-install
```

> **Note**: `vehicle_simulator` builds the `vehicleSimulator` binary only when Gazebo
> is installed (its gazebo deps are `QUIET`/optional); launch files, rviz configs and
> `cruiseController` always build. Velodyne simulation packages were removed.
> Convenience scripts: `1build.sh` (build), `2run.sh` (planner only), `3cruise.sh`
> (planner + cruise, `[CRUISE]`-filtered), `4debug_cruise.sh` (planner + cruise, full),
> `5repeat180.sh [N]` (planner + repeat cruise, N round-trips default infinite,
> `[REPEAT]`/`[CRUISE]`-filtered), `6debug_repeat.sh` (planner + repeat cruise, full),
> `7multi.sh [yaml|rviz] [loop_count] [odin_odom|odin_map]` (MULTI cruise; default rviz+odin_odom
> no-map; `odin_map` auto-selects `cruise_map.rviz` and consumes the SLAM-published `/overall_map`;
> `MULTI|CRUISE|WAYPOINT|WARN|ERROR`-filtered), `8multi_start.sh` (call `/multi_start`
> service to start a collected RViz route), `9multi_debug.sh` (MULTI cruise, same filter).
> SLAM run scripts: `2run_slam.sh` (Odin SLAM mapping, mode 1, Odin RViz on, no `/overall_map`),
> `3run_relocalization.sh` (Odin relocalization, mode 2, Odin RViz on, publishes `/overall_map`),
> `showmap.sh` (standalone saved-map viewer — its own `/overall_map` publisher + `overall_map.rviz`;
> do NOT run alongside `3run_relocalization.sh`; they would both publish `/overall_map`).

Each workspace is an **independent colcon workspace** — source them separately with `source install/setup.bash`.

## Architecture

### Pipeline

```
rslidar_sdk (driver) → rs_converter (format) → SLAM/super_lio (odometry) → cmu_planner (planning)
                                                       ↓
                                              Odin 自带 SLAM → cmu_planner (planning)
```

Two interchangeable SLAM backends, both remap to the same CMU planner topics. When using Odin, `registered_scan_adapter_node` (in `odin_ros_driver`) converts its `PointXYZRGB` `/odin1/cloud_slam` into the `PointXYZI` format CMU planner expects (drops rgb, sets `intensity=0`, filters points closer than `scan_min_range` from the vehicle via `/state_estimation`), publishing to `/registered_scan`.

### Workspaces

- **`robosense_ws`** — RoboSense LiDAR driver. CMakeLists auto-detects ROS1 vs ROS2 and adapts. `POINT_TYPE` (`XYZIRT` by default) must match the physical LiDAR model. Low-level packet I/O in `src/rs_driver/`. Three data sources (`config/config.yaml`): `msg_source=1` online LiDAR, `2` ROS packet replay, `3` PCAP file.
- **`rs_converter_ws`** — Republishes RoboSense-format point clouds as Velodyne-format. Ring remapping tables for Ruby (128-line), Bpearl (32-line), and generic 16-line LiDARs. Usage: `ros2 run rs_converter <in_format> <out_format>`.
- **`LI_init_ws`** — LiDAR-IMU extrinsic (6-DOF) + temporal offset calibration via Ceres + ikd-Tree. OpenMP parallelism auto-enabled when CPU cores >3. **Note**: `laserMapping.cpp` embeds the full FAST-LIO2 pipeline — it transitions from calibration to odometry after init.
- **`SLAM`** — Core SLAM system:
  - `basic` — shared library: Eigen type aliases, manifold math (SO3/SE3/S2), ring buffers (must be built first)
  - `super_lio` — ESKF-based LIO with 18-D state (R, p, v, bg, ba, g). OctVoxMap for scan-to-map registration. State machine: `stateWaitKFInit` → `stateWaitMapInit` → `stateProcess`. Nodes: `super_lio_node` (online SLAM), `relocation_node` (global localization against pre-built map)
  - `odin_ros_driver` — Odin 深度传感器 ROS 2 驱动，自带 SLAM 里程计和建图。配置拆分为 `control_command.yaml`（活动配置，mode 2）、`control_command_slam.yaml`（mode 1）、`control_command_relocalization.yaml`（与活动配置逐字节一致，mode 2）。`custom_map_mode`: 0=里程计, 1=SLAM 建图, 2=重定位（需 `relocalization_map_abs_path` 指向 `.bin` 地图）。`.bin` 地图是设备私有格式（`lidar_set_relocalization_map` 直接交给设备解析，SDK 无"取回地图点云"接口）；`./set_param.sh save_map 1` 保存，地图仅几何、无颜色，只用于重定位不用于可视化。**`/overall_map` 所有权在 SLAM**：`3run_relocalization.sh` 启动重定位时 `odin1_ros2.launch.py` 的 `publish_overall_map:=true` + `overall_map_pcd` 会启动 `pcl_ros/pcd_to_pointcloud` 把 `.pcd`（与 `.bin` 同目录，经 `map_downsample` 工具从 PLY 转换）发布为 `/overall_map`（`frame_id=odin_map`，周期 10000 ms）。launch 参数：`enable_rviz`（默认 true）、`publish_overall_map`（默认 false）、`overall_map_pcd`。
- **`cmu_planner`** — Path planning and terrain analysis stack (CMU):
  - `local_planner` — local path planning + path following; also hosts `cruiseController` (patrol: go → turn 180° → return → turn 180°; yaw-closed-loop turning via `/state_estimation`). Repeat mode via `repeat_enabled`/`loop_count` params: loops back-and-forth N round-trips (-1 = infinite), external `/stop=2` aborts (self-published stops ignored via `ignore_next_internal_stop_`, queued stops consumed at both turn completions via `consumePendingStop()`), mid-cruise retarget resets loops. **MULTI mode** (`multi_enabled` + `multi_source` `yaml|rviz` + `multi_frame` `odin_odom|odin_map`, default `odin_odom`): closed-loop waypoint route; waypoints come from YAML or RViz clicks, always stored in `multi_frame_` coords (`odin_odom` mode needs only `/state_estimation`; `odin_map` mode TF-converts each waypoint `odin_map→odin_odom` at leg start). RViz clicks are frame-agnostic — `addWaypointCallback()` transforms any input frame to `multi_frame_`. `/multi_start` (Trigger service) publishes `/cruise_autonomy=true` so local_planner/pathFollower ignore `/joy` during MULTI; released on external `/stop=2` or finite-loop completion
  - `terrain_analysis` / `terrain_analysis_ext` — terrain traversability analysis
  - `sensor_scan_generation` — synthetic scan generation for planning
  - `waypoint_example` / `waypoint_rviz_plugin` — waypoint following; `WaypointTool` (legacy SINGLE/REPEAT: publishes `/way_point` in the current RViz Fixed Frame + a fake `/joy` burst, subscribes `/state_estimation`) and `MultiWaypointTool` (MULTI: PoseTool XY-plane projection — clickable in empty/point-cloud-free areas, publishes `/multi_waypoint_add` in the current RViz Fixed Frame; never publishes `/joy`/`/way_point`)
  - `vehicle_simulator` — simulation + system launch aggregator (`system_real_robot.launch`); gazebo deps optional (`vehicleSimulator` binary skipped if no gazebo)
  - `visualization_tools` / `panda3v2_description` / `loam_interface` — viz and legacy adapters
- **`supports`** — Build dependencies: `Livox-SDK2` (LiDAR SDK library), `ws_livox/livox_ros_driver2` (Livox ROS 2 driver wrapper)

### Topic remap design

SLAM 输出通过 launch 文件 remap 统一对接 CMU 规划栈，**无需 bridge 节点**:

| SLAM 原始 Topic | remap → | CMU 标准 Topic |
|-----------------|---------|----------------|
| super_lio `/lio/odom` | → | `/state_estimation` |
| super_lio `/lio/cloud_world` | → | `/registered_scan` |
| Odin `odin1/odometry` | → | `/state_estimation` |
| Odin `odin1/cloud_slam` | → | `/registered_scan` |

### Frame conventions (real robot, Odin chain)

- Navigation tree: `odin_map` (map/relocalization frame) → `odin_odom` (local planning frame) → `odin1_base_link`, `sensor_at_scan` → `vehicle`/`camera`. The Unitree motion-control tree (`map → base → legs…`) is separate and NEVER linked to the Odin tree.
- `/state_estimation`, `/registered_scan`, `/terrain_map`, `/terrain_map_ext`, `/path_viz` carry `frame_id=odin_odom`; `/overall_map` carries `frame_id=odin_map`. `terrain_analysis`/`_ext` **inherit** `/registered_scan`'s frame.
- RViz configs: `vehicle_simulator.rviz` (no-map mode) Fixed Frame `odin_odom`; `cruise_map.rviz` (map-mode MULTI) Fixed Frame `odin_map` with the `OverallMap` display consuming `/overall_map` (and RegScan/FreePaths disabled by default). `MultiWaypointTool` clicks carry the current RViz Fixed Frame.
- MULTI `odin_odom` mode: clicks arrive `odin_odom` and are stored directly (`multi_frame=odin_odom`, needs only `/state_estimation`). MULTI `odin_map` mode: Fixed Frame `odin_map`, clicks stored as map coords, converted `odin_map→odin_odom` per leg via `lookupTransform("odin_odom","odin_map")`.
- Launch semantics: `planning_frame=odin_odom`, `global_frame=odin_map`, `multi_frame=odin_odom|odin_map`; `cruiseController` transforms map-frame waypoints to the planning frame before publishing `/way_point`.
- `/cruise_autonomy` (Bool) locks planner autonomy during MULTI so PS3 `/joy` jitter cannot clear `autonomyMode`; released on abort/completion.

### Language standards vary

| Package | C++ Standard |
|---------|-------------|
| `basic` | C++20 |
| `super_lio` | C++20 |
| `lidar_imu_init` | C++14 |
| `rslidar_sdk` | C++17 |
| `rs_converter` | C++14 |

### Custom ROS 2 messages

- `rslidar_msg` (in `robosense_ws`) — custom point cloud types for RoboSense driver
- `lidar_imu_init` — `Pose6D.msg`, `States.msg` (calibration output)
- `super_lio` — `CloudPose.msg`, `CloudPose2.msg`

### Key ROS topics

| Topic | Publisher | Subscriber |
|-------|-----------|------------|
| `/rslidar_points` | `rslidar_sdk` | `rs_converter` |
| `/velodyne_points` | `rs_converter` | `super_lio` |
| `/livox/lidar` (CustomMsg) | Livox driver | `super_lio`, `lidar_imu_init` |
| `/state_estimation` | `super_lio` (via remap) or `odin_ros_driver` (via remap) | `local_planner`, `terrain_analysis`, `terrain_analysis_ext`, `waypoint_example`, `waypoint_rviz_plugin`, `visualization_tools`, `cruiseController` |
| `/registered_scan` | `super_lio` (via remap) or `odin_ros_driver` → `registered_scan_adapter_node` | `local_planner`, `terrain_analysis`, `terrain_analysis_ext`, `visualization_tools` |
| `/overall_map` | `SLAM/odin_ros_driver` (relocalization via `3run_relocalization.sh`, `pcl_ros/pcd_to_pointcloud`, frame `odin_map`) or `SLAM/showmap.sh` (standalone) | `cruise_map.rviz` `OverallMap` display; waypoint marking |
| `/path` | `local_planner` | `pathFollower` |
| `/terrain_map` | `terrain_analysis` | `local_planner`, `terrain_analysis_ext` |
| `/cmd_vel` | `pathFollower` (travel) or `cruiseController` (turns) | robot / `vehicle_simulator` |
| `/way_point` | `waypoint_rviz_plugin` (current RViz Fixed Frame), `waypoint_example`, `cruiseController` (frame `planning_frame` = `odin_odom`) | `local_planner` |
| `/way_point_cruise` | RViz waypoint tool (when `enableCruise:=true`) | `cruiseController` |
| `/multi_waypoint_add` (PointStamped) | `MultiWaypointTool` (RViz, current Fixed Frame) | `cruiseController` |
| `/multi_waypoints` (MarkerArray, transient_local) | `cruiseController` | RViz |
| `/multi_start` (Trigger service) | `cruiseController` | `8multi_start.sh` / user |
| `/cruise_autonomy` (Bool) | `cruiseController` (MULTI only) | `local_planner`, `pathFollower` |
| `/stop` | `cruiseController` | `pathFollower` (data 2 = full stop, cruise takes over `/cmd_vel`); `cruiseController` also subscribes in repeat AND multi mode (external stop) |
| `/joy` | `joy_node` | `local_planner`, `terrain_analysis`, `terrain_analysis_ext`, `pathFollower` |

### Key build-time flags

- `rslidar_sdk`: `POINT_TYPE`, `ENABLE_TRANSFORM`, `ENABLE_IMU_DATA_PARSE`, `ENABLE_DIFOP_PARSE` — set in CMakeLists.txt lines 8-59
- `LI_init`: CPU core count determines OpenMP parallelism — defined in CMakeLists.txt lines 17-35

## Branches

`main` → `cruise` (SINGLE/REPEAT patrol, field-verified) → `multi` (MULTI multi-point cruise + MultiWaypointTool + `/cruise_autonomy`, field-verified no-map state) → `mapmulti` (current working branch; map-mode MULTI — `/overall_map` owned by SLAM, `cruise_map.rviz` map-view, `multi_frame=odin_map`; the cmu_planner-publisher approach in its early history is superseded).
