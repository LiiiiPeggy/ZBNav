# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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

# 4. cmu_planner — planning stack (skip simulation packages if not needed)
cd cmu_planner
colcon build --symlink-install \
  --packages-skip vehicle_simulator velodyne_simulator velodyne_gazebo_plugins velodyne_description
```

Each workspace is an **independent colcon workspace** — source them separately with `source install/setup.bash`.

## Architecture

### Pipeline

```
rslidar_sdk (driver) → rs_converter (format) → SLAM/super_lio (odometry) → cmu_planner (planning)
                                                       ↓
                                              Odin 自带 SLAM → cmu_planner (planning)
```

Two interchangeable SLAM backends, both remap to the same CMU planner topics.

### Workspaces

- **`robosense_ws`** — RoboSense LiDAR driver. CMakeLists auto-detects ROS1 vs ROS2 and adapts. `POINT_TYPE` (`XYZIRT` by default) must match the physical LiDAR model. Low-level packet I/O in `src/rs_driver/`. Three data sources (`config/config.yaml`): `msg_source=1` online LiDAR, `2` ROS packet replay, `3` PCAP file.
- **`rs_converter_ws`** — Republishes RoboSense-format point clouds as Velodyne-format. Ring remapping tables for Ruby (128-line), Bpearl (32-line), and generic 16-line LiDARs. Usage: `ros2 run rs_converter <in_format> <out_format>`.
- **`LI_init_ws`** — LiDAR-IMU extrinsic (6-DOF) + temporal offset calibration via Ceres + ikd-Tree. OpenMP parallelism auto-enabled when CPU cores >3. **Note**: `laserMapping.cpp` embeds the full FAST-LIO2 pipeline — it transitions from calibration to odometry after init.
- **`SLAM`** — Core SLAM system:
  - `basic` — shared library: Eigen type aliases, manifold math (SO3/SE3/S2), ring buffers (must be built first)
  - `super_lio` — ESKF-based LIO with 18-D state (R, p, v, bg, ba, g). OctVoxMap for scan-to-map registration. State machine: `stateWaitKFInit` → `stateWaitMapInit` → `stateProcess`. Nodes: `super_lio_node` (online SLAM), `relocation_node` (global localization against pre-built map)
  - `odin_ros_driver` — Odin 深度传感器 ROS 2 驱动，自带 SLAM 里程计和建图。配置: `config/control_command.yaml` (传感器参数、数据开关、重定位)
- **`cmu_planner`** — Path planning and terrain analysis stack (CMU):
  - `local_planner` — local path planning + path following
  - `terrain_analysis` / `terrain_analysis_ext` — terrain traversability analysis
  - `sensor_scan_generation` — synthetic scan generation for planning
  - `waypoint_example` / `waypoint_rviz_plugin` — waypoint following
  - `vehicle_simulator` / `velodyne_simulator` / `velodyne_gazebo_plugins` — simulation (build with `--packages-skip` to exclude)
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
| `/state_estimation` | `super_lio` (via remap) or `odin_ros_driver` (via remap) | `local_planner`, `terrain_analysis`, `terrain_analysis_ext`, `waypoint_example`, `waypoint_rviz_plugin`, `visualization_tools` |
| `/registered_scan` | `super_lio` (via remap) or `odin_ros_driver` (via remap) | `local_planner`, `terrain_analysis`, `terrain_analysis_ext`, `visualization_tools` |
| `/path` | `local_planner` | `pathFollower` |
| `/terrain_map` | `terrain_analysis` | `local_planner`, `terrain_analysis_ext` |
| `/cmd_vel` | `pathFollower` | robot / `vehicle_simulator` |
| `/way_point` | `waypoint_rviz_plugin`, `waypoint_example` | `local_planner` |
| `/joy` | `joy_node` | `local_planner`, `terrain_analysis`, `terrain_analysis_ext` |

### Key build-time flags

- `rslidar_sdk`: `POINT_TYPE`, `ENABLE_TRANSFORM`, `ENABLE_IMU_DATA_PARSE`, `ENABLE_DIFOP_PARSE` — set in CMakeLists.txt lines 8-59
- `LI_init`: CPU core count determines OpenMP parallelism — defined in CMakeLists.txt lines 17-35
