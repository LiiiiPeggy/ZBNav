# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# robosense_ws — LiDAR driver (ROS1 or ROS2, auto-detected)
cd robosense_ws && colcon build --symlink-install

# superlio_ws — MUST build basic BEFORE super_lio
cd superlio_ws
colcon build --symlink-install --packages-select basic
colcon build --symlink-install --packages-select super_lio

# LI_init_ws — LiDAR-IMU calibration
cd LI_init_ws && colcon build --symlink-install

# rs_converter_ws — format converter
cd rs_converter_ws && colcon build --symlink-install

# Build a single package with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select <pkg>
```

Each workspace is an **independent colcon workspace** — source them separately with `source install/setup.bash`. They are not a merged workspace.

## Architecture

### Four-workspace pipeline

```
rslidar_sdk (driver) → rs_converter (optional format conversion) → LI_init (calibration) → super_lio (odometry)
```

- **`robosense_ws`** — RoboSense LiDAR driver. CMakeLists auto-detects ROS1 vs ROS2 and adapts. The `POINT_TYPE` variable (`XYZIRT` by default) must match the physical LiDAR model. Low-level packet I/O lives in `src/rs_driver/` (separate CMake project, brought in via `add_subdirectory`). Three data sources supported (set in `config/config.yaml`): `msg_source=1` for online LiDAR, `2` for ROS packet replay, `3` for PCAP file.
- **`rs_converter_ws`** — Lightweight node that republishes RoboSense-format point clouds as Velodyne-format (`sensor_msgs/PointCloud2`). Contains hardcoded ring remapping tables for Ruby (128-line), Bpearl (32-line), and generic 16-line LiDARs. Usage: `ros2 run rs_converter <in_format> <out_format>` (e.g., `XYZIRT XYZIRT`).
- **`LI_init_ws`** — Estimates LiDAR-IMU extrinsic (6-DOF) and temporal offset via Ceres optimization + ikd-Tree. CPU core count auto-detected at build time — if >3 cores, OpenMP parallel processing is enabled via `-DMP_EN`. **Note**: `laserMapping.cpp` contains the full FAST-LIO2 pipeline embedded inside — it transitions from LI-Init calibration to FAST-LIO2 odometry after initialization completes.
- **`superlio_ws`** — The core odometry system. Two packages:
  - `basic` — shared utility library with Eigen type aliases, manifold math (SO3/SE3/S2), ring buffers. Must be built first.
  - `super_lio` — ESKF-based LIO with 18-D state (R, p, v, bg, ba, g). Uses OctVoxMap (robin_hood hash-based octree voxel grid) for scan-to-map registration. State machine: `stateWaitKFInit` → `stateWaitMapInit` → `stateProcess`. Produces `super_lio_node` (online SLAM) and `relocation_node` (global localization against pre-built map with given initial pose).

### Language standards vary

| Package | C++ Standard |
|---------|-------------|
| `basic` | C++20 |
| `super_lio` | C++20 |
| `lidar_imu_init` | C++14 |
| `rslidar_sdk` | C++17 |
| `rs_converter` | C++14 |

### Custom ROS 2 messages

- `rslidar_msg` (in `robosense_ws`) — custom point cloud types for the RoboSense driver
- `lidar_imu_init` — `Pose6D.msg`, `States.msg` (calibration output)
- `super_lio` — `CloudPose.msg`, `CloudPose2.msg`

### Key ROS topics (inter-package data flow)

| Topic | Publisher | Subscriber |
|-------|-----------|------------|
| `/rslidar_points` | `rslidar_sdk` | `rs_converter`, consumers |
| `/velodyne_points` | `rs_converter` | Downstream SLAM nodes |
| `/livox/lidar` (CustomMsg) | Livox driver | `super_lio`, `lidar_imu_init` |
| `/odom` | `super_lio` | — |
| `/Pose6D`, `/States` | `lidar_imu_init` | — |

### Key build-time flags

- `rslidar_sdk`: `POINT_TYPE`, `ENABLE_TRANSFORM`, `ENABLE_IMU_DATA_PARSE`, `ENABLE_DIFOP_PARSE` — set in CMakeLists.txt lines 8-59
- `LI_init`: CPU core count determines OpenMP parallelism — defined in CMakeLists.txt lines 17-35
