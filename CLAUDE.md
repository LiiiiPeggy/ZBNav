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

- **`robosense_ws`** — RoboSense LiDAR driver. CMakeLists auto-detects ROS1 vs ROS2 and adapts. The `POINT_TYPE` variable (`XYZIRT` by default) must match the physical LiDAR model. Low-level packet I/O lives in `src/rs_driver/` (separate CMake project, brought in via `add_subdirectory`).
- **`rs_converter_ws`** — Lightweight node that republishes RoboSense-format point clouds as Velodyne-format (`sensor_msgs/PointCloud2`), enabling downstream tools that expect Velodyne frames.
- **`LI_init_ws`** — Estimates LiDAR-IMU extrinsic (6-DOF) and temporal offset. Uses ikd-Tree for fast k-d tree operations and Ceres for optimization. CPU core count auto-detected at build time — if >3 cores, OpenMP-based parallel processing is enabled via `-DMP_EN` and `-DMP_PROC_NUM`.
- **`superlio_ws`** — The core odometry system. Two packages:
  - `basic` — shared utility library (must be built first)
  - `super_lio` — ESKF-based LIO with OctVoxMap (octree voxel grid for scan-to-map registration). Produces `super_lio_node` (online SLAM) and `relocation_node` (global localization against pre-built map).

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

### Key build-time flags

- `rslidar_sdk`: `POINT_TYPE`, `ENABLE_TRANSFORM`, `ENABLE_IMU_DATA_PARSE`, `ENABLE_DIFOP_PARSE` — set in CMakeLists.txt lines 8-59
- `LI_init`: CPU core count determines OpenMP parallelism — defined in CMakeLists.txt lines 17-35
