# ZBNav

Robotics navigation system integrating **LiDAR-IMU initialization**, **LiDAR-inertial odometry (LIO)**, and **multi-LiDAR driver support** on ROS 2.

## Overview

This repository contains four ROS 2 workspaces that together form a complete LiDAR-based perception and localization pipeline:

| Workspace | Package | Description |
|-----------|---------|-------------|
| `LI_init_ws` | `lidar_imu_init` | LiDAR-IMU extrinsic calibration and temporal initialization |
| `superlio_ws` | `super_lio` + `basic` | LiDAR-inertial odometry with ESKF and octree voxel maps |
| `robosense_ws` | `rslidar_sdk` + `rslidar_msg` | RoboSense LiDAR driver (ROS 1 & ROS 2 dual support) |
| `rs_converter_ws` | `rs_converter` | RoboSense → Velodyne point cloud format converter |

### Pipeline

```
LiDAR Driver (rslidar_sdk) → Point Cloud
                                    ↓
                              rs_converter (format conversion if needed)
                                    ↓
                              LiDAR-IMU Init (extrinsic + temporal calibration)
                                    ↓
                              Super LIO (odometry + mapping)
```

## Prerequisites

- **OS**: Ubuntu 20.04+ / 22.04
- **ROS 2**: Humble (Galactic/Iron should also work)
- **ROS 1**: Noetic (optional, only for `rslidar_sdk` ROS1 mode)
- **Build tool**: colcon
- **Key dependencies**: PCL ≥ 1.8, Eigen3, Ceres Solver, OpenMP, glog, yaml-cpp, Livox ROS Driver 2

## Build

Each workspace is built independently with `colcon`:

```bash
# RoboSense LiDAR SDK
cd robosense_ws
colcon build --symlink-install
source install/setup.bash

# RoboSense → Velodyne converter
cd rs_converter_ws
colcon build --symlink-install
source install/setup.bash

# Super LIO (build basic first, then super_lio)
cd superlio_ws
colcon build --symlink-install --packages-select basic
colcon build --symlink-install --packages-select super_lio
source install/setup.bash

# LiDAR-IMU Init
cd LI_init_ws
colcon build --symlink-install
source install/setup.bash
```

> **Note**: `super_lio` depends on `basic`. Build `basic` first.

## Run

### 1. Launch LiDAR Driver

```bash
ros2 launch rslidar_sdk start.py    # config at robosense_ws/src/rslidar_sdk/config/config.yaml
```

### 2. LiDAR-IMU Initialization

Choose a launch file matching your LiDAR model:

```bash
ros2 launch lidar_imu_init livox_mid360.launch.py    # Livox Mid-360
ros2 launch lidar_imu_init livox_avia.launch.py      # Livox Avia
ros2 launch lidar_imu_init velodyne.launch.py         # Velodyne
ros2 launch lidar_imu_init ouster.launch.py           # Ouster
ros2 launch lidar_imu_init robosence.launch.py        # RoboSense
```

Config files are at `LI_init_ws/src/LiDAR_IMU_Init_ROS2/config/<model>.yaml`.

### 3. Super LIO

```bash
ros2 launch super_lio <dataset>.launch.py    # e.g., NCLT, M2DGR, NTU, MCD_ATH
```

Configs are at `superlio_ws/src/super_lio/config/<dataset>.yaml`.

## Package Details

### `rslidar_sdk` — RoboSense LiDAR Driver

Dual ROS 1/ROS 2 support. Key CMake options:

- `POINT_TYPE`: XYZI, XYZIRT (default), XYZIF, XYZIRTF
- `ENABLE_TRANSFORM`: coordinate transform support
- `ENABLE_IMU_DATA_PARSE`: parse built-in IMU data
- `ENABLE_DIFOP_PARSE`: DIFOP packet parsing (enabled by default)

### `lidar_imu_init` — LiDAR-IMU Initialization

Estimates extrinsic calibration (6-DOF transform) and temporal offset between LiDAR and IMU. Uses:
- **ikd-Tree** for fast incremental k-d tree operations
- **Ceres** for nonlinear optimization
- **OpenMP** for parallel processing (auto-detects CPU core count)

Output: `/Pose6D` and `/States` custom ROS 2 messages.

### `super_lio` — LiDAR-Inertial Odometry

Real-time tightly-coupled LIO system:
- **ESKF** (Error-State Kalman Filter) for state estimation
- **OctVoxMap** — hierarchical octree-based voxel map for efficient point cloud registration
- **Relocation** node — global localization in a pre-built map

Custom messages: `CloudPose.msg`, `CloudPose2.msg`.

### `basic` — Shared Utilities

Library providing common data structures, timer utilities, and Eigen/PCL wrappers used by `super_lio`.

## License

- `super_lio` and `basic`: GPLv3
- `lidar_imu_init`: BSD
- `rslidar_sdk`: BSD
- `rs_converter`: Apache 2.0
