# ZBNav

Robotics navigation system: **LiDAR driver → SLAM → path planning + terrain analysis**.

## Overview

| Workspace | Packages | Description |
|-----------|----------|-------------|
| `robosense_ws` | `rslidar_sdk`, `rslidar_msg` | RoboSense LiDAR driver (ROS 1 & 2 dual support) |
| `rs_converter_ws` | `rs_converter` | RoboSense → Velodyne point cloud format converter |
| `SLAM` | `basic`, `super_lio`, `odin_ros_driver` | SLAM + Odin depth sensor driver |
| `LI_init_ws` | `lidar_imu_init` | LiDAR-IMU extrinsic calibration + FAST-LIO2 |
| `cmu_planner` | `local_planner`, `terrain_analysis`, ... | Path planning, terrain analysis, simulation |
| `supports` | `Livox-SDK2`, `livox_ros_driver2` | Build dependencies |

### Pipeline

```
rslidar_sdk → rs_converter → super_lio ──remap──▶ /state_estimation, /registered_scan ──▶ cmu_planner
                                                         or
                              odin_ros_driver ──remap──▶ /state_estimation, /registered_scan ──▶ cmu_planner
```

Two SLAM backends (super_lio + external LiDAR, or Odin with built-in SLAM) — both publish to the same standard topics via remap, interchangeable.

## Prerequisites

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **Build tool**: colcon
- **Dependencies**: PCL, Eigen3, Ceres Solver, OpenMP, glog, yaml-cpp, Livox SDK 2

Install system deps:
```bash
sudo apt install -y libpcap-dev ros-humble-pcl-conversions libpcl-dev libeigen3-dev \
  libgoogle-glog-dev libtbb-dev ros-humble-pcl-ros libceres-dev
```

## Build

```bash
# 0. Livox SDK 2 (one-time)
cd supports/Livox-SDK2 && mkdir -p build && cd build && cmake .. && make -j && sudo make install
cd supports/ws_livox/src/livox_ros_driver2 && ./build.sh humble

# 1. LiDAR driver
cd robosense_ws && colcon build --symlink-install

# 2. Format converter
cd rs_converter_ws && colcon build --symlink-install

# 3. SLAM (basic first, then super_lio, then odin)
cd SLAM
colcon build --symlink-install --packages-select basic
colcon build --symlink-install --packages-select super_lio
colcon build --symlink-install --packages-select odin_ros_driver

# 4. Planner (skip simulation packages)
cd cmu_planner
colcon build --symlink-install \
  --packages-skip vehicle_simulator velodyne_simulator velodyne_gazebo_plugins velodyne_description
```

## Run

### Option A: RoboSense LiDAR + super_lio + CMU planner

```bash
# Terminal 1: LiDAR pipeline
source robosense_ws/install/setup.bash && ros2 launch rslidar_sdk start.py
source rs_converter_ws/install/setup.bash && ros2 launch rs_converter rs_converter.launch.py

# Terminal 2: SLAM (topic remap to /state_estimation, /registered_scan)
source SLAM/install/setup.bash && ros2 launch super_lio velodyne.py

# Terminal 3: Planning
source cmu_planner/install/setup.bash && ros2 launch vehicle_simulator system_real_robot.launch
```

### Option B: Odin (built-in SLAM) + CMU planner

```bash
# Terminal 1: Odin driver (topic remap to /state_estimation, /registered_scan)
source SLAM/install/setup.bash && ros2 launch odin_ros_driver odin1_ros2.launch.py

# Terminal 2: Planning
source cmu_planner/install/setup.bash && ros2 launch vehicle_simulator system_real_robot.launch
```

## Data Flow Check

```bash
# Check publishing frequency
ros2 topic hz /state_estimation    # expected ~10 Hz
ros2 topic hz /registered_scan     # expected ~10 Hz
ros2 topic hz /path                # expected ~10 Hz
ros2 topic hz /terrain_map         # expected 1-5 Hz

# Check QoS compatibility
ros2 topic info /state_estimation
ros2 topic info /registered_scan

# Check content
ros2 topic echo /state_estimation --once --field pose
```

## License

- `super_lio`, `basic`: GPLv3
- `lidar_imu_init`: BSD
- `rslidar_sdk`: BSD
- `rs_converter`: Apache 2.0
