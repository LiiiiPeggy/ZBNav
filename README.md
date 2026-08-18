# ZBNav

Robotics navigation system: **LiDAR driver → SLAM → path planning + terrain analysis**.

## Overview

| Workspace | Packages | Description |
|-----------|----------|-------------|
| `robosense_ws` | `rslidar_sdk`, `rslidar_msg` | RoboSense LiDAR driver (ROS 1 & 2 dual support) |
| `rs_converter_ws` | `rs_converter` | RoboSense → Velodyne point cloud format converter |
| `SLAM` | `basic`, `super_lio`, `odin_ros_driver` | SLAM + Odin depth sensor driver |
| `LI_init_ws` | `lidar_imu_init` | LiDAR-IMU extrinsic calibration + FAST-LIO2 |
| `cmu_planner` | `local_planner`, `terrain_analysis`, ... | Path planning, terrain analysis, cruise patrol |
| `supports` | `Livox-SDK2`, `livox_ros_driver2` | Build dependencies |

### Pipeline

```
rslidar_sdk → rs_converter → super_lio ──remap──▶ /state_estimation, /registered_scan ──▶ cmu_planner
                                                         or
                              odin_ros_driver ──remap──▶ /state_estimation, /registered_scan ──▶ cmu_planner
                                                                        │
                                              registered_scan_adapter_node (XYZRGB→XYZI + min-range filter)
                                                                        │
                                                                        ▼
                                                               /registered_scan
```

Two SLAM backends (super_lio + external LiDAR, or Odin with built-in SLAM) — both publish to the same standard topics via remap, interchangeable. When using Odin, `registered_scan_adapter_node` converts its `PointXYZRGB` cloud to the `PointXYZI` format that CMU planner expects and filters close-range points.

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

# 4. Planner
cd cmu_planner
colcon build --symlink-install
```

> **Note**: `vehicle_simulator` builds the `vehicleSimulator` binary only when
> Gazebo is installed (its gazebo deps are `QUIET`/optional). The launch files,
> rviz configs, and `cruiseController` always build. Velodyne simulation packages
> have been removed from this repo.

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

### Odin map save & relocalization

Odin has three algorithm modes via `custom_map_mode` in
`SLAM/src/odin_ros_driver/config/control_command.yaml`:

| Mode | Meaning |
|------|---------|
| `0` | Odometry only (no full mapping) |
| `1` | SLAM: localize + map |
| `2` | Relocalization: locate against a saved map (requires `relocalization_map_abs_path`) |

**Save a map** (SLAM mode `1`): after driving the scene, run
```bash
cd SLAM/src/odin_ros_driver && ./set_param.sh save_map 1
```
The `.bin` map is transferred from the device to
`map/{map_save_time}/map_*.bin` (or `mapping_result_dest_dir` /
`mapping_result_file_name` if set). Note this map is geometric only — it
does **not** contain RGB color; it is for relocalization, not visualization.

**Relocalize** (mode `2`): set `relocalization_map_abs_path` to the saved
`.bin`, then launch via `SLAM/3run_relocalization.sh`. On success the driver
outputs the `odin_map → odin_odom` TF; `/state_estimation` and
`/registered_scan` are in the `odin_odom` planning frame, and the saved map
is published as `/overall_map` (`frame_id=odin_map`).

The driver config is split for the two run modes (each a copy of the active
`control_command.yaml`, differing only in `custom_map_mode`):
- `control_command_slam.yaml` — `custom_map_mode: 1` (mapping), launched by
  `SLAM/2run_slam.sh` (Odin RViz on, no `/overall_map`).
- `control_command_relocalization.yaml` — `custom_map_mode: 2`, launched by
  `SLAM/3run_relocalization.sh` (Odin RViz on, publishes `/overall_map`).

`SLAM/showmap.sh` is a standalone offline viewer of the saved `.pcd` (starts
its own `/overall_map` publisher + `overall_map.rviz`) — do not run it at the
same time as `3run_relocalization.sh` (both would publish `/overall_map`).

### Option C: Odin + one-click scripts

Convenience scripts live in the workspace root (`1.sh`) and `cmu_planner/`:

| Script | Purpose |
|--------|---------|
| `1.sh` | Full build (Docker + all workspaces) |
| `cmu_planner/1build.sh` | Build cmu_planner only |
| `cmu_planner/2run.sh` | Launch main planner stack (no cruise) |
| `cmu_planner/3cruise.sh` | Launch planner **with cruise**, output filtered to `[CRUISE]` lines |
| `cmu_planner/4debug_cruise.sh` | Launch planner with cruise, full output (debug) |
| `cmu_planner/5repeat180.sh` | Launch planner with **repeat cruise**, filtered to `[REPEAT]`/`[CRUISE]`; `./5repeat180.sh [N]` = N round-trips, default infinite (turn_angle=180°) |
| `cmu_planner/6debug_repeat.sh` | Launch planner with repeat cruise, full output (debug) |
| `cmu_planner/7multi.sh` | Launch planner with **MULTI cruise** (`[yaml|rviz] [loop_count] [odin_odom|odin_map]`); `odin_map` auto-opens `cruise_map.rviz` and consumes the SLAM-published `/overall_map` |
| `cmu_planner/8multi_start.sh` | Call `/multi_start` to start a collected RViz route |
| `cmu_planner/9multi_debug.sh` | MULTI cruise, full output |
| `SLAM/2run_slam.sh` | Odin SLAM mapping (mode 1), Odin RViz on, no `/overall_map` |
| `SLAM/3run_relocalization.sh` | Odin relocalization (mode 2), Odin RViz on, publishes `/overall_map` |
| `SLAM/showmap.sh` | Standalone saved-map viewer (own `/overall_map` + `overall_map.rviz`) |

## Cruise Patrol (往返巡航)

`cruiseController` drives a round-trip patrol: go to a waypoint → turn 180° in
place → return to start → turn 180° → stop.

```
/way_point_cruise → cruiseController → /way_point → localPlanner → /path → pathFollower → /cmd_vel
                                   \── /stop (2 during turns) ──/
                                   \── /cmd_vel directly during turns ──/
```

- **Input topic**: `/way_point_cruise` (`geometry_msgs/PointStamped`), or click
  in RViz — the RViz waypoint tool is remapped to `/way_point_cruise` when
  `enableCruise:=true`.
- **Yaw-closed-loop turning**: reads `/state_estimation` yaw, P-controls to
  `current_yaw + π`, clamps at `max_yaw_rate`, done when `|yaw_error| < yaw_tolerance`.
- **Turns**: `cruiseController` publishes `/stop=2` (pathFollower then stops
  publishing `/cmd_vel`) and takes over `/cmd_vel` directly.

Launch:
```bash
cd cmu_planner && ./3cruise.sh          # or ./4debug_cruise.sh for full output
```

Parameters (override on command line):
```bash
ros2 launch local_planner cruise.launch \
  max_yaw_rate:=45.0 yaw_kp:=1.5 yaw_tolerance:=0.12 goal_clear_range:=0.5
```

| Param | Default | Meaning |
|-------|---------|---------|
| `max_yaw_rate` | 45.0 | Max turn rate (deg/s) |
| `yaw_kp` | 1.5 | Yaw P-gain during turns |
| `yaw_tolerance` | 0.12 | Turn completion tolerance (rad, ~7°) |
| `goal_clear_range` | 0.5 | "Reached" distance (m) |

Structured log output (filter with `grep CRUISE`):
```
[CRUISE][INPUT] start=(1.250, 2.430), destination=(8.100, -3.200)
[CRUISE][WAYPOINT] phase=GO_TO_DEST, publish /way_point: x=8.100, y=-3.200
[CRUISE] Destination reached, turning...
[CRUISE] Starting 180-degree turn: TURN_AT_DEST, target_yaw=...
[CRUISE] Turn done, returning to start...
[CRUISE][WAYPOINT] phase=RETURN_TO_START, publish /way_point: x=1.250, y=2.430
[CRUISE] Start reached, turning...
[CRUISE] Cruise complete!
```

## Repeat Cruise (重复巡航)

The same `cruiseController` node, with `repeat_enabled:=true`, patrols
back-and-forth: each round-trip is go → turn 180° → return → turn 180°,
then loop again. `repeat_enabled=false` (default) keeps the single-pass
behavior above — `3cruise.sh` unaffected.

Launch:
```bash
cd cmu_planner && ./5repeat180.sh       # infinite loops
cd cmu_planner && ./5repeat180.sh 3     # exactly 3 round-trips
```

| Param | Default | Meaning |
|-------|---------|---------|
| `repeat_enabled` | false | Enable repeat mode |
| `loop_count` | -1 | Round-trips: `-1` = infinite, `N` = N round-trips, `1` = one; invalid (0 or < -1) clamped to 1 |

**Stop during repeat**: publish `/stop` with `data=2`
(`ros2 topic pub /stop std_msgs/msg/Int8 "{data: 2}" --once`). If mid-turn it
is queued and applied right after the turn; otherwise it aborts immediately.
The node's own `/stop=2` during its turns is ignored via an internal flag.

**Retarget mid-cruise** (repeat mode): clicking RViz while cruising replaces
the destination, re-anchors start at the current pose, resets the loop count,
and restarts from `GO_TO_DEST`.

Repeat log output (filter with `grep REPEAT`):
```
[REPEAT][LOOP] loop 1/3 start: (1.250, 2.430) -> (8.100, -3.200)
[REPEAT] Destination reached, loop 1
[REPEAT] Returning to start, loop 1
[REPEAT][LOOP] loop 1/3 complete
[REPEAT][LOOP] loop 2/3 start: ...
[REPEAT] Cruise complete after 3 loops
```

## Multi-Point Cruise (MULTI 多点巡航)

`cruiseController` in MULTI mode follows a closed-loop waypoint route collected
in RViz or from a YAML route file.

```bash
cd cmu_planner
bash 7multi.sh                    # default: RViz clicks, odin_odom frame (no map needed)
bash 7multi.sh rviz -1 odin_map   # map-mode MULTI: cruise_map.rviz + SLAM /overall_map
bash 7multi.sh yaml -1 odin_map   # YAML route in map frame
```

- **Waypoints**: `MultiWaypointTool` RViz clicks (carrying the current RViz
  Fixed Frame) or `multi_route.yaml` (`multi_source:=yaml|rviz`). Stored in
  `multi_frame` (`odin_odom` no-map mode, `odin_map` map mode).
- **Map-mode MULTI** (`7multi.sh rviz -1 odin_map`): requires the SLAM stack —
  run `SLAM/3run_relocalization.sh` first (relocalized, `/overall_map`
  published). `7multi.sh` opens `cruise_map.rviz` (Fixed Frame `odin_map`,
  `OverallMap` enabled). Two RViz may coexist: the Odin RViz (monitoring
  localization) and `cruise_map.rviz` (marking waypoints). Do NOT run
  `SLAM/showmap.sh` at the same time (both publish `/overall_map`).
- **Start / stop**: `/multi_start` (Trigger) starts the route; `/cruise_autonomy`
  locks planner autonomy during MULTI so `/joy` jitter cannot clear it;
  `/stop` with `data=2` aborts.

Usage: `bash 7multi.sh [yaml|rviz] [loop_count] [odin_odom|odin_map]` (defaults
rviz / -1 / odin_odom; legacy `odom`/`map` accepted and normalized).

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

# Check /registered_scan field format (expect: x, y, z, intensity)
ros2 topic echo /registered_scan --once --field fields

# Cruise topics
ros2 topic info /way_point_cruise -v    # publisher: RViz (cruise mode) / subscriber: cruise_controller
ros2 topic info /way_point -v           # publisher: cruise_controller / subscriber: localPlanner
```

## License

- `super_lio`, `basic`: GPLv3
- `lidar_imu_init`: BSD
- `rslidar_sdk`: BSD
- `rs_converter`: Apache 2.0
