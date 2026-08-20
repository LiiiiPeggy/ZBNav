# Map-Mode MULTI: One-Command Map Cruise (7multi.sh + cruise_map.rviz) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `bash 7multi.sh rviz -1 odin_map` a single command that starts map-mode MULTI cruise AND opens the map-mode RViz (Fixed Frame preset `odin_map`, `OverallMap` enabled, `MultiWaypointTool` ready) AND publishes the prebuilt map on `/overall_map` — no separate `showmap.sh` in the cruise chain.

**Architecture:** `system_real_robot.launch` gains selectable `rviz_config_file` (default `vehicle_simulator.rviz`) and a conditional `pcl_ros/pcd_to_pointcloud` publisher (`enable_pcd_map` + `pcd_map_file`). `7multi.sh` passes `rviz_config_file=cruise_map.rviz` + `enable_pcd_map=true` + the map PCD path only when `frame=odin_map`; `frame=odin_odom` passes nothing new (byte-identical behavior to today). `cruise_map.rviz` is a copy of `vehicle_simulator.rviz` with Fixed Frame `odin_map`, `OverallMap` enabled, and map-mode comments. `showmap.sh` (existing standalone viewer) is extended to also open the map RViz — still not called by `7multi.sh`.

**Tech Stack:** ROS 2 Humble, `pcl_ros` (`pcd_to_pointcloud`), rviz2, bash, launch Python.

**Spec:** In-chat design agreed with user (bounded task, 9-point rewrite directive). User directives: no SLAM/ changes; one RViz total in map-mode MULTI; `7multi.sh` must NOT call `showmap.sh`; `showmap.sh` = standalone viewer with trap cleanup and NO `sleep 12`; no-map behavior byte-identical.

## Architecture Invariants (must NOT change)

- Unitree TF tree stays independent and untouched: `map → base → legs…`.
- Odin navigation tree: `odin_map → odin_odom → odin1_base_link`, `odin_odom → sensor_at_scan → vehicle|camera`. Never add a `map↔odin_map` link.
- Map-mode RViz: Fixed Frame `odin_map`. Local planning stays `planning_frame=odin_odom`; global waypoints stay `multi_frame=odin_map`; `cruiseController` transforms `odin_map → odin_odom` before publishing the local `/way_point`. `/path` stays `vehicle`, `/path_viz` stays `odin_odom` — do NOT modify either for the map display.
- On-robot, `tf2_echo odin_map odin_odom` must work (Odin relocalization, `custom_map_mode: 2`).

## Context

- `planner/7multi.sh` (committed): launches ONLY `system_real_robot.launch` with `enableCruise:=true multi_enabled:=true repeat_enabled:=false multi_source:=$mode multi_frame:=$frame loop_count:=$loop_count rvizWaypointTopic:=/way_point_cruise`, output filtered by `grep -E 'MULTI|CRUISE|WAYPOINT|WARN|ERROR'`. Frame normalization: `odom→odin_odom`, `map→odin_map`.
- `planner/src/vehicle_simulator/launch/system_real_robot.launch` (committed): hardcodes `rviz_config_file = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz')` and launches `rviz2 -d <that>` behind a `TimerAction(period=8.0)`; already imports `IfCondition` (used by `start_cruise`).
- `planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz` (committed, no-map mode): Fixed Frame `odin_odom`; `OverallMap` PointCloud2 display (Topic `/overall_map`, Color 255;255;255, FlatColor, Style Points, Size(m) 0.01) present but **disabled** (`Enabled: false` at L345, trailing `Value: false` at L366, block header `- Alpha: 0.10000000149011612` at L333); frame marker + operator note at L523-530. Contains `MultiWaypoints`, `Waypoint`, `PathViz`, `TerrainMap`, `MultiWaypointTool`.
- `planner/showmap.sh` (committed `f34756c`, user-verified; originally authored as `7showmap.sh`): `pcd_to_pointcloud` with `-p file_name:="src/odin_ros_driver/map/map_20260807_151455.pcd" -p tf_frame:=odin_map -p publishing_period_ms:=10000 -r cloud_pcd:=/overall_map`, run after `cd "$(dirname "$0")/../SLAM"`.
- Measured: `pcd_to_pointcloud` (volatile publisher) emits the first message on the first 10 s timer tick (~10 s after node start), then every 10 s.
- Map files are git-ignored, machine-local: `SLAM/src/odin_ros_driver/map/map_20260807_151455.pcd` (PointXYZ, 405,532 points) + `.bin` (device relocalization map, `custom_map_mode: 2`).

## Global Constraints

- **planner only.** Never modify any file under `SLAM/` (the map PCD is only READ at runtime).
- **No-map behavior byte-identical:** with `frame=odin_odom` (or no frame arg) `7multi.sh` must pass nothing new to the launch; `system_real_robot.launch` defaults must reproduce today's behavior exactly (`vehicle_simulator.rviz`, no PCD publisher).
- `7multi.sh` must NOT call `showmap.sh` (no chained launch — would double RViz, double `/overall_map` publisher, couple shell lifecycles).
- Fixed names: topic `/overall_map`, frame `odin_map`, display name `OverallMap`, fixed frames `odin_odom` (no-map) / `odin_map` (map), PCD params `tf_frame` + `publishing_period_ms` (field-verified names — do not rename).
- **`vehicle_simulator.rviz` stays completely unchanged.** `cruise_map.rviz` differs from it ONLY in: (1) `Global Options → Fixed Frame: odin_odom → odin_map`, (2) `OverallMap` `Enabled: false → true` + trailing `Value: false → true`, (3) the frame marker comment + operator note reworded for map-mode semantics. Do NOT modify `Target Frame`, any other display, `PathViz`/`TerrainMap`/`MultiWaypointTool`/`MultiWaypoints`, any existing topic, or any other RViz parameter.
- Marker comment rules per `.claude/rules/code-edit-markers.md`: `# ################################` + `# Python: <desc>` / `# Bash: <desc>` / `# YAML: <desc>`; one marker per logical block; no END markers; never remove existing markers. In `cruise_map.rviz` the copied marker's description must be updated (rule 12) since its purpose changes (odin_map instead of odin_odom).
- Commit messages: no `Co-Authored-By` trailer. Do NOT push. Never `git add -A`; stage only the files named in each task.
- Verify staged files with `git diff --cached --name-only` (NOT `git status --porcelain` — the working tree always contains unrelated untracked files, e.g. `.claude/`, `__pycache__/`, `control_command backup.yaml`, the plan doc itself; their presence is expected and not a signal that staging is wrong).

---

### Task 1: Create `cruise_map.rviz` — map-mode cruise RViz config

**Files:**
- Create: `planner/src/vehicle_simulator/rviz/cruise_map.rviz`

**Interfaces:**
- Consumes: the display set and marker conventions of `vehicle_simulator.rviz` (unchanged).
- Produces: the config referenced by Task 2's `rviz_config_file` default-override and used by Task 3's `showmap.sh`; Fixed Frame `odin_map`, `OverallMap` enabled — renders `/overall_map` (published by Task 2/3) and `/multi_waypoints` in map coordinates.

- [ ] **Step 1: Copy the base config**

Run:
```bash
cd /home/yu/Codes_rk
cp planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz planner/src/vehicle_simulator/rviz/cruise_map.rviz
```

- [ ] **Step 2: Preset Fixed Frame to `odin_map` and reword the frame comment block**

In `cruise_map.rviz`, replace the tail block (currently at L523-530):

```yaml
# ################################
# YAML: use odin_odom as shared RViz fixed frame for current Odin navigation chain
# ################################
# Operator note: MULTI waypoint clicks carry the RViz Fixed Frame.
# - No-map MULTI: RViz Fixed Frame stays odin_odom (the default below).
# - Map-mode MULTI: manually set Global Options -> Fixed Frame to odin_map
#   after launch. Automatic RViz-config switching is out of scope.
    Fixed Frame: odin_odom
```

with:

```yaml
# ################################
# YAML: use odin_map as shared RViz fixed frame for map-mode cruise
# ################################
# Operator note: MULTI waypoint clicks carry the RViz Fixed Frame.
# - This config presets Fixed Frame = odin_map (map-mode MULTI).
# - Requires Odin relocalization (custom_map_mode: 2) so the odin_map ->
#   odin_odom TF is active; /overall_map is published automatically when
#   map mode is started via 7multi.sh.
    Fixed Frame: odin_map
```

- [ ] **Step 3: Enable the `OverallMap` display**

In `cruise_map.rviz`, change the display's top-level `Enabled` (L345 area):

```yaml
      Enabled: false
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: OverallMap
```

→ same block with `Enabled: true`. Then change the block's trailing value (L366 area):

```yaml
        Value: /overall_map
      Use Fixed Frame: true
      Use rainbow: true
      Value: false
```

→ same block ending in `Value: true`.

Constraints on the display (keep as copied — do not change): Topic `/overall_map`, `Class: rviz_default_plugins/PointCloud2`, `Color Transformer: FlatColor`, `Style: Points`, frame comes from the message (`odin_map`).

- [ ] **Step 4: Validate the diff against the base config**

Run:
```bash
cd /home/yu/Codes_rk
diff planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz planner/src/vehicle_simulator/rviz/cruise_map.rviz
```

Expected: ONLY the tail frame-comment block (Step 2) and the two `OverallMap` lines `Enabled: false → true` and `Value: false → true` (Step 3). Any other difference is a defect — fix it.

- [ ] **Step 5: Validate the YAML parses with the expected values**

Run:
```bash
python3 -c "
import yaml
d = yaml.safe_load(open('/home/yu/Codes_rk/planner/src/vehicle_simulator/rviz/cruise_map.rviz'))
assert d['Visualization Manager']['Global Options']['Fixed Frame'] == 'odin_map'
disp = d['Visualization Manager']['Displays']
m = [x for x in disp if x.get('Name') == 'OverallMap'][0]
assert m['Enabled'] is True, m['Enabled']
print('OK: Fixed Frame =', d['Visualization Manager']['Global Options']['Fixed Frame'], '| OverallMap enabled:', m['Enabled'])
"
```

Expected: `OK: Fixed Frame = odin_map | OverallMap enabled: True`

- [ ] **Step 6: Commit only this file**

```bash
cd /home/yu/Codes_rk
git add planner/src/vehicle_simulator/rviz/cruise_map.rviz
git diff --cached --name-only     # must print exactly: planner/src/vehicle_simulator/rviz/cruise_map.rviz
git commit -m "feat(rviz): add cruise_map.rviz for map-mode waypoint marking"
```

### Task 2: `system_real_robot.launch` — selectable RViz config + conditional PCD map publisher

**Files:**
- Modify: `planner/src/vehicle_simulator/launch/system_real_robot.launch`

**Interfaces:**
- Consumes: `cruise_map.rviz` path (passed by Task 3's `7multi.sh`); map PCD path (passed by Task 3, absolute).
- Produces (new launch arguments):
  - `rviz_config_file` (string, default `get_package_share_directory('vehicle_simulator')/rviz/vehicle_simulator.rviz`) — RViz config path.
  - `enable_pcd_map` (bool, default `false`) — start the PCD map publisher.
  - `pcd_map_file` (string, default `''`) — PCD file path, used when `enable_pcd_map=true`.
- Publishes (when enabled): `/overall_map` (sensor_msgs/PointCloud2, frame `odin_map`, first message ~10 s after node start, then every 10 s).

- [ ] **Step 1: Add the new launch configuration bindings**

In `system_real_robot.launch`, near the existing `planning_frame`/`global_frame` `LaunchConfiguration` bindings (L35-36 area), ADD — do not replace anything there:

```python
  # ################################
  # Python: bind selectable RViz config and PCD map publisher launch args
  # ################################
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  enable_pcd_map = LaunchConfiguration('enable_pcd_map')
  pcd_map_file = LaunchConfiguration('pcd_map_file')
```

IMPORTANT — position: the hardcoded `rviz_config_file = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz')` sits directly BEFORE the `start_rviz` Node (~L141), NOT in this binding area. Do not touch it in this step; it is deleted in Step 3. After all edits the file must contain exactly ONE `rviz_config_file` assignment (the `LaunchConfiguration` from this step) plus its `DeclareLaunchArgument` default (Step 2).

(`os.path` and `get_package_share_directory` imports stay — used in Step 2's `DeclareLaunchArgument` default.)

- [ ] **Step 2: Declare the new launch arguments**

After the existing `declare_planning_frame`/`declare_global_frame` declarations (L60-62), add:

```python
  # ################################
  # Python: declare selectable RViz config and PCD map publisher args
  # ################################
  declare_rviz_config_file = DeclareLaunchArgument(
    'rviz_config_file',
    default_value=os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz'),
    description='Path to RViz config file (default: vehicle_simulator.rviz)')
  declare_enable_pcd_map = DeclareLaunchArgument(
    'enable_pcd_map', default_value='false', description='Publish prebuilt map PCD on /overall_map (map-mode MULTI)')
  declare_pcd_map_file = DeclareLaunchArgument(
    'pcd_map_file', default_value='', description='Absolute path to prebuilt map PCD (used when enable_pcd_map=true)')
```

- [ ] **Step 3: Delete the hardcoded `rviz_config_file` assignment before `start_rviz`**

The hardcoded assignment sits immediately before the `start_rviz` Node (~L141). Delete exactly that one line; leave the `start_rviz` Node block untouched:

```python
  rviz_config_file = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
```

→

```python
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
```

(`rviz_config_file` now resolves the `LaunchConfiguration` from Step 1; the remappings and `TimerAction(period=8.0)` stay as-is. After this step, grep for the hardcoded assignment:
```bash
grep -n 'rviz_config_file = os.path.join' planner/src/vehicle_simulator/launch/system_real_robot.launch
```
must return NOTHING — the only `rviz_config_file = ...` left in the file is the `LaunchConfiguration` binding from Step 1.)

- [ ] **Step 4: Add the conditional PCD map publisher node**

After the `start_rviz` block (before `delayed_start_rviz`), add:

```python
  # ################################
  # Python: publish prebuilt map PCD on /overall_map when enabled
  # ################################
  start_pcd_map = Node(
    package='pcl_ros',
    executable='pcd_to_pointcloud',
    name='pcd_map_publisher',
    output='screen',
    condition=IfCondition(enable_pcd_map),
    parameters=[{
      'file_name': pcd_map_file,
      'tf_frame': 'odin_map',
      'publishing_period_ms': 10000,
    }],
    remappings=[
      ('cloud_pcd', '/overall_map'),
    ]
  )
```

Field-verified parameter names only: `file_name`, `tf_frame`, `publishing_period_ms` (from the working `showmap.sh`) — do not rename.

- [ ] **Step 5: Add the new actions to the LaunchDescription**

In the `ld.add_action(...)` section (near `ld.add_action(declare_planning_frame)` and `ld.add_action(delayed_start_rviz)`), add:

```python
  ld.add_action(declare_rviz_config_file)
  ld.add_action(declare_enable_pcd_map)
  ld.add_action(declare_pcd_map_file)
  ld.add_action(start_pcd_map)
```

- [ ] **Step 6: Verify no-map/default behavior is unchanged**

Run:
```bash
cd /home/yu/Codes_rk/planner
source install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch > /tmp/launch_default.log 2>&1 &
sleep 12
ros2 node list | grep -c pcd_map_publisher     # expect: 0 (grep returns nothing, count 0)
pgrep -af "rviz2 -d" | grep -o "vehicle_simulator.rviz"   # expect: the default config path
ros2 topic list | grep -c overall_map          # expect: 0
pkill -f "system_real_robot.launch"; pkill -f rviz2
```
Expected: no `pcd_map_publisher` node, RViz uses `vehicle_simulator.rviz`, no `/overall_map` topic — default behavior byte-identical to today.

- [ ] **Step 7: Verify map-mode wiring (local, no robot)**

Run:
```bash
cd /home/yu/Codes_rk/planner
ROOT="$(cd .. && pwd)"
ros2 launch vehicle_simulator system_real_robot.launch \
  enableCruise:=true multi_enabled:=true multi_source:=rviz multi_frame:=odin_map \
  rvizWaypointTopic:=/way_point_cruise \
  rviz_config_file:="$ROOT/planner/src/vehicle_simulator/rviz/cruise_map.rviz" \
  enable_pcd_map:=true \
  pcd_map_file:="$ROOT/SLAM/src/odin_ros_driver/map/map_20260807_151455.pcd" \
  > /tmp/launch_map.log 2>&1 &
sleep 15
ros2 topic echo /overall_map --once --field header.frame_id   # expect: odin_map
ros2 topic echo /overall_map --once --field width             # expect: 405532 (current map's check value)
pgrep -af "rviz2 -d" | grep -o "cruise_map.rviz"               # expect: cruise_map.rviz
pkill -f "system_real_robot.launch"; pkill -f rviz2; pkill -f pcd_to_pointcloud
```

- [ ] **Step 8: Commit only this file**

```bash
cd /home/yu/Codes_rk
git add planner/src/vehicle_simulator/launch/system_real_robot.launch
git diff --cached --name-only     # must print exactly: planner/src/vehicle_simulator/launch/system_real_robot.launch
git commit -m "feat(launch): selectable RViz config and conditional PCD map publisher"
```

### Task 3: `7multi.sh` frame-dependent behavior + extend `showmap.sh` (standalone viewer)

**Files:**
- Modify: `planner/7multi.sh`
- Modify: `planner/showmap.sh` (already exists and is committed — the verified map publisher; extend it, do NOT rename anything, do NOT assume `7showmap.sh` exists)

**Interfaces:**
- Consumes: `rviz_config_file` / `enable_pcd_map` / `pcd_map_file` args (Task 2); `cruise_map.rviz` (Task 1); map PCD at `SLAM/src/odin_ros_driver/map/map_20260807_151455.pcd` (resolved absolute from repo root).
- Produces (`frame=odin_map` only): the same system launch as today PLUS `rviz_config_file=cruise_map.rviz`, `enable_pcd_map=true`, `pcd_map_file=<abs path>` → single RViz (Fixed Frame `odin_map`, `OverallMap` enabled) + `/overall_map` published automatically.
- `showmap.sh` (standalone viewer, NOT called by `7multi.sh`): publishes `/overall_map` (frame `odin_map`) AND opens `cruise_map.rviz` in the same command; kills its own background publisher on exit.

#### Part A: `7multi.sh` — frame-dependent launch args

- [ ] **Step 1: Read the current script** — confirm the structure: normalization block (`odom→odin_odom`, `map→odin_map`) then a single `ros2 launch vehicle_simulator system_real_robot.launch ...` invocation with the `grep` filter. The `mode`/`loop_count`/`frame` semantics stay unchanged.

- [ ] **Step 2: Add the frame-dependent map args before the launch invocation**

Insert between the frame normalization block and the `ros2 launch` line:

```bash
# ################################
# Bash: enable map-mode RViz config and prebuilt map publisher
# ################################
map_args=()
if [[ "$frame" == "odin_map" ]]; then
  ROOT="$(cd "$(dirname "$0")/.." && pwd)"
  map_args+=(
    rviz_config_file:="$ROOT/planner/src/vehicle_simulator/rviz/cruise_map.rviz"
    enable_pcd_map:=true
    pcd_map_file:="$ROOT/SLAM/src/odin_ros_driver/map/map_20260807_151455.pcd"
  )
fi
```

- [ ] **Step 3: Pass the args to the launch invocation**

In the existing `ros2 launch vehicle_simulator system_real_robot.launch \` command, add `"${map_args[@]}"` on its own line right before the `2>&1 | grep` filter:

```bash
  rvizWaypointTopic:=/way_point_cruise \
  "${map_args[@]}" \
  2>&1 | grep --line-buffered -E 'MULTI|CRUISE|WAYPOINT|WARN|ERROR'
```

Behavior contract (verify by reading the diff): `frame=odin_odom` → `map_args` empty → the launch command is byte-identical to today; `frame=odin_map` → map RViz config + `/overall_map` publisher.

- [ ] **Step 4: Update the usage text**

Inside the existing usage block (keep the existing marker; add the map-mode lines), change the two map-mode example lines:

```
  echo "  bash 7multi.sh yaml -1 odin_map # YAML route in prebuilt map frame (needs Odin relocalization)"
  echo "  bash 7multi.sh rviz -1 odin_map # RViz clicks in odin_map frame (needs Odin relocalization)"
```

to:

```
  echo "  bash 7multi.sh yaml -1 odin_map # YAML route in prebuilt map frame (needs Odin relocalization)"
  echo "  bash 7multi.sh rviz -1 odin_map # Map-mode MULTI: opens cruise_map.rviz (Fixed Frame=odin_map,"
  echo "                                  # OverallMap enabled) and auto-publishes /overall_map from the"
  echo "                                  # prebuilt map PCD. Needs Odin relocalization."
```

- [ ] **Step 5: Syntax check**

Run: `bash -n /home/yu/Codes_rk/planner/7multi.sh && echo OK` — Expected: `OK`

#### Part B: extend `showmap.sh` to also open the map-mode RViz (standalone viewer)

`planner/showmap.sh` already exists and is committed; its verified publish block must stay byte-identical (same `cd "$(dirname "$0")/../SLAM"` pattern, same `file_name` relative path, same params, same remap). The only additions: source the workspace setup, background the publisher with a cleanup trap, and launch RViz immediately.

- [ ] **Step 6: Read the current file and confirm the verified block**

Run: `cat /home/yu/Codes_rk/planner/showmap.sh`

Expected — the verified block (must stay unchanged):

```bash
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# ################################
# Bash: show saved odin map point cloud on /overall_map
# ################################
# pcd_to_pointcloud resolves file_name relative to cwd — cd into SLAM workspace
cd "$(dirname "$0")/../SLAM"

ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="src/odin_ros_driver/map/map_20260807_151455.pcd" \
  -p tf_frame:=odin_map \
  -p publishing_period_ms:=10000 \
  -r cloud_pcd:=/overall_map
```

- [ ] **Step 7: Rewrite `showmap.sh` with the minimal additions**

Write the full content of `planner/showmap.sh` (the verified publish block stays as-is; the script resolves an absolute `SCRIPT_DIR` BEFORE any `cd`, because the later `cd "$SCRIPT_DIR/../SLAM"` would invalidate relative `$(dirname "$0")` lookups):

```bash
#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ################################
# Bash: show saved odin map point cloud on /overall_map
# ################################
# pcd_to_pointcloud resolves file_name relative to cwd — cd into SLAM workspace
cd "$SCRIPT_DIR/../SLAM"

ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="src/odin_ros_driver/map/map_20260807_151455.pcd" \
  -p tf_frame:=odin_map \
  -p publishing_period_ms:=10000 \
  -r cloud_pcd:=/overall_map &

MAP_PID=$!

# ################################
# Bash: clean up map publisher when standalone viewer exits
# ################################
cleanup() {
  kill "$MAP_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Launch RViz immediately: the publisher is a periodic volatile publisher,
# so subscribing now catches the first ~10 s tick (no sleep — waiting
# would miss it and delay the first render by a full period).
ros2 run rviz2 rviz2 -d \
  "$SCRIPT_DIR/src/vehicle_simulator/rviz/cruise_map.rviz"
```

Constraints: (A) `source "$SCRIPT_DIR/install/setup.bash"` after the humble source — for the workspace's own RViz plugin; (B) the pcd command is backgrounded (`&`) with `MAP_PID=$!`; (C) `trap cleanup EXIT INT TERM` kills the publisher when the viewer exits — no stray `/overall_map` publisher left for the next run; (D) NO `sleep 12`; (E) RViz opens with `cruise_map.rviz` — no second terminal needed. Do NOT change the `cd` target or the map relative path to repo-root style.

- [ ] **Step 8: Syntax check + smoke test**

Run:
```bash
bash -n /home/yu/Codes_rk/planner/showmap.sh && echo OK
cd /home/yu/Codes_rk
bash planner/showmap.sh > /tmp/showmap.log 2>&1 &
sleep 4
source /opt/ros/humble/setup.bash
pgrep -af "rviz2 -d .*cruise_map" | head -2     # expect: rviz2 with cruise_map.rviz (starts immediately)
sleep 8
ros2 topic echo /overall_map --once --field header.frame_id   # expect: odin_map (~10 s after start)
ros2 topic echo /overall_map --once --field width             # expect: 405532 (current map check value)
pgrep -af pcd_to_pointcloud | wc -l              # expect: 1 (only this viewer's publisher)
pkill -f rviz2
sleep 2
pgrep -af pcd_to_pointcloud | wc -l              # expect: 0 (trap cleaned up the publisher)
```

- [ ] **Step 9: Commit only these two files**

```bash
cd /home/yu/Codes_rk
git add planner/7multi.sh planner/showmap.sh
git diff --cached --name-only     # must print exactly: planner/7multi.sh and planner/showmap.sh
git commit -m "feat(cmu): 7multi.sh map-mode RViz selection; showmap.sh auto-opens map RViz"
```

### Task 4: End-to-end verification

**Files:** none — verification only (no commit).

**Focus check:** `bash 7multi.sh rviz -1 odin_map` starts EXACTLY ONE RViz, and that RViz has both the map display and `MultiWaypointTool` clickability.

**Interfaces:**
- Consumes: Tasks 1-3; the map-mode MULTI chain (`cruiseController` `multi_frame=odin_map`, `MultiWaypointTool`, Odin relocalization on-robot).

- [ ] **Step 1: Local run — `bash 7multi.sh rviz -1 odin_map` (no robot)**

Run:
```bash
cd /home/yu/Codes_rk/planner
bash 7multi.sh rviz -1 odin_map > /tmp/multi_map.log 2>&1 &
sleep 20
source /opt/ros/humble/setup.bash
```

- [ ] **Step 2: Exactly one RViz, with the map config**

```bash
pgrep -c -f "rviz2 -d"                    # expect: 1
pgrep -af "rviz2 -d" | grep -o "cruise_map.rviz"   # expect: cruise_map.rviz
```
Expected: a single RViz process running `cruise_map.rviz` (Fixed Frame `odin_map` preselected, `OverallMap` enabled — confirm visually; `MultiWaypointTool` is present in the Tools section).

- [ ] **Step 3: Topics (local-safe checks only)**

```bash
ros2 topic echo /overall_map --once --field header.frame_id    # expect: odin_map
ros2 topic echo /overall_map --once --field width              # expect: 405532 (current map check value; if the downsampled PCD changes later, this value changes with it — not a fixed interface contract)
ros2 topic hz /overall_map --w 5                                # expect: ~0.1 Hz (10 s period)
pgrep -af pcd_to_pointcloud | wc -l                             # expect: 1 (single /overall_map publisher)
```
Expected: `/overall_map` in `odin_map`, single publisher. NOTE — do NOT run `ros2 topic echo /path_viz --once` or `/terrain_map --once` locally: without `/state_estimation` their publishers may never emit, so `--once` blocks forever. Those frame checks belong to the on-robot acceptance (Step 6): `/path_viz → odin_odom`, `/terrain_map → odin_odom` (no `/path` control-semantics change).

- [ ] **Step 4: Waypoint click frame check (local)**

In the RViz from Step 1: click on the map with `MultiWaypointTool` (Fixed Frame is already `odin_map`).

```bash
ros2 topic echo /multi_waypoint_add --once --field header.frame_id   # expect: odin_map
```
Expected: the click carries `frame_id: odin_map` (waypoints stored as `multi_frame=odin_map`). NOTE: locally (no robot, no `odin_map→odin_odom` TF) `cruiseController` will reject the waypoint with the `[MULTI] Waiting for odin_map -> odin_odom relocalization TF...` gate — expected; consumption is verified on-robot in Step 6.

- [ ] **Step 5: Standalone viewer acceptance (local, no robot)**

```bash
cd /home/yu/Codes_rk
bash planner/showmap.sh > /tmp/showmap_accept.log 2>&1 &
sleep 4
source /opt/ros/humble/setup.bash
pgrep -af "rviz2 -d .*cruise_map" | head -2    # expect: rviz2 with cruise_map.rviz opens automatically (no second terminal)
sleep 8
ros2 topic echo /overall_map --once --field header.frame_id   # expect: odin_map
pgrep -af pcd_to_pointcloud | wc -l            # expect: 1
pkill -f rviz2
sleep 2
pgrep -af pcd_to_pointcloud | wc -l            # expect: 0 (publisher exits with the viewer)
```
Expected: one command starts the `/overall_map` publisher AND opens `cruise_map.rviz`; closing RViz stops the publisher — no separate RViz terminal needed, no leftover publisher.

- [ ] **Step 6: On-robot manual acceptance checklist** (robot site; not executable on this machine — report as pending)

Formal map-mode cruise runs ONLY:
```bash
cd planner
bash 7multi.sh rviz -1 odin_map
```
(no separate `showmap.sh`)

1. Robot running with `custom_map_mode: 2` (relocalization against `map_20260807_151455.bin`): `SLAM/2run.sh` (driver) then the `7multi.sh` command above — nothing else.
2. `ros2 run tf2_ros tf2_echo odin_map odin_odom` → transforms flowing (relocalization converged).
3. EXACTLY ONE RViz window (from `7multi.sh`): Fixed Frame `odin_map` preselected, `OverallMap` checked and showing the map, live terrain/`/path_viz` transformed onto it.
4. Frame checks: `ros2 topic echo /path_viz --once --field header.frame_id` → `odin_odom`; `ros2 topic echo /terrain_map --once --field header.frame_id` → `odin_odom` (both publish once `/state_estimation` flows).
5. Click waypoints with `MultiWaypointTool` on the map → markers appear on `/multi_waypoints` (frame `odin_map`); `/multi_start` starts the map-frame cruise; cruise follows the waypoints.
6. No-map mode regression: `bash 7multi.sh rviz` (default odin_odom) still uses `vehicle_simulator.rviz`, no `/overall_map` publisher — behavior unchanged.

- [ ] **Step 7: Report**

Write the report file with: per-step results (pass/fail/pending), the exact outputs of Step 2-3 commands, the Task 1 Step 4 `diff` output, and Step 6 marked pending-on-robot.
