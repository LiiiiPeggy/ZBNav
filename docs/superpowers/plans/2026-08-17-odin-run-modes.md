# SLAM-Side Map Ownership: Odin Run Modes + /overall_map — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move map-file / relocalization / `/overall_map` ownership into `SLAM/odin_ros_driver` (relocalization mode publishes it), and reduce `cmu_planner` to a pure consumer of `/overall_map` (keeps `cruise_map.rviz` + map-mode MULTI, publishes nothing, knows no PCD path). The standalone saved-map viewer `showmap.sh` moves from `cmu_planner/` to `SLAM/` and opens SLAM's own `overall_map.rviz`.

**Architecture:** Two new SLAM entry scripts + two split configs. `2run_slam.sh` launches Odin SLAM mapping (`custom_map_mode: 1`, Odin RViz on, no `/overall_map`). `3run_relocalization.sh` launches Odin relocalization (`custom_map_mode: 2`, Odin RViz on, `publish_overall_map:=true` → the Odin launch starts a `pcl_ros/pcd_to_pointcloud` node publishing `/overall_map` with `frame_id=odin_map`). `odin1_ros2.launch.py` gains `enable_rviz` / `publish_overall_map` / `overall_map_pcd` args. `cmu_planner` removes its `pcd_to_pointcloud` publisher (launch + 7multi.sh, and deletes its `showmap.sh`) while keeping the `rviz_config_file` selection and `cruise_map.rviz`. The standalone saved-map viewer moves to `SLAM/showmap.sh` (publishes `/overall_map` itself + opens SLAM's own `SLAM/src/odin_ros_driver/config/overall_map.rviz`); it never references `cmu_planner` — no `SLAM → cmu_planner` reverse dependency.

**Tech Stack:** ROS 2 Humble, launch Python, bash, `pcl_ros` (`pcd_to_pointcloud`, launch-runtime only), rviz2.

**Spec:** In-chat design directive (19 sections) from the user. **Supersedes** `docs/superpowers/plans/2026-08-17-map-cloud-rviz.md` (which owned the now-abandoned cmu_planner-publisher approach). No separate spec file; rulings provisional against this plan text.

## Context (current real state — verified 2026-08-17)

- Four commits landed from the previous plan and are KEPT in history (no revert/reset):
  - `c2754d4` cruise_map.rviz · `b30227e` launch selectable RViz + pcd publisher · `86e572b` 7multi.sh map-mode + showmap.sh auto-RViz · `d933ddd` showmap.sh cleanup fix.
- Kept as-is: `cmu_planner/src/vehicle_simulator/rviz/cruise_map.rviz` (Fixed Frame `odin_map`, `OverallMap` enabled, topic `/overall_map`); `system_real_robot.launch` `rviz_config_file = LaunchConfiguration('rviz_config_file')` (default `vehicle_simulator.rviz`); `7multi.sh` `frame=odin_map → cruise_map.rviz` selection.
- `SLAM/src/odin_ros_driver/config/control_command.yaml` is the active verified config: `custom_map_mode: 2`, `relocalization_map_abs_path: "/root/work/lqp/SLAM/src/odin_ros_driver/map/map_20260807_151455.bin"`, plus verified sensor fields (`use_host_ros_time: 2`, `sendodom: 1`, `sendcloudslam: 1`, `dtof_fps: 100`, `custom_init_pos`, `custom_init_pose_search_radius: 4.0`, `custom_init_pose_max_rot_deg: 180.0`, …). Full `register_keys` block spans L1-197.
- `SLAM/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py` (current): args `config_file` (default share `config/control_command.yaml`), `rviz_config` (default share `config/odin_ros2.rviz`); nodes `host_sdk_sample`, `registered_scan_adapter_node`, `pcd2depth_ros2_node`, `cloud_reprojection_ros2_node`, `image_overlay_node`, `rviz_node` (UNCONDITIONAL, L98-104); no `launch.conditions` import. Note: `pcd2depth`/`reprojection`/`overlay` read `os.path.join(package_dir, 'config', 'control_command.yaml')` (the installed share copy) directly, NOT the `config_file` arg — out of scope to parameterize, harmless because the two new configs keep sensor fields identical.
- `SLAM/src/odin_ros_driver/package.xml`: no `pcl_ros` anywhere. `CMakeLists.txt` already `install(DIRECTORY config/)` in both ROS1 (L246) and ROS2 (L460) sections — new config files in `config/` are installed automatically; no CMake change needed.
- `SLAM/2run.sh` (unchanged, out of scope): `source install/setup.bash; ros2 launch odin_ros_driver odin1_ros2.launch.py` → after Task 4, behaves as relocalization-without-map-publish (`publish_overall_map` default false).
- Map PCD at `SLAM/src/odin_ros_driver/map/map_20260807_151455.pcd` (PointXYZ, 405,532 points) — the host-side copy of the relocalization `.bin`. **`pcd_to_pointcloud` first message ~10 s after node start (10 s timer), then every 10 s.**
- `cmu_planner/showmap.sh` (committed `f34756c`, then `86e572b`/`d933ddd`) MOVES to `SLAM/showmap.sh` with a new SLAM RViz config (`SLAM/src/odin_ros_driver/config/overall_map.rviz`). The d933ddd-verified cleanup approach (pkill the real `pcd_to_pointcloud` node, not just the `ros2 run` launcher) is carried over.

## Global Constraints

### Architecture / ownership
- **`/overall_map` ownership = SLAM/odin_ros_driver, relocalization mode only.** cmu_planner CONSUMES only; it must not read the PCD, must not start `pcd_to_pointcloud`, and **`SLAM/src/odin_ros_driver/map/map_20260807_151455.pcd` must not appear anywhere in `cmu_planner/`** (final check).
- **No `SLAM → cmu_planner` reverse dependency.** `SLAM/showmap.sh` must NOT open `cmu_planner`'s `cruise_map.rviz`; it opens SLAM's own `SLAM/src/odin_ros_driver/config/overall_map.rviz`. `cruise_map.rviz` belongs to the formal MULTI flow only (`bash cmu_planner/7multi.sh rviz -1 odin_map`).
- **Standalone viewer vs relocalization are exclusive.** `SLAM/showmap.sh` (offline saved-map view; starts its own `pcd_to_pointcloud`) and `SLAM/3run_relocalization.sh` (formal relocalization; the Odin launch publishes `/overall_map`) must NOT run simultaneously — both would publish `/overall_map` (two publishers). Usage text states this.
- **`cmu_planner/` must contain NO `showmap.sh`** and no map-asset term (`map_20260807_151455.pcd`, `pcd_to_pointcloud`, `enable_pcd_map`, `pcd_map_file`, `SLAM/src/odin_ros_driver/map`, `cloud_pcd`).
- `custom_map_mode` values: SLAM config `1`, relocalization config `2`. Both configs are copies of the current active `control_command.yaml`; besides the mode field, keep all verified sensor params consistent. Do NOT change the map (`relocalization_map_abs_path` stays `/root/work/lqp/SLAM/.../map_20260807_151455.bin`).
- `/overall_map`: `frame_id=odin_map`, `publishing_period_ms=10000`, `width=405532` is the CURRENT map's check value (not a fixed interface contract).
- **Formal map-mode flow allows TWO RViz:** (1) Odin's own RViz (from `3run_relocalization.sh`, monitors relocalization/cloud/pose) and (2) `cruise_map.rviz` (from `7multi.sh`, waypoint marking/cruise). The old "one RViz only" constraint is dropped.
- TF/frame invariants (NO changes): Unitree tree `map → base → legs…` stays independent and untouched (never add `map→odin_map`). Odin tree `odin_map → odin_odom → odin1_base_link`, `odin_odom → sensor_at_scan → vehicle|camera`. `planning_frame=odin_odom`, `global_frame=odin_map`, MULTI `multi_frame=odin_map`, CruiseController transforms `odin_map→odin_odom` then publishes local `/way_point`. `/path=vehicle`, `/path_viz=odin_odom`, `/terrain_map=odin_odom` unchanged.

### Git / workspace
- The four commits above are kept; NO `git reset`, `git revert`, `git checkout --`, `git restore` of whole dirs.
- Do NOT touch / delete / rename / stage user's own uncommitted items: `D SLAM/3map.sh`, `?? SLAM/3trans2pcd.sh`, `?? SLAM/4downsample.sh`, `?? "SLAM/src/odin_ros_driver/config/control_command backup.yaml"` (a fully commented-out reference copy), `?? .claude/`, `?? cmu_planner/src/vehicle_simulator/launch/__pycache__/`, `?? docs/superpowers/plans/...`.
- Never `git add -A` / `git add .` / `git clean`. Stage only the files each task names. Verify staged set with `git diff --cached --name-only` (NOT `git status --porcelain` — the tree always has unrelated untracked files). No push. No `Co-Authored-By` trailer.
- Commit grouping (fixed): ① `feat(odin): add explicit slam and relocalization run modes` (configs + 2 scripts) ② `feat(odin): publish relocalization map on /overall_map` (launch + package.xml) ③ `refactor(cmu): remove overall map publishing from planner` (system_real_robot.launch + 7multi.sh + delete `cmu_planner/showmap.sh`) ④ `feat(odin): add standalone saved-map viewer` (`SLAM/showmap.sh` + `SLAM/src/odin_ros_driver/config/overall_map.rviz`). If git pairs the showmap.sh delete+add as a rename, ③/④ may be merged per the actual diff — but never mix unrelated files and never mix the user's own uncommitted files.

### Marker rules
- `# ################################` + `# Python: <desc>` / `# Bash: <desc>` / `# YAML: <desc>` BEFORE each new/modified logical block. No END markers. Never remove unrelated existing markers. If a block's purpose changes substantially, update its marker description (rule 12). `cmu_planner/showmap.sh` is deleted as a whole file (its old markers go with it); the new `SLAM/showmap.sh` is a fresh file carrying its own marker.

### Scope — do NOT add
- PCD downsampling, transient_local publishers, new C++ map publisher, map server, Target Frame changes, new TF, frame guards, auto-detection of relocalization success, auto-closing Odin RViz, auto-calling `7multi.sh`/`showmap.sh`, shell-chaining SLAM↔cmu_planner. Only: explicit SLAM/relocalization launch modes, `/overall_map` from relocalization, Odin RViz stays on, cmu_planner consumes.

---

### Task 1: Split Odin configs — SLAM mode and relocalization mode

**Files:**
- Create: `SLAM/src/odin_ros_driver/config/control_command_slam.yaml`
- Create: `SLAM/src/odin_ros_driver/config/control_command_relocalization.yaml`

**Interfaces:**
- Produces: the `config_file` targets used by Task 2 (`..._slam.yaml`) and Task 3 (`..._relocalization.yaml`). Both installed to share automatically via the existing `install(DIRECTORY config/)` rule — no CMake change.

- [ ] **Step 1: Copy the active config**

```bash
cd /home/yu/Codes_rk
cp SLAM/src/odin_ros_driver/config/control_command.yaml SLAM/src/odin_ros_driver/config/control_command_relocalization.yaml
cp SLAM/src/odin_ros_driver/config/control_command.yaml SLAM/src/odin_ros_driver/config/control_command_slam.yaml
```

- [ ] **Step 2: Relocalization config — byte-identical copy (no edit)**

`control_command_relocalization.yaml` is already the active relocalization config (`custom_map_mode: 2`, `relocalization_map_abs_path` → `map_20260807_151455.bin`). No content change. Verify:

```bash
diff SLAM/src/odin_ros_driver/config/control_command.yaml SLAM/src/odin_ros_driver/config/control_command_relocalization.yaml && echo IDENTICAL
```

Expected: `IDENTICAL`. Also confirm the relocalization fields survived the copy:

```bash
grep -n "custom_init_pos\|custom_init_pose_search_radius\|custom_init_pose_max_rot_deg\|relocalization_map_abs_path" SLAM/src/odin_ros_driver/config/control_command_relocalization.yaml
```

Expected: `custom_map_mode: 2` present; `relocalization_map_abs_path` = `/root/work/lqp/SLAM/src/odin_ros_driver/map/map_20260807_151455.bin` (unchanged).

- [ ] **Step 3: SLAM config — flip the mode to `1`**

In `control_command_slam.yaml`, insert a YAML marker above the mode line and change the value:

```yaml
  # ################################
  # YAML: enable Odin SLAM mapping mode
  # ################################
  custom_map_mode: 1
```

(replace `custom_map_mode: 2` with the marker block + `custom_map_mode: 1`). Nothing else changes. Verify the ONLY difference from the active config is the mode:

```bash
diff SLAM/src/odin_ros_driver/config/control_command.yaml SLAM/src/odin_ros_driver/config/control_command_slam.yaml
```

Expected: exactly the `custom_map_mode` line (plus the inserted marker lines).

- [ ] **Step 4: Validate both YAML files parse and carry the right mode**

```bash
python3 -c "
import yaml
for name, want in [('control_command_slam.yaml',1),('control_command_relocalization.yaml',2)]:
    p = f'/home/yu/Codes_rk/SLAM/src/odin_ros_driver/config/{name}'
    d = yaml.safe_load(open(p))
    assert d['register_keys']['custom_map_mode'] == want, (name, d['register_keys']['custom_map_mode'])
    print(name, 'mode =', d['register_keys']['custom_map_mode'])
"
```

Expected: `control_command_slam.yaml mode = 1` and `control_command_relocalization.yaml mode = 2`.

- [ ] **Step 5: No commit yet** — part of group commit ① at Task 3.

### Task 2: Add `SLAM/2run_slam.sh` — Odin SLAM mapping entry

**Files:**
- Create: `SLAM/2run_slam.sh`

**Interfaces:**
- Consumes: `control_command_slam.yaml` (Task 1); `enable_rviz` / `publish_overall_map` args (Task 4).
- Produces: Odin SLAM mapping mode, Odin RViz on, NO `/overall_map`.

- [ ] **Step 1: Write `SLAM/2run_slam.sh`**

Exact content (must match — `SCRIPT_DIR` makes it cwd-independent):

```bash
#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ################################
# Bash: launch Odin SLAM mapping mode
# ################################
ros2 launch odin_ros_driver odin1_ros2.launch.py \
  config_file:="$SCRIPT_DIR/src/odin_ros_driver/config/control_command_slam.yaml" \
  enable_rviz:=true \
  publish_overall_map:=false
```

- [ ] **Step 2: Syntax + static checks**

```bash
bash -n /home/yu/Codes_rk/SLAM/2run_slam.sh && echo OK
chmod +x /home/yu/Codes_rk/SLAM/2run_slam.sh
grep -n "control_command_slam\|enable_rviz\|publish_overall_map\|overall_map_pcd" /home/yu/Codes_rk/SLAM/2run_slam.sh
```

Expected: `OK`; the grep shows `control_command_slam.yaml`, `enable_rviz:=true`, `publish_overall_map:=false` and NO `overall_map_pcd` / NO `map_20260807_151455.pcd`.

- [ ] **Step 3: No commit yet** — part of group commit ① at Task 3.

### Task 3: Add `SLAM/3run_relocalization.sh` — Odin relocalization entry

**Files:**
- Create: `SLAM/3run_relocalization.sh` (name is STRICT — not `2run_relocalization.sh`)

**Interfaces:**
- Consumes: `control_command_relocalization.yaml` (Task 1); `enable_rviz` / `publish_overall_map` / `overall_map_pcd` args (Task 4).
- Produces: Odin relocalization (`custom_map_mode: 2`), Odin RViz on, `/overall_map` (frame `odin_map`, period 10000 ms) published by the Odin launch.

- [ ] **Step 1: Write `SLAM/3run_relocalization.sh`**

Exact content:

```bash
#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ################################
# Bash: launch Odin relocalization and publish map on /overall_map
# ################################
ros2 launch odin_ros_driver odin1_ros2.launch.py \
  config_file:="$SCRIPT_DIR/src/odin_ros_driver/config/control_command_relocalization.yaml" \
  enable_rviz:=true \
  publish_overall_map:=true \
  overall_map_pcd:="$SCRIPT_DIR/src/odin_ros_driver/map/map_20260807_151455.pcd"
```

- [ ] **Step 2: Syntax + static checks**

```bash
bash -n /home/yu/Codes_rk/SLAM/3run_relocalization.sh && echo OK
chmod +x /home/yu/Codes_rk/SLAM/3run_relocalization.sh
grep -n "control_command_relocalization\|publish_overall_map\|overall_map_pcd\|map_20260807_151455.pcd\|enable_rviz" /home/yu/Codes_rk/SLAM/3run_relocalization.sh
```

Expected: `OK`; grep shows `control_command_relocalization.yaml`, `publish_overall_map:=true`, `overall_map_pcd:="$SCRIPT_DIR/src/odin_ros_driver/map/map_20260807_151455.pcd"`, `enable_rviz:=true`.

- [ ] **Step 3: Group commit ①**

```bash
cd /home/yu/Codes_rk
git add SLAM/2run_slam.sh SLAM/3run_relocalization.sh SLAM/src/odin_ros_driver/config/control_command_slam.yaml SLAM/src/odin_ros_driver/config/control_command_relocalization.yaml
git diff --cached --name-only   # must print exactly those 4 paths (nothing else — user's SLAM files stay unstaged)
git commit -m "feat(odin): add explicit slam and relocalization run modes"
```

### Task 4: `odin1_ros2.launch.py` — `enable_rviz` + conditional `/overall_map` publisher

**Files:**
- Modify: `SLAM/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py`

**Interfaces:**
- Consumes: `config_file` arg (already exists, used by Tasks 2-3 scripts); `overall_map_pcd` (absolute PCD path from Task 3).
- Produces (new launch args): `enable_rviz` (default `true`), `publish_overall_map` (default `false`), `overall_map_pcd` (default `''`); the `overall_map_publisher` node publishing `/overall_map` (frame `odin_map`, period 10000 ms) when `publish_overall_map=true`.

- [ ] **Step 1: Add the `IfCondition` import**

After the existing import line `from launch_ros.actions import Node` (L9), add:

```python
from launch.conditions import IfCondition
```

- [ ] **Step 2: Declare the three new launch arguments**

After the existing `rviz_config_arg` (L23-27), add:

```python
    # ################################
    # Python: declare RViz-on and relocalization map publish options
    # ################################
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Start the Odin RViz'
    )
    publish_overall_map_arg = DeclareLaunchArgument(
        'publish_overall_map',
        default_value='false',
        description='Publish relocalization map PCD on /overall_map'
    )
    overall_map_pcd_arg = DeclareLaunchArgument(
        'overall_map_pcd',
        default_value='',
        description='Path to the map PCD published on /overall_map'
    )
```

- [ ] **Step 3: Make `rviz_node` conditional on `enable_rviz`**

In the existing `rviz_node` Node (L98-104), add the marker + condition line:

```python
    # ################################
    # Python: make Odin RViz optional via enable_rviz
    # ################################
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
```

(only the marker + `condition=` line are added; the rest is unchanged — default behavior keeps Odin RViz on.)

- [ ] **Step 4: Add the conditional `/overall_map` publisher node**

Before the `# Create launch description` comment (L106), add:

```python
    # ################################
    # Python: optionally publish relocalization map on /overall_map
    # ################################
    overall_map_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='overall_map_publisher',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('publish_overall_map')
        ),
        parameters=[{
            'file_name': LaunchConfiguration('overall_map_pcd'),
            'tf_frame': 'odin_map',
            'publishing_period_ms': 10000,
        }],
        remappings=[
            ('cloud_pcd', '/overall_map'),
        ]
    )
```

Field-verified param names only: `file_name`, `tf_frame`, `publishing_period_ms` — do not rename.

- [ ] **Step 5: Add the new actions to the LaunchDescription**

In the `ld.add_action(...)` section (L107-115), add:

```python
    ld.add_action(enable_rviz_arg)
    ld.add_action(publish_overall_map_arg)
    ld.add_action(overall_map_pcd_arg)
    ld.add_action(overall_map_node)
```

- [ ] **Step 6: Verify syntax + default behavior**

```bash
cd /home/yu/Codes_rk/SLAM
source /opt/ros/humble/setup.bash
python3 -m py_compile src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py && echo COMPILE-OK
ros2 launch odin_ros_driver odin1_ros2.launch.py --show-args 2>&1 | grep -A1 "enable_rviz\|publish_overall_map\|overall_map_pcd" | head -12
```

Expected: `COMPILE-OK`; `--show-args` lists the three new args (defaults `true`/`false`/`''`). NOTE: `--show-args` reads the INSTALLED share launch — with `--symlink-install` it is live; if the installed copy looks stale, run Task 6's build first.

- [ ] **Step 7: No commit yet** — part of group commit ② at Task 5.

### Task 5: `package.xml` — add `pcl_ros` runtime dependency

**Files:**
- Modify: `SLAM/src/odin_ros_driver/package.xml`

**Interfaces:**
- Produces: the `pcl_ros` manifest dependency required because the Odin launch now runs `pcl_ros/pcd_to_pointcloud` at runtime.

- [ ] **Step 1: Add the exec dependency**

After the existing `<exec_depend>rosidl_default_runtime</exec_depend>` (L27), add:

```xml
  <!-- ################################ -->
  <!-- XML: add pcl_ros launch runtime dependency -->
  <!-- ################################ -->
  <exec_depend>pcl_ros</exec_depend>
```

- [ ] **Step 2: Verify**

```bash
grep -n "<exec_depend>pcl_ros</exec_depend>" /home/yu/Codes_rk/SLAM/src/odin_ros_driver/package.xml
```

Expected: exactly one line — the new `<exec_depend>pcl_ros</exec_depend>` (the marker comment above it also contains the word `pcl_ros`, so grep for the full tag, not the bare word).

Constraints: launch-runtime dependency only. Do NOT add `find_package(pcl_ros REQUIRED)` to `CMakeLists.txt` (not needed — the package's own C++ code does not link pcl_ros). Do NOT touch `host_sdk_sample` targets.

- [ ] **Step 3: Group commit ②**

```bash
cd /home/yu/Codes_rk
git add SLAM/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py SLAM/src/odin_ros_driver/package.xml
git diff --cached --name-only   # must print exactly those 2 paths
git commit -m "feat(odin): publish relocalization map on /overall_map"
```

### Task 6: Rebuild the SLAM workspace (odin_ros_driver)

**Files:** none (build only, no commit).

- [ ] **Step 1: Build**

```bash
cd /home/yu/Codes_rk/SLAM
source /opt/ros/humble/setup.bash
colcon build --packages-select odin_ros_driver --symlink-install --executor sequential
source install/setup.bash
```

Expected: build completes cleanly.

- [ ] **Step 2: Confirm install space is current**

```bash
ls /home/yu/Codes_rk/SLAM/install/odin_ros_driver/share/odin_ros_driver/config/control_command_slam.yaml /home/yu/Codes_rk/SLAM/install/odin_ros_driver/share/odin_ros_driver/config/control_command_relocalization.yaml
ros2 launch odin_ros_driver odin1_ros2.launch.py --show-args 2>&1 | grep -c "enable_rviz\|publish_overall_map\|overall_map_pcd"
```

Expected: both config files visible in the install share; `--show-args` shows the three new args (count = 3).

### Task 7: `system_real_robot.launch` — remove the PCD map publisher, keep RViz selection

**Files:**
- Modify: `cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch`

**Interfaces:**
- Consumes: the `rviz_config_file` mechanism from commit `b30227e` (KEEP).
- Produces: a launch that selects the RViz config only — no map asset knowledge (no `pcd_to_pointcloud`, no `/overall_map`, no `enable_pcd_map`/`pcd_map_file`).

- [ ] **Step 1: Remove the `enable_pcd_map` / `pcd_map_file` bindings**

Delete from the bindings block (currently ~L38-43; keep the `rviz_config_file` binding + its marker):

```python
  enable_pcd_map = LaunchConfiguration('enable_pcd_map')
  pcd_map_file = LaunchConfiguration('pcd_map_file')
```

- [ ] **Step 2: Remove the two DeclareLaunchArguments**

Delete `declare_enable_pcd_map` and `declare_pcd_map_file` (currently ~L78-81). Keep `declare_rviz_config_file` (L74-77).

- [ ] **Step 3: Remove the PCD publisher node**

Delete the entire `start_pcd_map` Node block (currently ~L170-187) including its marker comment `# Python: publish prebuilt map PCD on /overall_map when enabled`.

- [ ] **Step 4: Remove the three `ld.add_action` lines**

Delete `ld.add_action(declare_enable_pcd_map)`, `ld.add_action(declare_pcd_map_file)`, `ld.add_action(start_pcd_map)` (currently ~L229-230, L239). Keep `ld.add_action(declare_rviz_config_file)` and `ld.add_action(delayed_start_rviz)`.

- [ ] **Step 5: Verify the launch has no map-asset knowledge**

```bash
cd /home/yu/Codes_rk
grep -n "pcd\|overall_map\|enable_pcd\|pcd_map\|pcl_ros\|cloud_pcd" cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch || echo "NO-MAP-REFS"
grep -n "rviz_config_file\|declare_rviz_config_file\|arguments=\['-d', rviz_config_file\]\|TimerAction" cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch
```

Expected: first grep prints `NO-MAP-REFS` (no `pcd`/`overall_map`/`pcl_ros` anywhere); second grep still shows the `rviz_config_file` binding, `declare_rviz_config_file`, `arguments=['-d', rviz_config_file]`, and the `TimerAction(period=8.0)`.

- [ ] **Step 6: Python compile check**

```bash
python3 -m py_compile cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch && echo COMPILE-OK
```

- [ ] **Step 7: No commit yet** — part of group commit ③ at Task 9.

### Task 8: `7multi.sh` — drop map-publish args, keep `cruise_map.rviz` selection

**Files:**
- Modify: `cmu_planner/7multi.sh`

**Interfaces:**
- Consumes: `rviz_config_file` launch arg (Task 2 of previous plan, kept) and `cruise_map.rviz`.
- Produces: `frame=odin_map` → passes ONLY `rviz_config_file:=.../cruise_map.rviz`; never starts a PCD publisher; `/overall_map` comes from `SLAM/3run_relocalization.sh`.

- [ ] **Step 1: Simplify the `map_args` block**

Replace the current `map_args` block (commit `86e572b`, ~L45-56) with:

```bash
# ################################
# Bash: select cruise_map.rviz for map-mode MULTI
# ################################
map_args=()
if [[ "$frame" == "odin_map" ]]; then
  ROOT="$(cd "$(dirname "$0")/.." && pwd)"
  map_args+=(
    rviz_config_file:="$ROOT/cmu_planner/src/vehicle_simulator/rviz/cruise_map.rviz"
  )
fi
```

(remove the `enable_pcd_map:=true` and `pcd_map_file:=...` lines entirely). The `"${map_args[@]}"` insertion into the launch command stays.

- [ ] **Step 2: Update the usage text**

Replace the two map-mode usage lines (currently ~L26-28) with:

```
  echo "  bash 7multi.sh rviz -1 odin_map # Map-mode MULTI (needs Odin relocalization):"
  echo "                                  # requires SLAM/3run_relocalization.sh; /overall_map"
  echo "                                  # is published by the SLAM stack; opens cruise_map.rviz"
  echo "                                  # (Fixed Frame=odin_map, OverallMap enabled)."
```

- [ ] **Step 3: Verify no map-asset knowledge remains**

```bash
cd /home/yu/Codes_rk
grep -n "enable_pcd_map\|pcd_map_file\|pcd_to_pointcloud\|map_20260807_151455.pcd\|SLAM/src/odin_ros_driver/map" cmu_planner/7multi.sh || echo "NO-MAP-REFS"
bash -n cmu_planner/7multi.sh && echo SYNTAX-OK
grep -n "cruise_map.rviz\|rviz_config_file\|showmap" cmu_planner/7multi.sh
```

Expected: `NO-MAP-REFS`, `SYNTAX-OK`; the grep shows `cruise_map.rviz` selection + `rviz_config_file` arg + NO `showmap` call.

- [ ] **Step 4: No commit yet** — part of group commit ③ at Task 9.

### Task 9: Move the standalone saved-map viewer into SLAM

**Files:**
- Delete: `cmu_planner/showmap.sh` (committed — normal git delete in a new incremental commit; NO reset/revert)
- Create: `SLAM/showmap.sh`
- Create: `SLAM/src/odin_ros_driver/config/overall_map.rviz`

**Interfaces:**
- Consumes: the map PCD at `SLAM/src/odin_ros_driver/map/map_20260807_151455.pcd`; the d933ddd-verified process-cleanup approach.
- Produces: an offline saved-map viewer fully owned by SLAM — it starts its own `pcl_ros/pcd_to_pointcloud` publishing `/overall_map` (frame `odin_map`, period 10000 ms) and opens SLAM's own `overall_map.rviz`. NEVER references `cmu_planner` (no reverse dependency). Independent path — do NOT run together with `3run_relocalization.sh` (both publish `/overall_map`).

- [ ] **Step 1: Create `SLAM/src/odin_ros_driver/config/overall_map.rviz`**

Minimal saved-map viewer config. Do NOT reuse `odin_ros2.rviz` (that is the Odin runtime-status monitor: image/cloud_slam/odometry — not an offline map viewer). Do NOT touch `cmu_planner/cruise_map.rviz`. Global Options `Fixed Frame: odin_map`; displays: Grid (optional) + one PointCloud2 `OverallMap`:

```yaml
# ################################
# YAML: SLAM standalone saved-map viewer
# ################################
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_default_plugins/PointCloud2
      Enabled: true
      Name: OverallMap
      Color Transformer: FlatColor
      Position Transformer: XYZ
      Style: Points
      Size (Pixels): 2
      Decay Time: 0
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /overall_map
  Global Options:
    Background Color: 0; 0; 0
    Fixed Frame: odin_map
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
```

The current PCD has only x/y/z — do NOT depend on RGB8 or Intensity for map coloring. Do NOT add `MultiWaypointTool`, `/path_viz`, `/terrain_map`, planner displays, or cmu_planner plugins. The existing `install(DIRECTORY config/)` rule installs this file — no CMakeLists change.

- [ ] **Step 2: Create `SLAM/showmap.sh`**

Exact content — SLAM owns the PCD + `pcd_to_pointcloud`; opens SLAM's `overall_map.rviz`; cleanup uses the d933ddd-verified approach (`pkill -f 'pcd_to_pointcloud'`, NOT `kill $MAP_PID` which orphans the node — do not reintroduce that bug); NO `sleep 12`:

```bash
#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ################################
# Bash: publish saved Odin map for standalone visualization
# ################################
cd "$SCRIPT_DIR"

ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="src/odin_ros_driver/map/map_20260807_151455.pcd" \
  -p tf_frame:=odin_map \
  -p publishing_period_ms:=10000 \
  -r cloud_pcd:=/overall_map &

# ################################
# Bash: clean up map publisher when standalone viewer exits
# ################################
cleanup() {
  pkill -f 'pcd_to_pointcloud' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Launch RViz immediately: the publisher is a periodic volatile publisher,
# so subscribing now catches the first ~10 s tick (no sleep).
ros2 run rviz2 rviz2 -d \
  "$SCRIPT_DIR/src/odin_ros_driver/config/overall_map.rviz"
```

- [ ] **Step 3: Delete `cmu_planner/showmap.sh`**

```bash
cd /home/yu/Codes_rk
git rm cmu_planner/showmap.sh
```

(removes the working file and stages the deletion — the clean way to delete a committed file.)

- [ ] **Step 4: Static checks**

```bash
cd /home/yu/Codes_rk
echo "--- cmu_planner: no showmap.sh, no map-asset terms ---"
ls cmu_planner/showmap.sh 2>&1 || echo "SHOWMAP-GONE"
grep -rn "map_20260807_151455.pcd\|pcd_to_pointcloud\|enable_pcd_map\|pcd_map_file\|SLAM/src/odin_ros_driver/map" cmu_planner/ || echo "CLEAN"
echo "--- SLAM/showmap.sh must not reference cmu_planner ---"
grep -n "cmu_planner\|cruise_map" SLAM/showmap.sh || echo "NO-CMU-REFS"
bash -n SLAM/showmap.sh && echo SYNTAX-OK
echo "--- publish block + rviz path ---"
grep -n "pcd_to_pointcloud\|map_20260807_151455.pcd\|tf_frame:=odin_map\|overall_map.rviz" SLAM/showmap.sh
```

Expected: `SHOWMAP-GONE`, cmu_planner `CLEAN`, `NO-CMU-REFS`, `SYNTAX-OK`; last grep shows the publish block + `overall_map.rviz` path (no `cruise_map`, no `cmu_planner`).

- [ ] **Step 5: Runtime smoke test (local, no device, no cmu_planner)**

```bash
cd /home/yu/Codes_rk/SLAM
source /opt/ros/humble/setup.bash
bash showmap.sh > /tmp/showmap_slam.log 2>&1 &
sleep 4
pgrep -af "rviz2 -d .*overall_map.rviz" | head -2   # expect: rviz2 with SLAM overall_map.rviz
sleep 8
ros2 topic echo /overall_map --once --field header.frame_id   # expect: odin_map
ros2 topic echo /overall_map --once --field width             # expect: 405532 (current check value)
pgrep -af pcd_to_pointcloud | wc -l                  # expect: 1 (single publisher, this viewer's)
pgrep -f host_sdk_sample | wc -l                     # expect: 0 (no Odin device process)
pkill -f rviz2
sleep 2
pgrep -af pcd_to_pointcloud | wc -l                  # expect: 0 (cleanup ran)
```

- [ ] **Step 6: Group commit ③ and ④**

```bash
cd /home/yu/Codes_rk
git add cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch cmu_planner/7multi.sh
git diff --cached --name-only   # must print exactly: launch file, 7multi.sh, and showmap.sh (deleted)
git commit -m "refactor(cmu): remove overall map publishing from planner"
git add SLAM/showmap.sh SLAM/src/odin_ros_driver/config/overall_map.rviz
git diff --cached --name-only   # must print exactly those 2 paths
git commit -m "feat(odin): add standalone saved-map viewer"
```

Note: `git rm cmu_planner/showmap.sh` (Step 3) already staged the deletion, so commit ③ carries `showmap.sh` as deleted alongside the two modified cmu_planner files. If git pairs that deletion with the new `SLAM/showmap.sh` as a rename (similar content), ③/④ may be merged per the actual diff — follow the diff, never mixing unrelated files.

### Task 10: End-to-end acceptance

**Files:** none (verification only, no commit). On-robot items are marked PENDING-ROBOT.

**Focus checks (must all be YES):**
1. `bash SLAM/2run_slam.sh` → SLAM mode, Odin RViz on, NO `/overall_map`.
2. `bash SLAM/3run_relocalization.sh` → relocalization mode, Odin RViz on, `/overall_map` published by SLAM with `frame_id=odin_map`.
3. `bash cmu_planner/7multi.sh rviz -1 odin_map` → does NOT read PCD, does NOT start `pcd_to_pointcloud`, opens `cruise_map.rviz`, consumes the SLAM-published `/overall_map`.
4. Formal relocalization + MULTI allows TWO RViz (Odin RViz for localization monitoring + cruise_map.rviz for waypoint cruise) — YES.
5. cmu_planner is completely ignorant of `SLAM/src/odin_ros_driver/map/map_20260807_151455.pcd` AND contains no `showmap.sh` — YES.

- [ ] **Step 1: Static whole-tree checks (local)**

```bash
cd /home/yu/Codes_rk
echo "--- cmu_planner must not know the PCD and must have no showmap.sh ---"
ls cmu_planner/showmap.sh 2>&1 || echo "SHOWMAP-GONE"
grep -rn "map_20260807_151455.pcd\|pcd_to_pointcloud\|enable_pcd_map\|pcd_map_file\|cloud_pcd" cmu_planner/ || echo "CLEAN"
echo "--- the 2 changed cmu_planner files carry no pcl_ros / no PCD-asset refs ---"
grep -l "pcl_ros\|map_20260807_151455\|pcd_to_pointcloud\|enable_pcd_map\|pcd_map_file" cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch cmu_planner/7multi.sh 2>/dev/null || echo "CLEAN-2"
echo "--- SLAM-side map references (allowed hits: 3run_relocalization.sh, showmap.sh, configs/tools) ---"
grep -rln "map_20260807_151455.pcd" SLAM/
echo "--- SLAM configs parse ---"
python3 -c "
import yaml
for n in ['control_command.yaml','control_command_slam.yaml','control_command_relocalization.yaml']:
    d=yaml.safe_load(open(f'/home/yu/Codes_rk/SLAM/src/odin_ros_driver/config/{n}'))
    print(n, 'mode=', d['register_keys']['custom_map_mode'])
"
echo "--- odin launch shows args ---"
cd /home/yu/Codes_rk/SLAM && source /opt/ros/humble/setup.bash && ros2 launch odin_ros_driver odin1_ros2.launch.py --show-args 2>&1 | grep -A1 "enable_rviz\|publish_overall_map\|overall_map_pcd" | head -12
```

Expected: `SHOWMAP-GONE` + cmu_planner grep prints `CLEAN`; `CLEAN-2` (NOTE: `7multi.sh`'s usage text legitimately mentions `/overall_map` and `SLAM/3run_relocalization.sh` — the CLEAN-2 grep deliberately excludes the bare word `overall_map`; a help-text mention is intended, not a violation); the SLAM grep lists ONLY SLAM-side files (`3run_relocalization.sh`, `showmap.sh`, `control_command*.yaml` and any SLAM tool) — never a cmu_planner path; the three configs parse with modes `2/1/2`; `--show-args` lists the three new args.

- [ ] **Step 2: Local headless launch test — `/overall_map` from the Odin launch (no device needed)**

The `overall_map_publisher` runs without the device (host_sdk_sample will fail to connect over USB on this machine — expected, launch keeps other nodes up). Run headless (`enable_rviz:=false`):

```bash
cd /home/yu/Codes_rk/SLAM && source /opt/ros/humble/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py \
  publish_overall_map:=true \
  overall_map_pcd:="$PWD/src/odin_ros_driver/map/map_20260807_151455.pcd" \
  enable_rviz:=false > /tmp/odin_map_test.log 2>&1 &
sleep 15
ros2 node list | grep overall_map_publisher                 # expect: /overall_map_publisher
ros2 topic echo /overall_map --once --field header.frame_id  # expect: odin_map
ros2 topic echo /overall_map --once --field width            # expect: 405532 (current map check value)
pgrep -af pcd_to_pointcloud | wc -l                          # expect: 1 (single publisher)
pkill -f "odin1_ros2.launch.py"; pkill -f "pcd_to_pointcloud"; pkill -f "host_sdk_sample"
```

- [ ] **Step 3: Local headless check — default launch publishes nothing**

```bash
cd /home/yu/Codes_rk/SLAM && source /opt/ros/humble/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py enable_rviz:=false > /tmp/odin_default_test.log 2>&1 &
sleep 15
ros2 node list | grep -c overall_map_publisher               # expect: 0
pkill -f "odin1_ros2.launch.py"; pkill -f "host_sdk_sample"
```

- [ ] **Step 4: Standalone saved-map viewer acceptance (local, no device, no cmu_planner)**

```bash
cd /home/yu/Codes_rk/SLAM
source /opt/ros/humble/setup.bash
bash showmap.sh > /tmp/showmap_accept.log 2>&1 &
sleep 4
pgrep -af "rviz2 -d .*overall_map.rviz" | head -2   # expect: SLAM overall_map.rviz opens automatically
sleep 8
ros2 topic echo /overall_map --once --field header.frame_id   # expect: odin_map
ros2 topic echo /overall_map --once --field width             # expect: 405532 (current check value)
pgrep -af pcd_to_pointcloud | wc -l                  # expect: 1 (single publisher, this viewer's)
pgrep -f host_sdk_sample | wc -l                     # expect: 0 (no Odin device)
pgrep -f "localPlanner\|cruiseController" | wc -l    # expect: 0 (no cmu_planner)
pkill -f rviz2
sleep 2
pgrep -af pcd_to_pointcloud | wc -l                  # expect: 0 (cleanup after RViz close)
```
Expected: one RViz (`overall_map.rviz`, Fixed Frame `odin_map`, OverallMap enabled) + `/overall_map` from SLAM's own `pcd_to_pointcloud` only; no `host_sdk_sample`, no cmu_planner, no dependency on `cruise_map.rviz`; publisher cleaned up when RViz closes.

- [ ] **Step 5: On-robot acceptance (PENDING-ROBOT; report as such)**

**A. SLAM mode** — `cd SLAM && bash 2run_slam.sh`:
- `host_sdk_sample` runs, `custom_map_mode=1`, Odin RViz auto-opens, `ros2 node list | grep overall_map_publisher` → no result, no `/overall_map`.

**B. Relocalization mode** — `cd SLAM && bash 3run_relocalization.sh`:
- `custom_map_mode=2`, loads `map_20260807_151455.bin`, Odin RViz auto-opens, `ros2 run tf2_ros tf2_echo odin_map odin_odom` works; `ros2 node list | grep overall_map_publisher` → present; `ros2 topic info /overall_map -v` → exactly 1 publisher; `ros2 topic echo /overall_map --once --field header.frame_id` → `odin_map`; width → `405532` (current check value, not a permanent contract).

**C. Map-mode MULTI** — keep `3run_relocalization.sh` running, new terminal `cd cmu_planner && bash 7multi.sh rviz -1 odin_map`:
- TWO RViz allowed: (1) Odin RViz monitoring relocalization, (2) `cruise_map.rviz` with Fixed Frame `odin_map`, OverallMap enabled, shows `/overall_map`, MultiWaypointTool present.
- Click → `ros2 topic echo /multi_waypoint_add --once --field header.frame_id` → `odin_map`. `/multi_start` → robot follows the map-frame MULTI route.
- Frame checks (unchanged): `/path_viz` → `odin_odom`, `/terrain_map` → `odin_odom`, `/way_point` → `odin_odom`.

**D. No-map regression** — no `3run_relocalization.sh`; `cd cmu_planner && bash 7multi.sh rviz` (or `rviz -1 odin_odom`): `vehicle_simulator.rviz`, Fixed Frame `odin_odom`, no `/overall_map` / `odin_map` / PCD dependency; no-map MULTI unchanged.

**E. Standalone viewer is a separate path** — `cd SLAM && bash showmap.sh` (covered by Step 4) is for offline saved-map viewing and MUST NOT run together with `3run_relocalization.sh` (both would publish `/overall_map` → two publishers).

- [ ] **Step 6: Report**

Write the report file with per-step results (pass / fail / pending-robot), the exact outputs of Steps 1-4 commands, the Step 5 items marked pending-robot, and the five focus-check answers.
