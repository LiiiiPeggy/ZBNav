# Odin + CMU Planner Frame-Semantics Unification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rename the Odin1/CMU-Planner TF frames to the `odin_` namespace (`odin_map`, `odin_odom`, `odin1_base_link`) so no-map local cruise and prebuilt-map global multi-point cruise both work, WITHOUT touching the motion-control `map → base → legs…` TF tree.

**Architecture:** The motion-control TF tree (`map → base → legs…`) is untouched and stays a separate, unconnected tree. All Odin frame labels that are currently bare `"odom"` / `"map"` become `"odin_odom"` / `"odin_map"`. The CMU planner chain (`/state_estimation`, `/registered_scan`, `/terrain_map`, local planning, terrain analysis) operates entirely in `odin_odom`. Cruise/multi accepts goals in `odin_odom` directly (no-map) or transforms `odin_map → odin_odom` at the cruise layer when the relocalization TF exists; goals labeled with the motion-control `"map"` are rejected. Frame strings are parameterized (`planning_frame` = `"odin_odom"`, `global_frame` = `"odin_map"`) instead of hardcoded.

**Tech Stack:** C++ (ROS 2 Humble), tf2, PCL. Files span `odin_ros_driver` (SLAM ws), `local_planner` / `terrain_analysis` / `terrain_analysis_ext` / `sensor_scan_generation` / `waypoint_rviz_plugin` / `vehicle_simulator` (planner ws).

## Global Constraints

- **Never touch the motion-control TF**: `map → base → legs…` must remain byte-for-byte unchanged. Do NOT modify/delete/re-publish/connect the motion-control `map→base` TF. Do NOT add `map→odin_map` / `map→odin_odom` / `base→odin1_base_link` links.
- **No bare `"map"` / `"odom"` frame labels for Odin data.** Use `odin_map` / `odin_odom` / `odin1_base_link`. (The motion-control `"map"` stays as-is; it belongs to the other tree.)
- **Renaming a frame label is NOT enough when the parent/child direction changes.** The STANDARD odometry branch keeps its values (`odom→odin1_base_link` → `odin_odom→odin1_base_link`). The relocalization branch must PUBLISH THE INVERSE (see Task 1 Step 2): the target tree is `odin_map → odin_odom`, but the currently-published value is labeled `odom→map` (i.e. `T_odom_map`), so inverting is required — not a plain relabel.
- **A `header.frame_id` label change never substitutes for verifying the coordinate VALUES belong to that frame.** Where a node subtracts point/pose values directly without tf2 (terrainAnalysis, localPlanner, sensorScanGeneration), the inputs must genuinely share `odin_odom`; a mismatch is REJECTED (fail-fast), never warned-and-continued.
- `localPlanner` / `terrain_analysis` / `terrain_analysis_ext` and `/terrain_map` stay in `odin_odom`; do NOT convert terrain to `odin_map`.
- No-map mode MUST work with only `odin_odom → odin1_base_link`; it must not require any `odin_map` TF.
- Map mode: global goals are `odin_map`; the cruise/multi layer transforms them `odin_map → odin_odom` before handing to localPlanner.
- Goals whose `header.frame_id` is the bare motion-control `"map"` are REJECTED, never silently accepted.
- Parameters `planning_frame` (default `"odin_odom"`) and `global_frame` (default `"odin_map"`) added where frame strings are used; legacy `multi_frame` values `"odom"`/`"map"` are normalized to `odin_odom`/`odin_map` with a WARN (backward compat).
- `loamInterface.cpp` is **NOT modified** — it is not launched by the Odin pipeline (verified: `odin1_ros2.launch.py` starts only `host_sdk_sample` + `registered_scan_adapter_node`); it belongs to the legacy super_lio/LOAM path.
- Do NOT restructure planner algorithms, path-planning params, or avoidance params. Minimal changes only.
- Project marker rule: leading `// ################################` + `// C++: <description>` (C++), `# ################################` + `# Bash:` (bash), `# YAML:` (yaml), `<!-- XML: -->` (launch XML) markers on new/modified blocks; no END markers.
- Commit messages have NO `Co-Authored-By` trailer.
- Build commands run from the workspace root of each package (`/home/yu/Codes_rk/SLAM` for odin_ros_driver, `/home/yu/Codes_rk/planner` for the planner packages), after `source /opt/ros/humble/setup.bash`.
- Working tree contains unrelated untracked junk (`.claude/`, `SLAM/src/odin_ros_driver/config/control_command backup.yaml`, `planner/src/vehicle_simulator/launch/__pycache__/`) — never stage or commit it; stage only the exact files each task names.

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `SLAM/src/odin_ros_driver/include/host_sdk_sample.h` | Modify | Rename Odin frame labels `"odom"→"odin_odom"`, Odin `"map"→"odin_map"` |
| `SLAM/src/odin_ros_driver/src/registered_scan_adapter_node.cpp` | No change | Already preserves the input header — inherits `odin_odom` automatically |
| `planner/src/sensor_scan_generation/src/sensorScanGeneration.cpp` | Modify | `planning_frame` param; drop hardcoded `"map"` |
| `planner/src/terrain_analysis/src/terrainAnalysis.cpp` | Modify | state/registered frame-mismatch check + `[FRAME]` first-data log |
| `planner/src/terrain_analysis_ext/src/terrainAnalysisExt.cpp` | Modify | same mismatch check + log |
| `planner/src/local_planner/src/cruiseController.cpp` | Modify | `multi_frame` odin_ values, `planning_frame`/`global_frame` params, goal frame policy, TF frames, logs |
| `planner/src/waypoint_rviz_plugin/src/waypoint_tool.cpp` | Modify | `waypoint.header.frame_id` from Fixed Frame, not `"map"` |
| `planner/7multi.sh` / `9multi_debug.sh` | Modify | `frame=${3:-odin_odom}`, validation + legacy normalization, usage text |
| `planner/src/local_planner/launch/cruise.launch` | Modify | `multi_frame` default `odin_odom` + `planning_frame`/`global_frame` args/params |
| `planner/src/vehicle_simulator/launch/system_real_robot.launch` | Modify | `multi_frame` default `odin_odom` + `planning_frame`/`global_frame` declares/forwarding, description |
| `planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz` | Modify | Fixed Frame `odin_odom` |
| `planner/src/loam_interface/src/loamInterface.cpp` | **No change** | Legacy LOAM path, not launched by Odin pipeline (documented) |

---

### Task 1: Odin driver frame rename

**Files:**
- Modify: `SLAM/src/odin_ros_driver/include/host_sdk_sample.h`

**Interfaces:**
- Consumes: nothing
- Produces: Odin publishes `/odin1/odometry` `header.frame_id="odin_odom"`, `child_frame_id="odin1_base_link"`; TF `odin_odom → odin1_base_link` (STANDARD); relocalization TF `odin_map → odin_odom` (TRANSFORM, inverted); `/odin1/cloud_slam` and the gray/intensity image `header.frame_id` renamed. `/registered_scan` (adapter) and `/state_estimation` (launch remap) inherit `odin_odom` automatically.

**Target TF topology after this task (must be verified with `view_frames`):**
```
odin_map
└── odin_odom
    └── odin1_base_link
```
NOT `odin_odom ├── odin_map` (that would be the wrong direction).

**Renames to apply — PLAIN label renames** (all in `host_sdk_sample.h`, both ROS2 and ROS1 branches where present; transform values unchanged because the parent/child direction is preserved):

| Line(s) | Current | Change to | Context |
|---------|---------|-----------|---------|
| 778 | `msg.header.frame_id = "map"` | `"odin_map"` | `publishGrayUInt8` grayscale/intensity image |
| 911, 939 | `msg.header.frame_id = "odom"` | `"odin_odom"` | `publishPC2XYZRGBA` → `/odin1/cloud_slam` |
| 1187, 1191 | `msg.header.frame_id = "odom"` | `"odin_odom"` | second odometry publish (odometry TF variant) |
| 1237 | `msg.header.frame_id = "odom"` | `"odin_odom"` | `publishOdometry` header |
| 1329, 1425 | `transformStamped.header.frame_id = "odom"` | `"odin_odom"` | odom→base TF parent (STANDARD) |

> The relocalization TRANSFORM branch (currently `odom→map` at lines ~1401-1416 and ~1500-1512) is handled SEPARATELY in Step 2 — it must NOT be a plain string rename.

- [ ] **Step 1: Apply the plain label renames**

Apply every row of the table above with exact string edits. These change only frame labels; transform VALUES are unchanged because the parent/child direction is preserved (`odin_odom → odin1_base_link` has the same meaning as `odom → odin1_base_link`). Add a single leading marker block before the first edited function:

```cpp
// ################################
// C++: rename Odin TF frames to odin_odom / odin_map
// ################################
```

(one marker at the top of the first edit site; the rest of the edits are inside the same logical rename and need no additional markers).

- [ ] **Step 2: Fix the relocalization TRANSFORM direction (invert)**

The TRANSFORM branch (`OdometryType::TRANSFORM`, ~lines 1401-1416 and the ROS1 mirror ~1500-1512) currently builds the TransformStamped from the odometry pose with `header.frame_id="odom"`, `child_frame_id="map"`, value = the odometry pose. By TF convention this value IS `T_odom_map`. The target tree `odin_map → odin_odom` needs `T_map_odom = inverse(T_odom_map)`, so the branch must be rewritten to publish the inverse:

```cpp
  // ################################
  // C++: publish odin_map -> odin_odom relocalization TF (invert odom->map)
  // ################################
  tf2::Transform t_odom_map;
  t_odom_map.setOrigin(tf2::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
  t_odom_map.setRotation(tf2::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
  tf2::Transform t_map_odom = t_odom_map.inverse();
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = msg.header.stamp;
  transformStamped.header.frame_id = "odin_map";   // parent
  transformStamped.child_frame_id = "odin_odom";   // child
  transformStamped.transform = tf2::toMsg(t_map_odom);
  tf_broadcaster->sendTransform(transformStamped);
```

The ROS1 mirror branch gets the identical change with `geometry_msgs::TransformStamped` / `tf2::toMsg`. After this step the single navigation chain is `odin_map → odin_odom → odin1_base_link` (with `sensor_at_scan` a child of `odin_odom` published by `sensor_scan_generation`, Task 2).

> **需要实施前确认 — SDK TRANSFORM value direction.** The current code LABELS the value `odom→map`, so by TF convention it is `T_odom_map` and the inversion above follows. However the SDK's `LIDAR_DT_SLAM_ODOMETRY_TF` pose semantics cannot be confirmed from the driver source alone. After this step, run `ros2 run tf2_ros tf2_echo odin_map odin_odom` and compare the robot's pose in `odin_map` against the prebuilt map (RViz). If the pose is mirrored or rotated 180° vs. the map, the SDK's value is actually `T_map_odom` and the inverse must be dropped. Exactly one of the two is correct — verify on the robot, do not guess.

- [ ] **Step 3: Verify no remaining bare Odin frames**

```bash
grep -n 'frame_id *=[[:space:]]*"odom"\|frame_id *=[[:space:]]*"map"\|child_frame_id *=[[:space:]]*"odom"\|child_frame_id *=[[:space:]]*"map"' \
  SLAM/src/odin_ros_driver/include/host_sdk_sample.h SLAM/src/odin_ros_driver/src/host_sdk_sample.cpp
```

Expected: only the motion-control `"map"` string may remain in non-Odin contexts; all Odin data-frame labels are `odin_odom` / `odin_map`; the TRANSFORM branch publishes `odin_map → odin_odom`.

- [ ] **Step 4: Build**

```bash
cd /home/yu/Codes_rk/SLAM && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select odin_ros_driver
```

Expected: builds clean.

- [ ] **Step 5: Commit**

```bash
cd /home/yu/Codes_rk && git add SLAM/src/odin_ros_driver/include/host_sdk_sample.h
git commit -m "feat(odin): odin_odom/odin_map frames, invert relocalization TF to odin_map->odin_odom"
```

---

### Task 2: sensorScanGeneration planning_frame

**Files:**
- Modify: `planner/src/sensor_scan_generation/src/sensorScanGeneration.cpp`

**Interfaces:**
- Consumes: `/state_estimation` (now `odin_odom`), `/registered_scan` (now `odin_odom`)
- Produces: `/state_estimation_at_scan` and `/sensor_scan` in `planning_frame` (`odin_odom`); TF `odin_odom → sensor_at_scan`.

- [ ] **Step 1: Add a global + planning_frame param**

Add a global near the top (after the `transformTfGeom` declaration block, ~line 42):

```cpp
// ################################
// C++: inherit planning frame instead of hardcoded map
// ################################
std::string planning_frame = "odin_odom";
```

In `main()`, before the subscriptions, declare+get the param:

```cpp
  nh->declare_parameter<std::string>("planning_frame", "odin_odom");
  nh->get_parameter("planning_frame", planning_frame);
```

- [ ] **Step 2: Replace the hardcoded `"map"` uses**

In `laserCloudAndOdometryHandler` (currently around lines 84-95):

```cpp
  odometryIn.header.frame_id = "map";          // → planning_frame
  odometryIn.child_frame_id = "sensor_at_scan";

  transformToMap.frame_id_ = "map";            // → planning_frame
  transformTfGeom = tf2::toMsg(transformToMap);
  transformTfGeom.header.stamp = laserCloud2->header.stamp;
  transformTfGeom.child_frame_id = "sensor_at_scan";
```

becomes:

```cpp
  odometryIn.header.frame_id = planning_frame;
  odometryIn.child_frame_id = "sensor_at_scan";

  transformToMap.frame_id_ = planning_frame;
  transformTfGeom = tf2::toMsg(transformToMap);
  transformTfGeom.header.stamp = laserCloud2->header.stamp;
  transformTfGeom.child_frame_id = "sensor_at_scan";
```

Do NOT change the transform math — `transformToMap` is built from the odometry pose, so the published `planning_frame → sensor_at_scan` transform is correct by construction. Optionally rename the variable `transformToMap` → `transformToPlanningFrame` (cosmetic, same math); if you rename it, update all three uses.

> **改 frame label 绝不能替代确认输入数值本身属于该 frame。** This node inverts `transformToMap` against every `/registered_scan` point, so the two inputs MUST genuinely share the planning frame — the check below enforces that.

- [ ] **Step 3: Add input frame-consistency check**

In `laserCloudAndOdometryHandler` (signature `(const nav_msgs::msg::Odometry::ConstSharedPtr odometry, const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)`), immediately at the top of the handler, add:

```cpp
  // ################################
  // C++: reject inputs not in the planning frame
  // ################################
  if (!odometry->header.frame_id.empty() &&
      odometry->header.frame_id != planning_frame) {
    RCLCPP_ERROR(rclcpp::get_logger("sensor_scan"),
      "[FRAME] /state_estimation frame '%s' != planning_frame '%s'; rejecting",
      odometry->header.frame_id.c_str(), planning_frame.c_str());
    return;
  }
  if (!laserCloud2->header.frame_id.empty() &&
      laserCloud2->header.frame_id != planning_frame) {
    RCLCPP_ERROR(rclcpp::get_logger("sensor_scan"),
      "[FRAME] /registered_scan frame '%s' != planning_frame '%s'; rejecting",
      laserCloud2->header.frame_id.c_str(), planning_frame.c_str());
    return;
  }
```

- [ ] **Step 4: Build**

```bash
cd /home/yu/Codes_rk/planner && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select sensor_scan_generation
```

- [ ] **Step 5: Commit**

```bash
cd /home/yu/Codes_rk && git add planner/src/sensor_scan_generation/src/sensorScanGeneration.cpp
git commit -m "feat(sensor_scan): planning_frame param, input frame-consistency check"
```

---

### Task 3: terrain frame-mismatch check + logs

**Files:**
- Modify: `planner/src/terrain_analysis/src/terrainAnalysis.cpp`
- Modify: `planner/src/terrain_analysis_ext/src/terrainAnalysisExt.cpp`

**Interfaces:**
- Consumes: `/state_estimation`, `/registered_scan`
- Produces: a `[FRAME]` mismatch guard so the two inputs are never silently computed in different frames.

- [ ] **Step 1: terrainAnalysis — mismatch check + first-data log**

The handlers in these files are **free functions** (no class), so only free-function logging is available (`rclcpp::get_logger(...)`); do NOT use `this->get_clock()`. The mismatch check fires only on actual disagreement (rare), so a non-throttled WARN is fine.

Add globals next to the existing `std::string laserCloudFrame = "map";`:

```cpp
  // ################################
  // C++: track /state_estimation frame and receipt for mismatch guard
  // ################################
  std::string odometry_frame_;
  bool odometry_received_ = false;
```

In `odometryHandler()` (subscribes `/state_estimation`, parameter name is **`odom`**), capture the frame and mark receipt at the top:

```cpp
  // ################################
  // C++: record state_estimation frame + receipt
  // ################################
  if (!odom->header.frame_id.empty()) {
    odometry_frame_ = odom->header.frame_id;
  }
  odometry_received_ = true;
```

In `laserCloudHandler()` (currently at ~line 149, parameter name **`laserCloud2`**), immediately after the existing `laserCloudTime = ...` and frame-inheritance lines, add — in this order:

```cpp
  // ################################
  // C++: require /state_estimation before processing cloud
  // ################################
  if (!odometry_received_) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("terrain_analysis"),
      rclcpp::Clock(), 3000,
      "[FRAME] no /state_estimation yet; ignoring registered_scan");
    return;
  }

  // ################################
  // C++: fail-fast when state_estimation and registered_scan frames disagree
  // ################################
  if (!odometry_frame_.empty() && !laserCloudFrame.empty() &&
      odometry_frame_ != laserCloudFrame) {
    RCLCPP_ERROR(rclcpp::get_logger("terrain_analysis"),
      "[FRAME] frame mismatch: state_estimation=%s but registered_scan=%s; rejecting cloud",
      odometry_frame_.c_str(), laserCloudFrame.c_str());
    return;
  }

  // ################################
  // C++: log one-time frame diagnostic once BOTH frames are known
  // ################################
  static bool frame_logged = false;
  if (!frame_logged && !odometry_frame_.empty() && !laserCloudFrame.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("terrain_analysis"),
      "[FRAME] state_estimation=%s registered_scan=%s",
      odometry_frame_.c_str(), laserCloudFrame.c_str());
    frame_logged = true;
  }
```

Notes:
- The mismatch check is **fail-fast**: it `return`s before any `point.x - vehicleX` arithmetic, so a wrong frame can never silently produce a bad terrain map.
- The `odometry_received_` guard prevents the initial `vehicleX=0`-based processing until a valid `/state_estimation` arrives.
- The one-time diagnostic prints only when BOTH frames are known (never a partial "first cloud only" print); `frame_logged` is set only in that branch.
- `rclcpp::Clock()` is a valid free-function throttle clock; these files have no class member clock.

- [ ] **Step 2: terrainAnalysisExt — same check**

Mirror Step 1 in `terrainAnalysisExt.cpp` using `rclcpp::get_logger("terrainAnalysisExt")` for the log names. Its `odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)` and `laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)` use the same parameter names — reuse the exact code above with `odom->` / `laserCloud2->`.

- [ ] **Step 3: Build**

```bash
cd /home/yu/Codes_rk/planner && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select terrain_analysis terrain_analysis_ext
```

- [ ] **Step 4: Commit**

```bash
cd /home/yu/Codes_rk && git add planner/src/terrain_analysis/src/terrainAnalysis.cpp planner/src/terrain_analysis_ext/src/terrainAnalysisExt.cpp
git commit -m "feat(terrain): frame-mismatch guard and [FRAME] first-data log"
```

---

### Task 4: cruiseController frame semantics + goal policy

**Files:**
- Modify: `planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `multi_frame` param (legacy `"odom"|"map"` normalized), RViz clicks, `/way_point_cruise`
- Produces: `planning_frame_`/`global_frame_` members + params; `transformGoalToPlanningFrame()`; goals always forwarded to localPlanner in `odin_odom`; the bare-`"map"` goal rejected.

- [ ] **Step 1: Add planning_frame / global_frame params + normalize multi_frame**

In the constructor, replace the current `multi_frame` declare/get/validate block (~lines 80-112):

```cpp
    this->declare_parameter<std::string>("multi_frame", "odin_odom");
    this->declare_parameter<std::string>("planning_frame", "odin_odom");
    this->declare_parameter<std::string>("global_frame", "odin_map");
    ...
    multi_frame_ = this->get_parameter("multi_frame").as_string();
    planning_frame_ = this->get_parameter("planning_frame").as_string();
    global_frame_ = this->get_parameter("global_frame").as_string();

    // ################################
    // C++: normalize legacy multi_frame values to odin_ namespace
    // ################################
    if (multi_frame_ == "odom") { multi_frame_ = "odin_odom"; }
    else if (multi_frame_ == "map") { multi_frame_ = "odin_map"; }
    if (multi_frame_ != "odin_odom" && multi_frame_ != "odin_map") {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] Invalid multi_frame='%s' (must be 'odin_odom' or 'odin_map')", multi_frame_.c_str());
      throw std::runtime_error("Invalid multi_frame");
    }
    RCLCPP_INFO(this->get_logger(),
      "[FRAME] planning_frame=%s global_frame=%s",
      planning_frame_.c_str(), global_frame_.c_str());
```

Add members in the private block (near `multi_frame_`):

```cpp
  std::string planning_frame_;
  std::string global_frame_;
```

- [ ] **Step 2: sendWaypointAndGo frame**

In `sendWaypointAndGo()` (~line 354), change `wp.header.frame_id = "map";` → `wp.header.frame_id = planning_frame_;`. Update the surrounding comment that says `hardcoded frame_id="map"`.

- [ ] **Step 3: transformToOdom frames**

In `transformToOdom()` (~line 708):

```cpp
      geometry_msgs::msg::TransformStamped t_map_odom =
        tf_buffer_.lookupTransform("odom", "map", tf2::TimePointZero);      // → (planning_frame_, global_frame_)
      geometry_msgs::msg::PointStamped map_pt;
      map_pt.header.frame_id = "map";                                       // → global_frame_
```

becomes:

```cpp
      geometry_msgs::msg::TransformStamped t_global_planning =
        tf_buffer_.lookupTransform(planning_frame_, global_frame_, tf2::TimePointZero);
      geometry_msgs::msg::PointStamped map_pt;
      map_pt.header.frame_id = global_frame_;
```

Keep the rest of the body and the `catch` unchanged (log message can keep the frames printed from the parameters).

- [ ] **Step 4: WAIT_LOCALIZATION and /multi_start TF gates**

Lines ~468-469 and ~1007-1008 both use:
```cpp
      if (multi_frame_ == "map" &&
          !tf_buffer_.canTransform("odom", "map", tf2::TimePointZero)) {
```
Change `multi_frame_ == "map"` → `multi_frame_ == "odin_map"` and `canTransform("odom", "map", ...)` → `canTransform(planning_frame_, global_frame_, ...)` in BOTH places.

- [ ] **Step 5: startNextWaypoint branch**

Lines ~868-874:
```cpp
    if (multi_frame_ == "odom") {
      gx_odom_ = w.x;
      gy_odom_ = w.y;
    } else {  // multi_frame_ == "map"
```
Change `"odom"` → `"odin_odom"` and the comment `// multi_frame_ == "map"` → `// multi_frame_ == "odin_map"`. The `transformToOdom` call inside is unchanged (its frames now come from Step 3's parameters).

- [ ] **Step 6: sendMultiWaypointAndGo frame**

In `sendMultiWaypointAndGo()` (~line 911), change `wp.header.frame_id = "odom";` → `wp.header.frame_id = planning_frame_;`. Update the surrounding comment.

- [ ] **Step 7: goal-frame policy + SINGLE/REPEAT waypointCallback**

Add a new policy function (place it next to `transformPointToFrame`):

```cpp
  // ################################
  // C++: resolve a received goal frame into the planning frame
  // ################################
  // Policy: empty -> assume planning frame (warn); planning frame -> copy;
  // global frame -> tf2 transform (reject if TF missing); motion-control
  // bare "map" -> reject; any other frame -> transform if possible, else reject.
  bool transformGoalToPlanningFrame(
    const geometry_msgs::msg::PointStamped & input,
    geometry_msgs::msg::PointStamped & output)
  {
    if (input.header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "[CRUISE] goal has empty frame; assuming %s (legacy compat, no transform)",
        planning_frame_.c_str());
      // Legacy compatibility: the coordinates are used as-is, but the output
      // MUST be explicitly tagged with the planning frame. This is an
      // assumption, NOT a coordinate transform.
      output = input;
      output.header.frame_id = planning_frame_;
      return true;
    }
    if (input.header.frame_id == planning_frame_) {
      RCLCPP_INFO(this->get_logger(),
        "[CRUISE] goal received frame=%s x=%.3f y=%.3f", planning_frame_.c_str(),
        input.point.x, input.point.y);
      RCLCPP_INFO(this->get_logger(),
        "[CRUISE] goal already in planning frame, no TF required");
      output = input;
      return true;
    }
    if (input.header.frame_id == "map") {
      RCLCPP_WARN(this->get_logger(),
        "[CRUISE] goal frame 'map' belongs to motion-control TF tree; use %s or %s",
        planning_frame_.c_str(), global_frame_.c_str());
      return false;
    }
    if (input.header.frame_id == global_frame_) {
      RCLCPP_INFO(this->get_logger(),
        "[CRUISE] goal received frame=%s x=%.3f y=%.3f", global_frame_.c_str(),
        input.point.x, input.point.y);
    }
    try {
      geometry_msgs::msg::TransformStamped t =
        tf_buffer_.lookupTransform(planning_frame_, input.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(input, output, t);
      RCLCPP_INFO(this->get_logger(),
        "[CRUISE] transformed %s -> %s: x=%.3f y=%.3f",
        input.header.frame_id.c_str(), planning_frame_.c_str(), output.point.x, output.point.y);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(),
        "[CRUISE] goal rejected: %s -> %s TF unavailable (%s)",
        input.header.frame_id.c_str(), planning_frame_.c_str(), e.what());
      return false;
    }
  }
```

In `waypointCallback()` (SINGLE/REPEAT, currently reads `msg->point.x/y` directly), insert the transform before using the coordinates:

```cpp
    // ################################
    // C++: resolve goal frame to planning frame before use
    // ################################
    geometry_msgs::msg::PointStamped goal;
    if (!transformGoalToPlanningFrame(*msg, goal)) {
      return;
    }
```
then replace the subsequent `msg->point.x` / `msg->point.y` reads with `goal.point.x` / `goal.point.y` (for both the repeat-retarget branch and the IDLE branch, and the start_x_/start_y_ anchors stay vehicle-local as before).

- [ ] **Step 8: unify MULTI waypoint addition across input channels**

**Current wiring (verified):** RViz `MultiWaypointTool` publishes `/multi_waypoint_add` → `addWaypointCallback()` → `waypoints_.push_back()`. The legacy RViz `WaypointTool` publishes `/way_point` which `system_real_robot.launch` remaps to `/way_point_cruise` (via `rvizWaypointTopic`); `waypointCallback()` currently **ignores** `/way_point_cruise` in MULTI mode (`[MULTI] Ignoring /way_point_cruise in MULTI mode`). The goal is to let BOTH tools add MULTI waypoints with one shared code path.

Extract the MULTI waypoint-addition logic into a shared helper (replaces the body of `addWaypointCallback`; the yaml/state/frame checks and the push logic move here verbatim):

```cpp
  // ################################
  // C++: unify MULTI waypoint addition for all input channels
  // ################################
  bool addMultiWaypoint(const geometry_msgs::msg::PointStamped & msg)
  {
    if (multi_source_ == "yaml") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[MULTI] Ignoring RViz waypoint because multi_source=yaml");
      return false;
    }
    if (state_ != CruiseState::COLLECTING_WAYPOINTS) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[MULTI] Route already started; RViz waypoint ignored");
      return false;
    }
    if (msg.header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "[MULTI] Waypoint has empty frame_id; ignored");
      return false;
    }
    // ################################
    // C++: reject waypoints in the motion-control map frame
    // ################################
    if (msg.header.frame_id == "map") {
      RCLCPP_WARN(this->get_logger(),
        "[MULTI] Waypoint frame 'map' belongs to motion-control TF tree; use %s or %s",
        planning_frame_.c_str(), global_frame_.c_str());
      return false;
    }
    geometry_msgs::msg::PointStamped converted;
    if (!transformPointToFrame(msg, multi_frame_, converted)) {
      RCLCPP_WARN(this->get_logger(),
        "[MULTI] Cannot add waypoint: TF %s -> %s unavailable",
        msg.header.frame_id.c_str(), multi_frame_.c_str());
      return false;
    }
    Waypoint w;
    w.x = converted.point.x;
    w.y = converted.point.y;
    w.turn_angle = 0.0;
    w.wait_time = default_wait_time_;
    waypoints_.push_back(w);
    publishMarkers();
    if (msg.header.frame_id == multi_frame_) {
      RCLCPP_INFO(this->get_logger(),
        "[MULTI] Added WP%zu at (%.3f, %.3f) in %s frame; %zu total",
        waypoints_.size() - 1, w.x, w.y, multi_frame_.c_str(), waypoints_.size());
    } else {
      RCLCPP_INFO(this->get_logger(),
        "[MULTI] Added WP%zu: input %s=(%.3f, %.3f) -> stored %s=(%.3f, %.3f); %zu total",
        waypoints_.size() - 1, msg.header.frame_id.c_str(), msg.point.x, msg.point.y,
        multi_frame_.c_str(), w.x, w.y, waypoints_.size());
    }
    return true;
  }
```

`addWaypointCallback()` becomes a thin wrapper:

```cpp
  void addWaypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    addMultiWaypoint(*msg);
  }
```

In `waypointCallback()`, replace the current MULTI-ignore block (`if (multi_enabled_) { WARN ... return; }`) with a route into the same helper:

```cpp
    // ################################
    // C++: route /way_point_cruise clicks into the MULTI list
    // ################################
    if (multi_enabled_) {
      addMultiWaypoint(*msg);
      return;
    }
```

Resulting MULTI input chain (both tools work):
```
RViz WaypointTool → /way_point → (launch remap) /way_point_cruise → waypointCallback → addMultiWaypoint → waypoints_
RViz MultiWaypointTool → /multi_waypoint_add → addWaypointCallback → addMultiWaypoint → waypoints_
```
`addMultiWaypoint` keeps `state_ != COLLECTING_WAYPOINTS` rejection, so clicks after cruise has started are still ignored. The SINGLE/REPEAT branch of `waypointCallback` (below the `multi_enabled_` block) is unchanged except for Step 7's `transformGoalToPlanningFrame`.

- [ ] **Step 9: Build**

```bash
cd /home/yu/Codes_rk/planner && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select local_planner
```

- [ ] **Step 10: Commit**

```bash
cd /home/yu/Codes_rk && git add planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): odin_ frame semantics, goal-frame policy, planning/global frame params"
```

---

### Task 5: legacy WaypointTool frame

**Files:**
- Modify: `planner/src/waypoint_rviz_plugin/src/waypoint_tool.cpp`

**Interfaces:**
- Consumes: RViz current Fixed Frame
- Produces: legacy `WaypointTool` publishes `/way_point` clicks with the RViz Fixed Frame (so no-map `odin_odom` / map-mode `odin_map` clicks carry the correct frame), without changing its `/joy` behavior.

- [ ] **Step 1: Use the Fixed Frame**

In `waypoint_tool.cpp` `onPoseSet()` (line ~82), change:

```cpp
  waypoint.header.frame_id = "map";
```
to:
```cpp
  // ################################
  // C++: publish waypoint in current RViz Fixed Frame
  // ################################
  waypoint.header.frame_id = context_->getFixedFrame().toStdString();
```

Everything else in `onPoseSet` (the fake `/joy` burst, `/way_point` topic, double publish) is unchanged.

- [ ] **Step 2: Build**

```bash
cd /home/yu/Codes_rk/planner && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select waypoint_rviz_plugin
```

- [ ] **Step 3: Commit**

```bash
cd /home/yu/Codes_rk && git add planner/src/waypoint_rviz_plugin/src/waypoint_tool.cpp
git commit -m "feat(rviz): WaypointTool publishes in RViz Fixed Frame, not hardcoded map"
```

---

### Task 6: launch / script / RViz defaults

**Files:**
- Modify: `planner/7multi.sh`, `planner/9multi_debug.sh`
- Modify: `planner/src/local_planner/launch/cruise.launch`
- Modify: `planner/src/vehicle_simulator/launch/system_real_robot.launch`
- Modify: `planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz`

**Interfaces:**
- Consumes: the `multi_frame` / `planning_frame` / `global_frame` node params (Task 4)
- Produces: all defaults / validation consistent with `odin_odom` / `odin_map`; `planning_frame`/`global_frame` reachable from the launch chain.

**Three-frame semantics (do not conflate):**
- `planning_frame` — the local planning / terrain / controller internal frame. Default `odin_odom`.
- `global_frame` — the map / relocalization frame. Default `odin_map`.
- `multi_frame` — the frame the MULTI waypoints belong to (no-map `odin_odom`, map-mode `odin_map`). `multi_frame` does NOT drive the RViz Fixed Frame.

- [ ] **Step 1: 7multi.sh**

- `frame=${3:-odom}` → `frame=${3:-odin_odom}`.
- Replace the frame-validation block so it accepts `odin_odom`/`odin_map` and normalizes legacy `odom`/`map`:
```bash
if [[ "$frame" == "odom" ]]; then frame=odin_odom; fi
if [[ "$frame" == "map" ]]; then frame=odin_map; fi
if [[ "$frame" != "odin_odom" && "$frame" != "odin_map" ]]; then
  echo "Error: frame must be 'odin_odom' or 'odin_map' (got '$frame'; legacy 'odom'/'map' accepted)"
  exit 1
fi
```
- Update the header comment block: default is `odin_odom`; RViz Fixed Frame is `odin_odom` (no-map) or `odin_map` (map mode); display/planning frames independent.

- [ ] **Step 2: 9multi_debug.sh**

Same as Step 1: `frame=${3:-odom}` → `frame=${3:-odin_odom}` + the normalization/validation block + usage text.

- [ ] **Step 3: cruise.launch — multi_frame default + planning/global args**

- `<arg name="multi_frame" default="odom"/>` → `<arg name="multi_frame" default="odin_odom"/>`.
- Add two new args (next to the `multi_*` args) and forward them to the node (next to the `multi_frame` param), so the Task 4 params are reachable from the launch chain:

```xml
  <arg name="planning_frame" default="odin_odom"/>
  <arg name="global_frame" default="odin_map"/>
  ...
    <param name="planning_frame" value="$(var planning_frame)" />
    <param name="global_frame" value="$(var global_frame)" />
```

- [ ] **Step 4: system_real_robot.launch**

- `declare_multi_frame = DeclareLaunchArgument('multi_frame', default_value='odom', description='Waypoint coordinate frame: odom|map')` → `default_value='odin_odom', description='Waypoint coordinate frame: odin_odom|odin_map'`.
- Add `planning_frame` / `global_frame` LaunchConfigurations + DeclareLaunchArguments (defaults `odin_odom` / `odin_map`) and forward them into the `start_cruise` `launch_arguments` dict alongside `multi_frame`, exactly mirroring how `multi_frame` is bound/declared/forwarded.

- [ ] **Step 5: vehicle_simulator.rviz — default + manual map-mode note**

- `Fixed Frame: odom` → `Fixed Frame: odin_odom`; update the nearby `# YAML:` marker comment to say `use odin_odom as shared RViz fixed frame`.
- **Do NOT claim `multi_frame` auto-switches RViz.** Add an explicit operator note in this step: no-map MULTI uses RViz Fixed Frame `odin_odom` (the default); **map-mode MULTI requires the operator to manually set RViz `Global Options → Fixed Frame` to `odin_map`** after launch. Automatic RViz-config switching is out of scope (optional future enhancement).

- [ ] **Step 6: Validate + build**

```bash
bash -n planner/7multi.sh && bash -n planner/9multi_debug.sh && echo "scripts OK"
python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('planner/src/local_planner/launch/cruise.launch'); print('cruise.launch OK')"
python3 -c "import ast; ast.parse(open('planner/src/vehicle_simulator/launch/system_real_robot.launch').read()); print('launch OK')"
python3 -c "import yaml; d=yaml.safe_load(open('planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz')); print('rviz Fixed Frame:', d['Visualization Manager']['Global Options']['Fixed Frame'])"
cd /home/yu/Codes_rk/planner && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select vehicle_simulator
```

- [ ] **Step 7: Commit**

```bash
cd /home/yu/Codes_rk && git add planner/7multi.sh planner/9multi_debug.sh \
  planner/src/local_planner/launch/cruise.launch \
  planner/src/vehicle_simulator/launch/system_real_robot.launch \
  planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz
git commit -m "feat(launch): odin_odom/odin_map frame defaults in launchers and RViz"
```

---

## Verification (end-to-end)

**No-map mode (Odin `custom_map_mode=0`):**
```bash
bash 7multi.sh    # rviz + odin_odom, no relocalization
# topics
ros2 topic echo /state_estimation --once --field header.frame_id      # odin_odom
ros2 topic echo /registered_scan --once --field header.frame_id       # odin_odom
ros2 topic echo /terrain_map --once --field header.frame_id           # odin_odom
# TF
ros2 run tf2_ros tf2_echo odin_odom odin1_base_link                   # must succeed
ros2 run tf2_ros tf2_echo odin_map odin_odom                          # NOT required (odin_map absent)
# RViz Fixed Frame = odin_odom; click goal → cruise log
#   [CRUISE] goal received frame=odin_odom ...
#   [CRUISE] goal already in planning frame, no TF required
# robot produces /path and /cmd_vel
```

**Map mode (Odin `custom_map_mode=2` + `.bin`):**
```bash
bash 7multi.sh rviz -1 odin_map    # or legacy: bash 7multi.sh rviz -1 map
ros2 run tf2_ros tf2_echo odin_map odin_odom                          # must succeed after relocalization
# RViz Fixed Frame = odin_map; click goal → cruise log
#   [CRUISE] goal received frame=odin_map ...
#   [CRUISE] transformed odin_map -> odin_odom: ...
```

**Negative checks:**
```bash
# A goal labeled bare "map" (motion-control) must be rejected:
ros2 topic pub /way_point_cruise geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 1.0, y: 1.0, z: 0.0}}" --once
#   cruise log: [CRUISE] goal frame 'map' belongs to motion-control TF tree; ...
# TF forest check — TWO unconnected trees are expected, not an error:
ros2 run tf2_tools view_frames
#   motion control: map -> base -> legs...
#   navigation:     odin_map -> odin_odom -> odin1_base_link
```

---

## Compatibility risks

1. **`multi_frame` legacy values** — `"odom"`/`"map"` are normalized to `odin_odom`/`odin_map` in both the node and the launchers, so existing `7multi.sh` invocations keep working; only the string stored/validated changes.
2. **RViz Fixed Frame default changes** `odom → odin_odom` — operators must have the new frame available (it is, once Odin publishes `odin_odom`). If an old rviz config is reused, the fixed frame must be updated.
3. **`/state_estimation`/`/registered_scan`/`/terrain_map` frame strings change** `odom → odin_odom` — any external consumer (scripts, logs, other tools) that matched the literal `"odom"` string must be updated; coordinate values are unchanged.
4. **`sensorScanGeneration` / `loamInterface`** — `sensor_scan_generation` now publishes `odin_odom`-framed output; `loamInterface` is untouched (legacy LOAM path, not launched by Odin). No functional regression in the Odin chain.
5. **Motion-control tree** — completely untouched; `map → base → legs…` remains as-is and unconnected to the Odin tree.

---

## Final data flows

**No-map mode (Odin `custom_map_mode=0`):**
```
odin_odom
└── odin1_base_link                       (Odin STANDARD TF)

/state_estimation     odin_odom
/registered_scan      odin_odom           (adapter inherits /odin1/cloud_slam header)
/terrain_map          odin_odom           (terrain inherits /registered_scan frame)
/sensor_scan          sensor_at_scan      (sensorScanGeneration; TF odin_odom -> sensor_at_scan)

goal → cruise/multi → localPlanner:       odin_odom (no TF; direct use)
localPlanner / terrainAnalysis arithmetic: odin_odom only
```
RViz Fixed Frame = `odin_odom`. No `odin_map`, no relocalization TF.

**Map mode (Odin `custom_map_mode=2` + `.bin`):**
```
odin_map
└── odin_odom
    ├── odin1_base_link
    └── sensor_at_scan

MULTI waypoint (RViz Fixed Frame = odin_map):   odin_map
    ↓ cruiseController (transformGoalToPlanningFrame / startNextWaypoint → transformToOdom)
local goal:                                      odin_odom
    ↓
localPlanner / terrainAnalysis:                  odin_odom only (terrain NEVER odin_map)
```

**Why terrain/localPlanner stay in `odin_odom`:** terrainAnalysis and localPlanner perform direct
`point.x - vehicleX` / distance arithmetic on raw values without per-point tf2 transforms. Feeding
`odin_map` values into those would silently corrupt the terrain/path computation. Only the
cruise/multi goal layer converts `odin_map → odin_odom`; the local planning chain is single-frame.

---

## Out of Scope but Required for Real-Robot Validation: Odin Timestamp Synchronization

The frame rename (`odom → odin_odom`, `map → odin_map`) and the topology fix solve TF
namespace/direction and waypoint-coordinate transforms ONLY. They do NOT address the
previously-observed mismatch between the Odin device time axis and the robot / ROS host time axis.

Before field validation, independently check that timestamps are aligned:

```bash
ros2 topic echo /odin1/odometry --once     # header.stamp vs host time
ros2 topic echo /registered_scan --once
ros2 topic echo /terrain_map --once
ros2 run tf2_ros tf2_echo odin_odom odin1_base_link
```

If `use_host_ros_time=2` is in use, verify that PTP / host-aligned timestamps actually take
effect separately. Do NOT modify the timestamp-sync implementation inside this plan's tasks —
none of the tasks cover it, and this plan is strictly about TF frame semantics.

---

## Acceptance Criteria

- **A. TF topology.** `ros2 run tf2_tools view_frames` shows:
  ```
  odin_map └── odin_odom └── odin1_base_link
  ```
  NOT `odin_odom ├── odin_map`.
- **B. No-map mode.** `ros2 run tf2_ros tf2_echo odin_odom odin1_base_link` succeeds; `odin_map → odin_odom` is NOT required. MULTI waypoints in `odin_odom`. `/state_estimation`, `/registered_scan`, `/terrain_map` all `frame_id: odin_odom`.
- **C. Map mode.** After relocalization, `ros2 run tf2_ros tf2_echo odin_map odin_odom` succeeds. MULTI waypoints can be `odin_map`; cruiseController forwards `odin_odom` goals to localPlanner; localPlanner/terrainAnalysis never consume `odin_map`.
- **D. Frame-mismatch rejection.** Deliberately publish a `/registered_scan` (or a waypoint) with a wrong frame: terrainAnalysis REJECTS (RCLCPP_ERROR, no output) rather than warns-and-continues; cruiseController REJECTS a goal whose TF to planning_frame is unavailable (never treats raw coords as planning-frame coords).
- **E. RViz MULTI waypoint.** With Fixed Frame `odin_odom`, clicks add to the MULTI list. With Fixed Frame `odin_map` (map mode), clicks add to the MULTI list, keep the `odin_map` frame, and after `/multi_start` are transformed `odin_map → odin_odom` by cruiseController.
- **F. Build.** At minimum: `odin_ros_driver` (SLAM ws); `terrain_analysis`, `terrain_analysis_ext`, `sensor_scan_generation`, `local_planner` (cruiseController), `waypoint_rviz_plugin`, `vehicle_simulator` (planner ws) — all build clean with `colcon build --symlink-install`.
