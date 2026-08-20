# Multi-Point Cruise Implementation Plan (rev. 2: multi_source / multi_frame split)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `multi_enabled` multi-point patrol to `cruiseController`: a closed-loop route of N waypoints, one of two exclusive sources (YAML file or RViz placement), each waypoint waits/turns, loops N rounds, RViz shows the route via MarkerArray. Waypoint **source** (`multi_source`) and coordinate **frame** (`multi_frame`) are independent concepts: the default `multi_frame=odom` works with NO prebuilt map and NO Odin relocalization — exactly like the field-verified `cruise`; `multi_frame=map` optionally enables prebuilt-map routes via Odin relocalization.

**Architecture:** Extend `cruiseController.cpp` with a waypoint queue (`waypoints_` + `waypoint_index_`), new states (`WAIT_LOCALIZATION`, `COLLECTING_WAYPOINTS`, `GO_TO_WAYPOINT`, `TURN_AT_WAYPOINT`, `WAIT_AT_WAYPOINT`), a `/multi_waypoints` MarkerArray publisher, `/multi_waypoint_add` subscription, and `/multi_start` Trigger service. `multi_frame` selects how waypoints enter the execution stage: `odom` → waypoint coordinates are used directly (no TF, no map dependency); `map` → each waypoint is converted map→odom via `lookupTransform` before driving. Both modes share the identical running state machine (GO_TO_WAYPOINT → TURN_AT_WAYPOINT → WAIT_AT_WAYPOINT → NEXT); only the coordinate handling at waypoint entry differs. `multi_enabled=false` (default) keeps SINGLE/REPEAT untouched. New deps: yaml-cpp, visualization_msgs, std_srvs, tf2/tf2_ros/tf2_geometry_msgs.

**Tech Stack:** C++14 (local_planner CMakeLists sets C++14 — do NOT change it), rclcpp, yaml-cpp, tf2, visualization_msgs, std_srvs, launch XML/Python.

**Revision note:** The MULTI implementation from rev. 1 (commits 48680d5..4fa5a88 on branch `multi`) hard-required the `odom↔map` TF in the WAIT_LOCALIZATION gate, `startNextWaypoint()`, and `/multi_start`, and hardcoded marker frame `"map"`. This revision removes that hard dependency: the tasks below modify that existing code in place — where a task says "replace the existing implementation", the code already exists on the branch in the map-TF form and must be changed to the new form shown.

## Global Constraints

- Branch: `multi` (fork of `cruise`).
- `multi_enabled=false` (default) MUST keep SINGLE/REPEAT behavior byte-for-byte identical.
- `multi_source` (`yaml` | `rviz`) and `multi_frame` (`odom` | `map`) are **INDEPENDENT**, both chosen at launch, both validated in the constructor (throw on invalid value).
- **`multi_frame=odom` is the DEFAULT and MUST NOT require**: `odom↔map` TF, Odin `custom_map_mode=2`, or a prebuilt `.bin` map. Only `/state_estimation` publishing is required. Waypoint coordinates are already odom-frame; NO map→odom conversion is performed.
- **`multi_frame=map` (optional enhancement)**: waypoint coordinates are map-frame; requires Odin relocalization so `odom↔map` TF exists. Each waypoint is converted map→odom via explicit `lookupTransform("odom","map", tf2::TimePointZero)` + `tf2::doTransform`, cached once per waypoint; on lookup failure, retry next tick (same retry contract as before).
- `WAIT_LOCALIZATION` gate: `has_odom_` is always required; `canTransform("odom","map")` is additionally required ONLY when `multi_frame_=="map"`. (State name stays `WAIT_LOCALIZATION` — renaming to `WAIT_POSE` deferred to keep the diff minimal.)
- `/multi_start` gate (rviz mode): `waypoints_.size()>=2 && has_odom_` in odom mode; additionally `canTransform("odom","map")` in map mode.
- `/multi_waypoints` markers are published with `header.frame_id = multi_frame_` (NOT hardcoded `"map"`).
- `/multi_waypoint_add` clicks MUST have `msg->header.frame_id == multi_frame_`; on mismatch the waypoint is REJECTED with a throttled WARN that names the expected frame (v1: no TF conversion of clicks). Ship RViz Fixed Frame as `odom`; in map mode the operator switches RViz Fixed Frame to `map` before clicking (documented in Task 9).
- YAML route coordinates carry NO frame field — their frame is `multi_frame_` by construction (`multi_route.yaml` values are odom coordinates in odom mode, map coordinates in map mode).
- `loop_count` counts **complete round-trips that physically drive WPN→WP0, arrive at WP0, AND finish WP0's own turn/wait** — then and only then `completed_loops_++`. `loop_count=1` must end parked at WP0.
- Every waypoint arrival publishes `/stop=2` (seize `/cmd_vel`) BEFORE any turn/wait; released only on next `/way_point`. The self-published `/stop=2` must be preceded by `ignore_next_internal_stop_ = true` (via `seizeControlForMulti()`), and `turning_internal_` must NOT be set — so an external `/stop=2` aborts immediately in any MULTI state.
- `/multi_waypoints` MarkerArray QoS = `rclcpp::QoS(10).reliable().transient_local()`.
- `default_wait_time` is a node param (2.0 s), not YAML-only.
- `multi_route_file` default = `ament_index_cpp::get_package_share_directory("local_planner") + "/config/multi_route.yaml"`.
- CMake `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`.
- Do NOT modify `localPlanner.cpp`, `pathFollower.cpp` travel logic, or terrain packages.
- `7multi.sh` forces `repeat_enabled:=false`; usage `bash 7multi.sh [yaml|rviz] [loop_count] [odom|map]` with `mode=${1:-yaml} loop_count=${2:--1} frame=${3:-odom}`.
- **MULTI publishes `/way_point` with `frame_id="odom"`** via a NEW `sendMultiWaypointAndGo()`; the existing `sendWaypointAndGo()` (hardcoded `frame_id="map"`) is left untouched for SINGLE/REPEAT.
- `multi_enabled_ && repeat_enabled_` is an **invalid combination** — node validates and refuses to start.
- `waypointCallback()` ignores `/way_point_cruise` entirely when `multi_enabled_`.
- `completed_loops_` and `loop_count_` members ALREADY EXIST (from REPEAT) — MULTI reuses them, does NOT redeclare.
- `markers_pub_` is created BEFORE any `publishMarkers()` call.
- All build paths in this plan are **repo-relative** (run from the workspace root), not absolute dev-machine paths.

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `planner/src/local_planner/src/cruiseController.cpp` | Modify | MULTI state machine, queue, `multi_frame` coordinate handling, TF (map mode), YAML, markers, service |
| `planner/src/local_planner/launch/cruise.launch` | Modify | Pass `multi_*` + `default_wait_time` + `multi_frame` params |
| `planner/src/vehicle_simulator/launch/system_real_robot.launch` | Modify | Forward `multi_*` + `default_wait_time` + `multi_frame` args |
| `planner/src/local_planner/CMakeLists.txt` | Modify | New deps + config install |
| `planner/src/local_planner/package.xml` | Modify | New deps |
| `planner/src/local_planner/config/multi_route.yaml` | Create | Default YAML route (coordinates in `multi_frame_`) |
| `planner/7multi.sh` | Modify | Executable launcher with optional 3rd param `frame` |
| `planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz` | Modify | Fixed Frame → `odom` (MarkerArray display + PublishPoint tool already present) |

---

### Task 1: Dependencies (CMake + package.xml + config dir)

**Files:**
- Modify: `planner/src/local_planner/CMakeLists.txt`
- Modify: `planner/src/local_planner/package.xml`
- Create: `planner/src/local_planner/config/multi_route.yaml`

**Interfaces:**
- Consumes: nothing
- Produces: `find_package` for yaml-cpp/visualization_msgs/std_srvs/ament_index_cpp; `ament_target_dependencies(cruiseController ...)` extended (yaml-cpp NOT among them); `target_link_libraries(cruiseController yaml-cpp)`; `install(DIRECTORY config ...)`; `<depend>` entries; default YAML file. Used by Tasks 2-9.

- [ ] **Step 1: CMakeLists — find_package**

Add after the existing `find_package` block (near line 28):

```cmake
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
```

`tf2`, `tf2_ros`, `tf2_geometry_msgs` are already found by `localPlanner`/`pathFollower`, so no new find needed — but confirm they are in the file. If not, add them.

- [ ] **Step 2: CMakeLists — cruiseController deps**

```cmake
ament_target_dependencies(cruiseController rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs ament_index_cpp)
```

**`yaml-cpp` goes ONLY in `target_link_libraries`, NOT in `ament_target_dependencies`** (it is a plain system lib, not an ament package). Add:

```cmake
target_link_libraries(cruiseController yaml-cpp)
```

- [ ] **Step 3: CMakeLists — install config dir**

After the existing `install(DIRECTORY launch paths ...)` block, add:

```cmake
install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
```

- [ ] **Step 4: package.xml — deps**

Add after the existing `<depend>` entries:

```xml
  <depend>yaml-cpp</depend>
  <depend>visualization_msgs</depend>
  <depend>std_srvs</depend>
  <depend>ament_index_cpp</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
```

- [ ] **Step 5: Default YAML route**

Create `planner/src/local_planner/config/multi_route.yaml`:

```yaml
multi_cruise:
  waypoints:
    - x: 1.0
      y: 0.0
    - x: 5.0
      y: 0.0
    - x: 9.0
      y: 0.0
      turn_angle: 180.0
```

The coordinates have NO frame field — they are interpreted in `multi_frame_` (odom coordinates in odom mode, map coordinates in map mode).

- [ ] **Step 6: Build**

```bash
cd planner
colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean (new deps resolve).

- [ ] **Step 7: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add planner/src/local_planner/CMakeLists.txt \
        planner/src/local_planner/package.xml \
        planner/src/local_planner/config/multi_route.yaml
git commit -m "feat(multi): add yaml-cpp/visualization_msgs/std_srvs/tf2 deps, config install, default route"
```

---

### Task 2: Waypoint struct + new params (incl. multi_frame) + MULTI state enum

**Files:**
- Modify: `planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: nothing
- Produces: `struct Waypoint { double x,y; double turn_angle=0.0; double wait_time; };`
  `enum class CruiseState` extended with `WAIT_LOCALIZATION`, `COLLECTING_WAYPOINTS`,
  `GO_TO_WAYPOINT`, `TURN_AT_WAYPOINT`, `WAIT_AT_WAYPOINT` (append after existing 5 states).
  Member fields used by Tasks 3-9: `std::vector<Waypoint> waypoints_;`, `size_t waypoint_index_=0;`,
  `bool closing_loop_=false;`, `bool multi_enabled_=false;`,
  `std::string multi_source_="yaml";`, `std::string multi_frame_="odom";`,
  `std::string multi_route_file_;`, `double default_wait_time_=2.0;`,
  `double gx_odom_=0.0, gy_odom_=0.0;`, `bool active_goal_valid_=false;`, `double wait_start_time_=0.0;`.
  **`completed_loops_` and `loop_count_` already exist from REPEAT — REUSE, do NOT redeclare.**

**Current code (already on branch `multi`):** struct, enum extensions, params, and members exist from rev. 1 WITHOUT `multi_frame`. This task adds `multi_frame` to the param reads/validation and to the members; the rest is unchanged.

- [ ] **Step 1: Add `#include`s and struct**

Add at top (after existing includes):

```cpp
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
```

After the `CruiseState` enum, add:

```cpp
struct Waypoint {
  double x, y;
  double turn_angle = 0.0;
  double wait_time = 2.0;
};
```

- [ ] **Step 2: Extend CruiseState enum**

Append to the existing enum:

```cpp
enum class CruiseState {
  IDLE = 0,
  GO_TO_DEST = 1,
  TURN_AT_DEST = 2,
  RETURN_TO_START = 3,
  TURN_AT_START = 4,
  WAIT_LOCALIZATION = 5,
  COLLECTING_WAYPOINTS = 6,
  GO_TO_WAYPOINT = 7,
  TURN_AT_WAYPOINT = 8,
  WAIT_AT_WAYPOINT = 9,
};
```

- [ ] **Step 3: Declare + read new params**

In the constructor, add `multi_frame` to the declares:

```cpp
    this->declare_parameter<bool>("multi_enabled", false);
    this->declare_parameter<std::string>("multi_source", "yaml");
    this->declare_parameter<std::string>("multi_frame", "odom");
    this->declare_parameter<std::string>("multi_route_file", "");
    this->declare_parameter<double>("default_wait_time", 2.0);
```

After the existing gets, add the `multi_frame` get + validation:

```cpp
    multi_enabled_ = this->get_parameter("multi_enabled").as_bool();
    multi_source_ = this->get_parameter("multi_source").as_string();
    multi_frame_ = this->get_parameter("multi_frame").as_string();
    std::string mrf = this->get_parameter("multi_route_file").as_string();
    multi_route_file_ = mrf.empty()
      ? ament_index_cpp::get_package_share_directory("local_planner") + "/config/multi_route.yaml"
      : mrf;
    default_wait_time_ = this->get_parameter("default_wait_time").as_double();

    // Validate multi_source
    if (multi_source_ != "yaml" && multi_source_ != "rviz") {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] Invalid multi_source='%s' (must be 'yaml' or 'rviz')", multi_source_.c_str());
      throw std::runtime_error("Invalid multi_source");
    }

    // Validate multi_frame
    if (multi_frame_ != "odom" && multi_frame_ != "map") {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] Invalid multi_frame='%s' (must be 'odom' or 'map')", multi_frame_.c_str());
      throw std::runtime_error("Invalid multi_frame");
    }

    // multi + repeat is an invalid combination
    if (multi_enabled_ && repeat_enabled_) {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] multi_enabled && repeat_enabled is invalid; refusing to start");
      throw std::runtime_error("multi_enabled && repeat_enabled");
    }
```

- [ ] **Step 4: Add member declarations**

Add to the private member block (after existing members):

```cpp
  // MULTI members
  std::vector<Waypoint> waypoints_;
  size_t waypoint_index_ = 0;
  // completed_loops_ and loop_count_ ALREADY EXIST from REPEAT — reuse, do NOT redeclare
  bool closing_loop_ = false;
  bool multi_enabled_ = false;
  std::string multi_source_;
  std::string multi_frame_;
  std::string multi_route_file_;
  double default_wait_time_ = 2.0;
  double gx_odom_ = 0.0, gy_odom_ = 0.0;
  bool active_goal_valid_ = false;
  double wait_start_time_ = 0.0;
```

- [ ] **Step 5: waypointCallback ignores /way_point_cruise when multi_enabled**

Add at the top of `waypointCallback()` (after the `has_odom_` guard):

```cpp
    if (multi_enabled_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[MULTI] Ignoring /way_point_cruise in MULTI mode");
      return;
    }
```

- [ ] **Step 6: Extend `stateName()` for MULTI states**

In the existing `stateName(CruiseState s)` switch, add the five new cases:

```cpp
      case CruiseState::WAIT_LOCALIZATION: return "WAIT_LOCALIZATION";
      case CruiseState::COLLECTING_WAYPOINTS: return "COLLECTING_WAYPOINTS";
      case CruiseState::GO_TO_WAYPOINT: return "GO_TO_WAYPOINT";
      case CruiseState::TURN_AT_WAYPOINT: return "TURN_AT_WAYPOINT";
      case CruiseState::WAIT_AT_WAYPOINT: return "WAIT_AT_WAYPOINT";
```

- [ ] **Step 7: Build**

```bash
cd planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 8: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): add multi_frame param (odom|map, default odom)"
```

---

### Task 3: tf2 buffer + MULTI setup + frame-aware waypoint entry

**Files:**
- Modify: `planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `multi_enabled_`, `multi_source_`, `multi_frame_`, `multi_route_file_`, `default_wait_time_`,
  `Waypoint`, new states, `has_odom_` (existing)
- Produces: `tf_buffer_`/`tf_listener_` members; `loadYaml()`, `publishMarkers()` (frame-aware),
  `transformToOdom()` (map mode only), `beginMultiCruise()`, `startNextWaypoint()` (frame-aware),
  `sendMultiWaypointAndGo()`, `addWaypointCallback()` (frame-checked), `startService()` (frame-aware gate).
  Used by Tasks 4-9.

**Current code (already on branch `multi`):** all of these exist from rev. 1. The changes in this task:
(1) `publishMarkers()` frame_id `"map"` → `multi_frame_`; (2) `startNextWaypoint()` unconditional
`transformToOdom()` → `multi_frame_`-dependent branch; (3) `addWaypointCallback()` gains a frame check;
(4) `startService()` TF check becomes conditional on `multi_frame_=="map"`. The rest is unchanged.

- [ ] **Step 1: Add tf2 members + init**

Add members:

```cpp
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
```

Add `tf_buffer_`/`tf_listener_` to the constructor's member-init list. `tf_buffer_`
must be initialized before `tf_listener_` (listener needs a live buffer). Insert
BOTH at the very start of the init list, before `state_`:

```cpp
  CruiseController()
  : Node("cruise_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    state_(CruiseState::IDLE),
    ...
```

- [ ] **Step 2: Add MULTI setup in constructor**

**IMPORTANT ORDER**: this MULTI setup block must come AFTER `markers_pub_` is
created (Step 7's `create_publisher`). In the constructor, place the
`markers_pub_` creation immediately before this block. (`publishMarkers()` is
invoked from the WAIT_LOCALIZATION gate in Task 4 and from `addWaypointCallback`
— all at runtime, after construction — but `loadYaml()` here must not run before
`markers_pub_` exists in case a later refactor calls `publishMarkers()` at setup.)

After the `/stop` subscription block, add MULTI setup:

```cpp
    // MULTI mode setup — MUST be after markers_pub_ creation (Step 7)
    // BOTH sources enter WAIT_LOCALIZATION first; Task 4's gate then dispatches
    // to beginMultiCruise() (yaml) or COLLECTING_WAYPOINTS (rviz).
    // The gate requires /state_estimation always, and the relocalization TF
    // only when multi_frame_=="map".
    if (multi_enabled_) {
      if (multi_source_ == "yaml") {
        loadYaml();
        if (waypoints_.size() < 2) {
          RCLCPP_ERROR(this->get_logger(),
            "[MULTI] multi_route.yaml has <2 waypoints (%zu); staying IDLE",
            waypoints_.size());
        }
      } else {
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] RViz mode: waiting for pose, then collect waypoints");
      }
      state_ = CruiseState::WAIT_LOCALIZATION;
      publishMarkers();
    }
```

- [ ] **Step 3: Implement `loadYaml()`**

```cpp
  void loadYaml()
  {
    try {
      YAML::Node root = YAML::LoadFile(multi_route_file_);
      YAML::Node wps = root["multi_cruise"]["waypoints"];
      for (const auto & wp : wps) {
        Waypoint w;
        w.x = wp["x"].as<double>();
        w.y = wp["y"].as<double>();
        w.turn_angle = wp["turn_angle"] ? wp["turn_angle"].as<double>() : 0.0;
        w.wait_time = wp["wait_time"] ? wp["wait_time"].as<double>() : default_wait_time_;
        waypoints_.push_back(w);
      }
      RCLCPP_INFO(this->get_logger(),
        "[MULTI] Loaded %zu waypoints from %s (frame=%s)",
        waypoints_.size(), multi_route_file_.c_str(), multi_frame_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] Failed to load %s: %s", multi_route_file_.c_str(), e.what());
    }
  }
```

- [ ] **Step 4: Implement `publishMarkers()` — frame follows multi_frame**

Replace the existing `publishMarkers()` (its header lambda hardcodes `"map"`).
Keep the existing marker comment `C++: publish MULTI route and waypoint markers`
(rule: reuse existing marker — purpose unchanged); only the header lambda changes:

```cpp
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    auto header = [this]() {
      std_msgs::msg::Header h;
      h.frame_id = multi_frame_;
      h.stamp = this->now();
      return h;
    };

    // LINE_STRIP connecting all waypoints (closed loop: last→first)
    visualization_msgs::msg::Marker line;
    line.header = header();
    line.ns = "route";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.05;
    line.color.r = 1.0f; line.color.g = 1.0f; line.color.b = 0.0f; line.color.a = 1.0f;
    line.pose.orientation.w = 1.0;
    for (const auto & w : waypoints_) {
      geometry_msgs::msg::Point p;
      p.x = w.x; p.y = w.y; p.z = 0.0;
      line.points.push_back(p);
    }
    if (waypoints_.size() >= 2) {
      geometry_msgs::msg::Point p0;
      p0.x = waypoints_[0].x; p0.y = waypoints_[0].y; p0.z = 0.0;
      line.points.push_back(p0);  // close the loop
    }
    arr.markers.push_back(line);

    for (size_t i = 0; i < waypoints_.size(); i++) {
      // SPHERE
      visualization_msgs::msg::Marker sphere;
      sphere.header = header();
      sphere.ns = "wp_sphere";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = waypoints_[i].x;
      sphere.pose.position.y = waypoints_[i].y;
      sphere.pose.position.z = 0.0;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.4; sphere.scale.y = 0.4; sphere.scale.z = 0.4;
      // highlight current
      bool is_current = (multi_enabled_ && state_ == CruiseState::GO_TO_WAYPOINT &&
                         i == waypoint_index_);
      if (is_current) {
        sphere.color.r = 0.0f; sphere.color.g = 1.0f; sphere.color.b = 0.0f;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.7;
      } else {
        sphere.color.r = 1.0f; sphere.color.g = 1.0f; sphere.color.b = 1.0f;
      }
      sphere.color.a = 1.0f;
      arr.markers.push_back(sphere);

      // TEXT_VIEW_FACING
      visualization_msgs::msg::Marker text;
      text.header = header();
      text.ns = "wp_text";
      text.id = static_cast<int>(i);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = waypoints_[i].x;
      text.pose.position.y = waypoints_[i].y;
      text.pose.position.z = 0.6;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.5;
      text.color.r = 0.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
      text.text = "WP" + std::to_string(i);
      arr.markers.push_back(text);
    }

    markers_pub_->publish(arr);
  }
```

- [ ] **Step 5: Implement `transformToOdom()` — used ONLY when `multi_frame_=="map"`**

```cpp
  bool transformToOdom(double mx, double my, double & ox, double & oy)
  {
    try {
      // Explicit lookup + doTransform (NOT buffer_.transform), and do NOT set
      // header.stamp to tf2::TimePointZero — use the lookup's own timepoint.
      geometry_msgs::msg::TransformStamped t_map_odom =
        tf_buffer_.lookupTransform("odom", "map", tf2::TimePointZero);

      geometry_msgs::msg::PointStamped map_pt;
      map_pt.header.frame_id = "map";
      map_pt.point.x = mx;
      map_pt.point.y = my;
      map_pt.point.z = 0.0;

      geometry_msgs::msg::PointStamped odom_pt;
      tf2::doTransform(map_pt, odom_pt, t_map_odom);
      ox = odom_pt.point.x;
      oy = odom_pt.point.y;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[MULTI] TF transform failed: %s", e.what());
      return false;
    }
  }
```

- [ ] **Step 6: Implement `beginMultiCruise()` + frame-aware `startNextWaypoint()`**

```cpp
  void beginMultiCruise()
  {
    waypoint_index_ = 0;
    completed_loops_ = 0;
    closing_loop_ = false;
    RCLCPP_INFO(this->get_logger(),
      "[MULTI] Cruise started: %zu waypoints, loop_count=%d, frame=%s",
      waypoints_.size(), loop_count_, multi_frame_.c_str());
    startNextWaypoint();
  }

  // Falls under the existing marker `C++: implement MULTI cruise start sequence`
  // (rule: reuse existing marker — purpose unchanged); only the body changes.
  void startNextWaypoint()
  {
    if (waypoint_index_ >= waypoints_.size()) {
      RCLCPP_ERROR(this->get_logger(), "[MULTI] waypoint_index_ out of range");
      state_ = CruiseState::IDLE;
      return;
    }
    // Enter GO_TO_WAYPOINT FIRST and clear active_goal_valid_ BEFORE any
    // coordinate work, so that if map-mode transform fails the controlLoop
    // retries next tick (GO_TO_WAYPOINT sees active_goal_valid_==false and
    // calls startNextWaypoint()).
    state_ = CruiseState::GO_TO_WAYPOINT;
    active_goal_valid_ = false;

    const Waypoint & w = waypoints_[waypoint_index_];
    if (multi_frame_ == "odom") {
      // odom mode (default): waypoint coordinates ARE odom coordinates.
      // No TF, no map, no relocalization required.
      gx_odom_ = w.x;
      gy_odom_ = w.y;
    } else {  // multi_frame_ == "map"
      if (!transformToOdom(w.x, w.y, gx_odom_, gy_odom_)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[MULTI] Cannot transform WP%zu to odom; retrying next tick", waypoint_index_);
        return;  // stay in GO_TO_WAYPOINT, retry on next tick
      }
    }
    active_goal_valid_ = true;
    RCLCPP_INFO(this->get_logger(),
      "[MULTI] Going to WP%zu: (%s frame) (%.3f, %.3f) -> odom (%.3f, %.3f)",
      waypoint_index_, multi_frame_.c_str(), w.x, w.y, gx_odom_, gy_odom_);
    sendMultiWaypointAndGo(gx_odom_, gy_odom_);
    publishMarkers();
  }

  // MULTI publishes /way_point with frame_id="odom" (unlike SINGLE/REPEAT's
  // sendWaypointAndGo which hardcodes frame_id="map"). SINGLE/REPEAT untouched.
  void sendMultiWaypointAndGo(double x, double y)
  {
    geometry_msgs::msg::PointStamped wp;
    wp.header.stamp = this->now();
    wp.header.frame_id = "odom";
    wp.point.x = x;
    wp.point.y = y;
    wp.point.z = 0.0;
    waypoint_pub_->publish(wp);

    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 0;
    stop_pub_->publish(stop_msg);

    RCLCPP_INFO(this->get_logger(),
      "[MULTI][WAYPOINT] publish /way_point (odom): x=%.3f, y=%.3f", x, y);
  }
```

- [ ] **Step 7: Add `markers_pub_` and `/multi_start`/`/multi_waypoint_add` in constructor**

Add members:

```cpp
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr add_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
```

In constructor, add (only when `multi_enabled_`):

```cpp
    if (multi_enabled_) {
      markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/multi_waypoints", rclcpp::QoS(10).reliable().transient_local());
      add_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/multi_waypoint_add", 10,
        std::bind(&CruiseController::addWaypointCallback, this, std::placeholders::_1));
      start_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/multi_start",
        std::bind(&CruiseController::startService, this, std::placeholders::_1, std::placeholders::_2));
    }
```

- [ ] **Step 8: Implement `addWaypointCallback()` (frame-checked) + `startService()` (frame-aware gate)**

```cpp
  void addWaypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (multi_source_ == "yaml") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[MULTI] Ignoring RViz waypoint because multi_source=yaml");
      return;
    }
    if (state_ != CruiseState::COLLECTING_WAYPOINTS) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[MULTI] Route already started; RViz waypoint ignored");
      return;
    }
    // This check is added under the existing marker `C++: handle RViz waypoint
    // add and start service` (rule: reuse existing marker — purpose unchanged).
    // v1: no TF conversion of clicks — the click frame must equal
    // multi_frame_ (RViz Fixed Frame must be set to multi_frame_).
    if (msg->header.frame_id != multi_frame_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[MULTI] Waypoint frame '%s' != multi_frame '%s'; set RViz Fixed Frame to '%s' and re-click",
        msg->header.frame_id.c_str(), multi_frame_.c_str(), multi_frame_.c_str());
      return;
    }
    Waypoint w;
    w.x = msg->point.x;
    w.y = msg->point.y;
    w.turn_angle = 0.0;
    w.wait_time = default_wait_time_;
    waypoints_.push_back(w);
    publishMarkers();
    RCLCPP_INFO(this->get_logger(),
      "[MULTI] Added WP%zu at (%.3f, %.3f) in %s frame; %zu total",
      waypoints_.size() - 1, w.x, w.y, multi_frame_.c_str(), waypoints_.size());
  }

  void startService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    // /multi_start is only valid in rviz mode + COLLECTING_WAYPOINTS state
    if (multi_source_ != "rviz" || state_ != CruiseState::COLLECTING_WAYPOINTS) {
      res->success = false;
      res->message = "/multi_start only valid in rviz COLLECTING_WAYPOINTS";
      RCLCPP_WARN(this->get_logger(), "[MULTI] %s", res->message.c_str());
      return;
    }
    if (waypoints_.size() < 2) {
      res->success = false;
      res->message = "At least 2 waypoints are required";
      RCLCPP_WARN(this->get_logger(), "[MULTI] %s", res->message.c_str());
      return;
    }
    if (!has_odom_) {
      res->success = false;
      res->message = "/state_estimation not available";
      RCLCPP_WARN(this->get_logger(), "[MULTI] %s", res->message.c_str());
      return;
    }
    // ################################
    // C++: gate /multi_start on relocalization TF only in map mode
    // ################################
    // odom mode starts with /state_estimation only; map mode additionally
    // needs the relocalization TF (re-checked at start time).
    if (multi_frame_ == "map" &&
        !tf_buffer_.canTransform("odom", "map", tf2::TimePointZero)) {
      res->success = false;
      res->message = "odom->map TF not available";
      RCLCPP_WARN(this->get_logger(), "[MULTI] %s", res->message.c_str());
      return;
    }
    res->success = true;
    res->message = "Starting multi cruise";
    RCLCPP_INFO(this->get_logger(), "[MULTI] /multi_start accepted; starting cruise");
    beginMultiCruise();
  }
```

- [ ] **Step 9: Build**

```bash
cd planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 10: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): frame-aware waypoint entry, markers in multi_frame, frame-checked RViz clicks"
```

---

### Task 4: WAIT_LOCALIZATION gating + multi dispatch in controlLoop

**Files:**
- Modify: `planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `multi_enabled_`, `has_odom_`, `multi_frame_`, `tf_buffer_`, `state_`
- Produces: controlLoop dispatch — when `multi_enabled_`, `WAIT_LOCALIZATION` gates
  YAML auto-start / RViz collection. **odom mode: only `has_odom_` required;
  map mode: additionally `canTransform("odom","map")`.**

**Current code (already on branch `multi`):** the gate requires
`has_odom_ && tf_buffer_.canTransform(...)` unconditionally. Replace it with the
frame-dependent gate below.

- [ ] **Step 1: Gate at top of controlLoop**

Add at the top of `controlLoop()` (before the existing switch):

```cpp
    // Existing marker `C++: MULTI localization gate dispatch` stays (rule:
    // reuse existing marker — purpose unchanged); the condition changes.
    // MULTI: pose gate — odom mode needs only /state_estimation (no map,
    // no relocalization); map mode additionally waits for the relocalization TF.
    if (multi_enabled_ && state_ == CruiseState::WAIT_LOCALIZATION) {
      if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[MULTI] Waiting for /state_estimation...");
        return;
      }
      if (multi_frame_ == "map" &&
          !tf_buffer_.canTransform("odom", "map", tf2::TimePointZero)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[MULTI] Waiting for odom->map TF (map mode)...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
        "[MULTI] Pose ready (frame=%s), proceeding", multi_frame_.c_str());
      if (multi_source_ == "yaml") {
        if (waypoints_.size() >= 2) {
          beginMultiCruise();
        } else {
          state_ = CruiseState::IDLE;
        }
      } else {
        state_ = CruiseState::COLLECTING_WAYPOINTS;
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] RViz mode: click waypoints (RViz Fixed Frame must be '%s'), then call /multi_start",
          multi_frame_.c_str());
        publishMarkers();
      }
      return;
    }
```

(The constructor's initial `WAIT_LOCALIZATION` state + `publishMarkers()` is already
set in Task 3 Step 2 — do NOT duplicate it here. This task only adds the gate in
Step 1, which dispatches that initial state at runtime.)

- [ ] **Step 2: Build**

```bash
cd planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 3: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): WAIT_LOCALIZATION gate requires relocalization TF only in map mode"
```

---

### Task 5: GO_TO_WAYPOINT arrival + /stop=2 seize + TURN_AT_WAYPOINT + WAIT_AT_WAYPOINT

**Files:**
- Modify: `planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `gx_odom_/gy_odom_`, `active_goal_valid_`, `waypoint_index_`, `closing_loop_`,
  `goal_clear_range` param, `wait_start_time_`, `default_wait_time_`, `publishTurnCmd()` (existing),
  `turnDone()` (existing), `publishZeroCmd()` (existing)
- Produces: the three MULTI running states in `controlLoop()`. Consumed by Task 6 (`advanceWaypoint`).
  **Unchanged from rev. 1 — these states consume the already-frame-resolved `gx_odom_/gy_odom_`.**
  Both modes share this identical running state machine (user requirement 十).

**Current code (already on branch `multi`):** this task is already fully implemented and
needs NO changes. Verify it is present, then commit nothing new — the task is a no-op
verification pass (build + confirm the cases exist).

- [ ] **Step 1: Verify the three running-state cases exist**

Confirm in `controlLoop()` the `GO_TO_WAYPOINT`, `TURN_AT_WAYPOINT`, `WAIT_AT_WAYPOINT`
cases are present exactly as in rev. 1 (arrival → `seizeControlForMulti()` + `publishZeroCmd()` →
turn (if `w.turn_angle != 0`) or wait → loop counting at WP0 → `advanceWaypoint()`).
No edits needed.

- [ ] **Step 2: Build**

```bash
cd planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 3: Commit (none — no changes; proceed)**

Skip the commit. Mark this task complete only after the build confirms the
existing code still compiles.

---

### Task 6: advanceWaypoint + loop_count semantics

**Files:**
- Modify: `planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `waypoint_index_`, `waypoints_`, `closing_loop_`
- Produces: `advanceWaypoint()` — increments index, wraps with `closing_loop_`.
  **Loop counting is NOT here** — it lives in Task 5's `WAIT_AT_WAYPOINT`
  completion (arrival at WP0 + wait done), per the closed-loop semantics.

**Current code (already on branch `multi`):** fully implemented, no changes needed.

- [ ] **Step 1: Verify `advanceWaypoint()`**

```cpp
  void advanceWaypoint()
  {
    waypoint_index_++;
    if (waypoint_index_ >= waypoints_.size()) {
      waypoint_index_ = 0;
      closing_loop_ = true;
    }
    startNextWaypoint();
  }
```

> **Note on loop semantics**: one round is credited ONLY when the robot has
> physically driven WPN→WP0, arrived at WP0 (`GO_TO_WAYPOINT` arrival),
> completed WP0's turn/wait (`WAIT_AT_WAYPOINT`), and then the completion
> handler sees `waypoint_index_==0 && closing_loop_==true`. `advanceWaypoint()`
> merely wraps the index and sets `closing_loop_`. `loop_count=1` must end
> parked at WP0.

- [ ] **Step 2: Build**

```bash
cd planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 3: Commit (none — no changes; proceed)**

Skip the commit.

---

### Task 7: /stop handling in MULTI mode (reuse turning flags)

**Files:**
- Modify: `planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: existing `stopCallback()`, `turning_internal_`, `ignore_next_internal_stop_`,
  `pending_stop_`; MULTI states
- Produces: MULTI mode subscribes `/stop`; stopCallback aborts to `IDLE` in any
  MULTI state (reusing the flag mechanism so internal `/stop=2` isn't mistaken for external).

**Current code (already on branch `multi`):** fully implemented, no changes needed.

- [ ] **Step 1: Verify `/stop` subscription + stopCallback**

Confirm the constructor subscribes `/stop` when `multi_enabled_` (in addition to
`repeat_enabled_`), and that `stopCallback()` resets `completed_loops_`,
`closing_loop_`, `pending_stop_`, `active_goal_valid_` and sets `state_ = IDLE`
on external stop. No edits needed.

- [ ] **Step 2: Build**

```bash
cd planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 3: Commit (none — no changes; proceed)**

Skip the commit.

---

### Task 8: Launch args (cruise.launch + system_real_robot.launch) + multi_frame

**Files:**
- Modify: `planner/src/local_planner/launch/cruise.launch`
- Modify: `planner/src/vehicle_simulator/launch/system_real_robot.launch`

**Interfaces:**
- Consumes: nothing
- Produces: `multi_enabled`, `multi_source`, `multi_frame`, `multi_route_file`, `default_wait_time`
  launch args forwarded to the node (consumed by Task 2 params).

**Current code (already on branch `multi`):** the four rev. 1 args exist in both
launch files. This task adds `multi_frame` (default `odom`) to both.

- [ ] **Step 1: cruise.launch — add `multi_frame` arg + param**

In `cruise.launch`, add the `multi_frame` arg after `multi_source`:

```xml
  <arg name="multi_source" default="yaml"/>
  <arg name="multi_frame" default="odom"/>
  <arg name="multi_route_file" default=""/>
  <arg name="default_wait_time" default="2.0"/>
```

And the param in the node block:

```xml
    <param name="multi_source" value="$(var multi_source)" />
    <param name="multi_frame" value="$(var multi_frame)" />
    <param name="multi_route_file" value="$(var multi_route_file)" />
    <param name="default_wait_time" value="$(var default_wait_time)" />
```

- [ ] **Step 2: Validate cruise.launch XML**

```bash
python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('planner/src/local_planner/launch/cruise.launch'); print('XML OK')"
```

- [ ] **Step 3: system_real_robot.launch — bind + declare + forward `multi_frame`**

Add the LaunchConfiguration after `multi_source`:

```python
  multi_frame = LaunchConfiguration('multi_frame')
```

Add the declare after `declare_multi_source`:

```python
  declare_multi_frame = DeclareLaunchArgument('multi_frame', default_value='odom', description='Waypoint coordinate frame: odom|map')
```

Add to the `start_cruise` `launch_arguments` (after `multi_source`):

```python
      'multi_frame': multi_frame,
```

Add to `ld.add_action` after `declare_multi_source`:

```python
  ld.add_action(declare_multi_frame)
```

- [ ] **Step 4: Validate system_real_robot.launch**

```bash
python3 -c "import ast; ast.parse(open('planner/src/vehicle_simulator/launch/system_real_robot.launch').read()); print('launch OK')"
```

- [ ] **Step 5: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add planner/src/local_planner/launch/cruise.launch \
        planner/src/vehicle_simulator/launch/system_real_robot.launch
git commit -m "feat(multi): forward multi_frame launch arg (odom|map, default odom)"
```

---

### Task 9: 7multi.sh frame param + RViz Fixed Frame

**Files:**
- Modify: `planner/7multi.sh`
- Modify: `planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz`

**Interfaces:**
- Consumes: `system_real_robot.launch` args (`enableCruise`, `multi_enabled`, `multi_source`,
  `multi_frame`, `repeat_enabled`, `loop_count`, `rvizWaypointTopic`)
- Produces: runnable launcher with optional 3rd param `frame`; RViz Fixed Frame `odom`
  so the DEFAULT odom mode works with no map TF (MarkerArray display + PublishPoint
  tool are already present from rev. 1).

- [ ] **Step 1: Rewrite 7multi.sh with `frame` param**

```bash
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# ################################
# Bash: launch multi-point cruise with yaml/rviz source and odom/map frame
# ################################
mode=${1:-yaml}
loop_count=${2:--1}
frame=${3:-odom}

if [[ "$mode" != "yaml" && "$mode" != "rviz" ]]; then
  echo "Usage: bash 7multi.sh [yaml|rviz] [loop_count] [odom|map]"
  echo "Examples:"
  echo "  bash 7multi.sh yaml        # YAML route, odom frame (no map needed)"
  echo "  bash 7multi.sh yaml 3      # YAML route, odom frame, 3 loops"
  echo "  bash 7multi.sh rviz        # RViz clicks, odom frame (no map needed)"
  echo "  bash 7multi.sh rviz 3      # RViz clicks, odom frame, 3 loops"
  echo "  bash 7multi.sh yaml -1 map # YAML route in prebuilt map frame (needs Odin relocalization)"
  echo "  bash 7multi.sh rviz -1 map # RViz clicks in map frame (needs Odin relocalization)"
  exit 1
fi

if [[ "$frame" != "odom" && "$frame" != "map" ]]; then
  echo "Error: frame must be 'odom' or 'map' (got '$frame')"
  exit 1
fi

ros2 launch vehicle_simulator system_real_robot.launch \
  enableCruise:=true \
  multi_enabled:=true \
  repeat_enabled:=false \
  multi_source:=$mode \
  multi_frame:=$frame \
  loop_count:=$loop_count \
  rvizWaypointTopic:=/way_point_cruise \
  2>&1 | grep --line-buffered -E 'MULTI|CRUISE|WAYPOINT|WARN|ERROR'
```

- [ ] **Step 2: Make executable + syntax check**

```bash
chmod +x planner/7multi.sh
bash -n planner/7multi.sh && echo "syntax OK"
```

- [ ] **Step 3: RViz config — Fixed Frame → odom**

In `vehicle_simulator.rviz`, change the root Fixed Frame from `map` to `odom`
(currently ~line 520):

```yaml
    Fixed Frame: odom
```

**Why:** in odom mode (the default) there is NO `map` frame published, so a Fixed
Frame of `map` would make RViz unable to display anything and Publish Point clicks
would arrive in the wrong frame and be rejected by the node's frame check. `odom`
is always published by `/state_estimation`, so it works in both modes for DISPLAY.
**Map-mode operational note (document in the script header/README):** in
`multi_frame=map` mode the operator must set RViz Fixed Frame to `map` before
clicking waypoints — clicks are otherwise published in `odom` and rejected
(throttled WARN names the expected frame).

- [ ] **Step 4: Validate rviz file is still parseable YAML**

```bash
python3 -c "import yaml; yaml.safe_load(open('planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz')); print('rviz YAML OK')"
```

- [ ] **Step 5: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add planner/7multi.sh planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz
git commit -m "feat(multi): 7multi.sh frame param (odom|map); RViz Fixed Frame odom"
```

---

## Verification (end-to-end)

**Scenario A — no-map local multi-point cruise (DEFAULT, multi_frame=odom):**

1. **Regression** — `multi_enabled=false` (default): `./3cruise.sh` and `./5repeat180.sh`
   behave unchanged.
2. **YAML + odom, no map** — Odin runs without relocalization (`custom_map_mode=0`, no
   `.bin` map, no `odom↔map` TF). `./7multi.sh yaml 1` auto-starts WP0→WP1→WP2 from the
   default route (odom-frame coordinates), waits 2 s each, one closed loop then stops at
   WP0; `/multi_waypoints` shows spheres, WP labels, closed-loop line in the odom frame.
3. **RViz + odom, no map** — `./7multi.sh rviz`: robot holds `COLLECTING_WAYPOINTS` once
   `/state_estimation` arrives (no TF wait — verify no "Waiting for odom->map TF" log);
   click 3 points via Publish Point (RViz Fixed Frame is `odom`); markers appear
   incrementally; `ros2 service call /multi_start std_srvs/srv/Trigger "{}"` starts
   cruise. With <2 points the service rejects. A click in the wrong frame is rejected
   with the throttled WARN.
4. **Loop / stop** — `loop_count=3` does 3 closed loops; `ros2 topic pub /stop
   std_msgs/msg/Int8 "{data: 2}" --once` aborts in any state (mid-drive/turn/wait).

**Scenario B — prebuilt-map global multi-point cruise (optional, multi_frame=map):**

5. **YAML + map** — Odin relocalization on (`custom_map_mode=2`, `.bin` map, `odom↔map`
   TF present). `./7multi.sh yaml -1 map`: YAML waypoints interpreted as map-frame,
   TF-converted to odom per waypoint, route runs in the local planner.
6. **RViz + map** — `./7multi.sh rviz -1 map`: set RViz Fixed Frame to `map` before
   clicking; clicks accepted (frame matches); markers displayed in the map frame.
   Without the relocalization TF, cruise never starts: WAIT_LOCALIZATION holds with a
   throttled WARN, and `/multi_start` rejects with "odom->map TF not available".

**Both modes (shared running state machine):**

7. **Source exclusivity** — yaml mode ignores `/multi_waypoint_add` (throttled warn);
   rviz mode ignores YAML.
8. **Late RViz** — start RViz after the node; `/multi_waypoints` still visible
   (transient_local latched).
9. **Build** — full `colcon build --symlink-install --packages-select local_planner vehicle_simulator`.

## Out of Scope

- Runtime route editing after cruise starts (route frozen).
- YAML + RViz mixed sources.
- TF conversion of mismatched RViz click frames (v1 rejects + WARN; RViz Fixed Frame must match `multi_frame` during collection).
- Pause/resume, auto-start on boot, multi-route switching.
- Changes to `localPlanner` / `pathFollower` travel logic or terrain stack.
- Renaming `WAIT_LOCALIZATION` → `WAIT_POSE`.
