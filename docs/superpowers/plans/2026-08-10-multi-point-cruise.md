# Multi-Point Cruise Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `multi_enabled` multi-point patrol to `cruiseController`: a closed-loop route of N waypoints, one of two exclusive sources (YAML file or RViz placement), each waypoint waits/turns, loops N rounds, RViz shows the route via MarkerArray.

**Architecture:** Extend `cruiseController.cpp` with a waypoint queue (`waypoints_` + `waypoint_index_`), new states (`WAIT_LOCALIZATION`, `COLLECTING_WAYPOINTS`, `GO_TO_WAYPOINT`, `TURN_AT_WAYPOINT`, `WAIT_AT_WAYPOINT`), a `/multi_waypoints` MarkerArray publisher, `/multi_waypoint_add` subscription, and `/multi_start` Trigger service. `multi_enabled=false` (default) keeps SINGLE/REPEAT untouched. New deps: yaml-cpp, visualization_msgs, std_srvs, tf2/tf2_ros/tf2_geometry_msgs.

**Tech Stack:** C++14 (local_planner CMakeLists sets C++14 — do NOT change it), rclcpp, yaml-cpp, tf2, visualization_msgs, std_srvs, launch XML/Python.

## Global Constraints

- Branch: `multi` (fork of `cruise`).
- `multi_enabled=false` (default) MUST keep SINGLE/REPEAT behavior byte-for-byte identical.
- Waypoint source is EXCLUSIVE, chosen at launch via `multi_source` (`yaml` | `rviz`), never merged.
- `loop_count` counts **complete round-trips that physically drive WPN→WP0, arrive at WP0, AND finish WP0's own turn/wait** — then and only then `completed_loops_++`. `loop_count=1` must end parked at WP0.
- Every waypoint arrival publishes `/stop=2` (seize `/cmd_vel`) BEFORE any turn/wait; released only on next `/way_point`. The self-published `/stop=2` must be preceded by `ignore_next_internal_stop_ = true` (via `seizeControlForMulti()`), and `turning_internal_` must NOT be set — so an external `/stop=2` aborts immediately in any MULTI state.
- `active_goal_odom_` = current waypoint transformed map→odom via explicit `lookupTransform("odom","map", tf2::TimePointZero)` + `tf2::doTransform`, cached once per waypoint; arrival check uses `/state_estimation` (odom frame) vs `active_goal_odom_`.
- `WAIT_LOCALIZATION` gate: cruise starts only when `has_odom_` AND `canTransform("odom","map")` both true.
- `/multi_waypoints` MarkerArray QoS = `rclcpp::QoS(10).reliable().transient_local()`.
- `default_wait_time` is a node param (2.0 s), not YAML-only.
- `multi_route_file` default = `ament_index_cpp::get_package_share_directory("local_planner") + "/config/multi_route.yaml"`.
- CMake `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`.
- Do NOT modify `localPlanner.cpp`, `pathFollower.cpp` travel logic, or terrain packages.
- `7multi.sh` forces `repeat_enabled:=false`.
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
| `cmu_planner/src/local_planner/src/cruiseController.cpp` | Modify | MULTI state machine, queue, TF, YAML, markers, service |
| `cmu_planner/src/local_planner/launch/cruise.launch` | Modify | Pass `multi_*` + `default_wait_time` params |
| `cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch` | Modify | Forward `multi_*` + `default_wait_time` args |
| `cmu_planner/src/local_planner/CMakeLists.txt` | Modify | New deps + config install |
| `cmu_planner/src/local_planner/package.xml` | Modify | New deps |
| `cmu_planner/src/local_planner/config/multi_route.yaml` | Create | Default YAML route |
| `cmu_planner/7multi.sh` | Create | Executable launcher |
| `cmu_planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz` | Modify | MarkerArray display + PublishPoint tool |

---

### Task 1: Dependencies (CMake + package.xml + config dir)

**Files:**
- Modify: `cmu_planner/src/local_planner/CMakeLists.txt`
- Modify: `cmu_planner/src/local_planner/package.xml`
- Create: `cmu_planner/src/local_planner/config/multi_route.yaml`

**Interfaces:**
- Consumes: nothing
- Produces: `find_package` for yaml-cpp/visualization_msgs/std_srvs/ament_index_cpp/tf2/tf2_ros/tf2_geometry_msgs; `ament_target_dependencies(cruiseController ...)` extended (yaml-cpp NOT among them); `target_link_libraries(cruiseController yaml-cpp)`; `install(DIRECTORY config ...)`; `<depend>` entries; default YAML file. Used by Task 2-9.

- [ ] **Step 1: CMakeLists — find_package**

Add after the existing `find_package` block (near line 28):

```cmake
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
```

`tf2`, `tf2_ros`, `tf2_geometry_msgs` are already found by `localPlanner`/`pathFollower` (lines 21-23), so no new find needed — but confirm they are in the file. If not, add them.

- [ ] **Step 2: CMakeLists — cruiseController deps**

Replace line 36:

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

Create `cmu_planner/src/local_planner/config/multi_route.yaml`:

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

- [ ] **Step 6: Build**

```bash
cd cmu_planner
colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean (new deps resolve).

- [ ] **Step 7: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/src/local_planner/CMakeLists.txt \
        cmu_planner/src/local_planner/package.xml \
        cmu_planner/src/local_planner/config/multi_route.yaml
git commit -m "feat(multi): add yaml-cpp/visualization_msgs/std_srvs/tf2 deps, config install, default route"
```

---

### Task 2: Waypoint struct + new params + MULTI state enum

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: nothing
- Produces: `struct Waypoint { double x,y; double turn_angle=0.0; double wait_time; };`
  `enum class CruiseState` extended with `WAIT_LOCALIZATION`, `COLLECTING_WAYPOINTS`,
  `GO_TO_WAYPOINT`, `TURN_AT_WAYPOINT`, `WAIT_AT_WAYPOINT` (append after existing 5 states).
  Member fields used by Tasks 3-9: `std::vector<Waypoint> waypoints_;`, `size_t waypoint_index_=0;`,
  `bool closing_loop_=false;`, `bool multi_enabled_=false;`,
  `std::string multi_source_="yaml";`, `std::string multi_route_file_;`, `double default_wait_time_=2.0;`,
  `double gx_odom_=0.0, gy_odom_=0.0;`, `bool active_goal_valid_=false;`, `double wait_start_time_=0.0;`.
  **`completed_loops_` and `loop_count_` already exist from REPEAT — REUSE, do NOT redeclare.**

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

In the constructor, after existing declares (line 37), add:

```cpp
    this->declare_parameter<bool>("multi_enabled", false);
    this->declare_parameter<std::string>("multi_source", "yaml");
    this->declare_parameter<std::string>("multi_route_file", "");
    this->declare_parameter<double>("default_wait_time", 2.0);
```

After the existing gets (line 42), add:

```cpp
    multi_enabled_ = this->get_parameter("multi_enabled").as_bool();
    multi_source_ = this->get_parameter("multi_source").as_string();
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
cd cmu_planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 8: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): Waypoint struct, MULTI params, extended state enum"
```

---

### Task 3: tf2 buffer + WAIT_LOCALIZATION gate + waypoint entry

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `multi_enabled_`, `multi_source_`, `multi_route_file_`, `default_wait_time_`,
  `Waypoint`, new states, `has_odom_` (existing)
- Produces: `tf_buffer_`/`tf_listener_` members; `loadYaml()`, `publishMarkers()`,
  `transformToOdom()`, `beginMultiCruise()`; `WAIT_LOCALIZATION` gating in `controlLoop()`.
  Used by Tasks 4-9.

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
          "[MULTI] RViz mode: waiting for localization, then collect waypoints");
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
        "[MULTI] Loaded %zu waypoints from %s", waypoints_.size(), multi_route_file_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] Failed to load %s: %s", multi_route_file_.c_str(), e.what());
    }
  }
```

- [ ] **Step 4: Implement `publishMarkers()`**

```cpp
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    auto header = [this]() {
      std_msgs::msg::Header h;
      h.frame_id = "map";
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

- [ ] **Step 5: Implement `transformToOdom()`**

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

- [ ] **Step 6: Implement `beginMultiCruise()`**

```cpp
  void beginMultiCruise()
  {
    waypoint_index_ = 0;
    completed_loops_ = 0;
    closing_loop_ = false;
    RCLCPP_INFO(this->get_logger(),
      "[MULTI] Cruise started: %zu waypoints, loop_count=%d",
      waypoints_.size(), loop_count_);
    startNextWaypoint();
  }

  void startNextWaypoint()
  {
    if (waypoint_index_ >= waypoints_.size()) {
      RCLCPP_ERROR(this->get_logger(), "[MULTI] waypoint_index_ out of range");
      state_ = CruiseState::IDLE;
      return;
    }
    // Enter GO_TO_WAYPOINT FIRST and clear active_goal_valid_ BEFORE the TF
    // transform, so that if lookup fails the controlLoop retries next tick
    // (GO_TO_WAYPOINT sees active_goal_valid_==false and calls startNextWaypoint()).
    state_ = CruiseState::GO_TO_WAYPOINT;
    active_goal_valid_ = false;

    const Waypoint & w = waypoints_[waypoint_index_];
    if (!transformToOdom(w.x, w.y, gx_odom_, gy_odom_)) {
      RCLCPP_WARN(this->get_logger(),
        "[MULTI] Cannot transform WP%zu to odom; retrying next tick", waypoint_index_);
      return;  // stay in GO_TO_WAYPOINT, retry on next tick
    }
    active_goal_valid_ = true;
    RCLCPP_INFO(this->get_logger(),
      "[MULTI] Going to WP%zu: map=(%.3f, %.3f) odom=(%.3f, %.3f)",
      waypoint_index_, w.x, w.y, gx_odom_, gy_odom_);
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

- [ ] **Step 8: Implement `addWaypointCallback()` + `startService()`**

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
    Waypoint w;
    w.x = msg->point.x;
    w.y = msg->point.y;
    w.turn_angle = 0.0;
    w.wait_time = default_wait_time_;
    waypoints_.push_back(w);
    publishMarkers();
    RCLCPP_INFO(this->get_logger(),
      "[MULTI] Added WP%zu at (%.3f, %.3f); %zu total",
      waypoints_.size() - 1, w.x, w.y, waypoints_.size());
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
    // Re-check localization TF before starting
    if (!tf_buffer_.canTransform("odom", "map", tf2::TimePointZero)) {
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
cd cmu_planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 10: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): tf2 buffer, WAIT_LOCALIZATION gate, loadYaml/publishMarkers, /multi_waypoint_add + /multi_start"
```

---

### Task 4: WAIT_LOCALIZATION gating + multi dispatch in controlLoop

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `multi_enabled_`, `has_odom_`, `tf_buffer_`, `state_`
- Produces: controlLoop dispatch — when `multi_enabled_`, `WAIT_LOCALIZATION` gates
  YAML auto-start / RViz collection; `GO_TO_WAYPOINT` handles arrival.

- [ ] **Step 1: Gate at top of controlLoop**

Add at the top of `controlLoop()` (before the existing switch):

```cpp
    // MULTI: localization gate
    if (multi_enabled_ && state_ == CruiseState::WAIT_LOCALIZATION) {
      if (has_odom_ && tf_buffer_.canTransform("odom", "map", tf2::TimePointZero)) {
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] Localization ready (odom→map TF available), proceeding");
        if (multi_source_ == "yaml") {
          if (waypoints_.size() >= 2) {
            beginMultiCruise();
          } else {
            state_ = CruiseState::IDLE;
          }
        } else {
          state_ = CruiseState::COLLECTING_WAYPOINTS;
          RCLCPP_INFO(this->get_logger(),
            "[MULTI] RViz mode: click waypoints, then call /multi_start");
          publishMarkers();
        }
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[MULTI] Waiting for localization/TF...");
      }
      return;
    }
```

(The constructor's initial `WAIT_LOCALIZATION` state + `publishMarkers()` is already
set in Task 3 Step 2 — do NOT duplicate it here. This task only adds the gate in
Step 1, which dispatches that initial state at runtime.)

- [ ] **Step 2: Build**

```bash
cd cmu_planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 3: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): WAIT_LOCALIZATION gate dispatch (yaml auto-start / rviz collect)"
```

---

### Task 5: GO_TO_WAYPOINT arrival + /stop=2 seize + TURN_AT_WAYPOINT + WAIT_AT_WAYPOINT

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `gx_odom_/gy_odom_`, `active_goal_valid_`, `waypoint_index_`, `closing_loop_`,
  `goal_clear_range` param, `wait_start_time_`, `default_wait_time_`, `publishTurnCmd()` (existing),
  `turnDone()` (existing), `publishZeroCmd()` (existing)
- Produces: the three MULTI running states in `controlLoop()`. Consumed by Task 6 (`advanceWaypoint`).

- [ ] **Step 1: Add `seizeControlForMulti()` helper**

Insert before `publishTurnCmd()`:

```cpp
  // Publish /stop=2 to seize /cmd_vel from pathFollower, but FIRST set
  // ignore_next_internal_stop_ so the node's own /stop=2 is not mistaken
  // for an external stop. turning_internal_ is deliberately NOT set, so an
  // external /stop=2 still aborts immediately in any MULTI state.
  void seizeControlForMulti()
  {
    ignore_next_internal_stop_ = true;
    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 2;
    stop_pub_->publish(stop_msg);
  }
```

- [ ] **Step 2: Add GO_TO_WAYPOINT / TURN_AT_WAYPOINT / WAIT_AT_WAYPOINT cases**

Add to the `controlLoop()` switch (before the default/end):

```cpp
    case CruiseState::GO_TO_WAYPOINT: {
      if (!active_goal_valid_) {
        startNextWaypoint();
        return;
      }
      double dx = current_x_ - gx_odom_;
      double dy = current_y_ - gy_odom_;
      double goal_clear_range = this->get_parameter("goal_clear_range").as_double();
      if (dx * dx + dy * dy < goal_clear_range * goal_clear_range) {
        active_goal_valid_ = false;
        // seize /cmd_vel BEFORE any turn/wait; publishZeroCmd to hold still
        seizeControlForMulti();
        publishZeroCmd();
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] Reached WP%zu", waypoint_index_);

        const Waypoint & w = waypoints_[waypoint_index_];
        if (fabs(w.turn_angle) > 1e-3) {
          RCLCPP_INFO(this->get_logger(),
            "[MULTI] Turning %.0f deg at WP%zu", w.turn_angle, waypoint_index_);
          target_yaw_ = normalizeAngle(current_yaw_ + w.turn_angle * M_PI / 180.0);
          state_ = CruiseState::TURN_AT_WAYPOINT;
        } else {
          wait_start_time_ = this->now().seconds();
          state_ = CruiseState::WAIT_AT_WAYPOINT;
        }
      }
      return;
    }

    case CruiseState::TURN_AT_WAYPOINT: {
      if (turnDone()) {
        publishZeroCmd();
        wait_start_time_ = this->now().seconds();
        state_ = CruiseState::WAIT_AT_WAYPOINT;
        RCLCPP_INFO(this->get_logger(), "[MULTI] Turn done at WP%zu", waypoint_index_);
      } else {
        publishTurnCmd();
      }
      return;
    }

    case CruiseState::WAIT_AT_WAYPOINT: {
      const Waypoint & w = waypoints_[waypoint_index_];
      double elapsed = this->now().seconds() - wait_start_time_;
      if (elapsed >= w.wait_time) {
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] Wait done at WP%zu (%.1fs), advancing", waypoint_index_, w.wait_time);

        // ################################
        // C++: count a closed loop only after arriving at WP0 and finishing its wait
        // ################################
        // One round = physically drive WPN→WP0, arrive at WP0, and finish WP0's
        // turn/wait. Only then completed_loops_++. loop_count=1 must end parked at WP0.
        if (waypoint_index_ == 0 && closing_loop_) {
          completed_loops_++;
          bool done = (loop_count_ > 0 && completed_loops_ >= loop_count_);
          RCLCPP_INFO(this->get_logger(),
            "[MULTI] Loop %d/%s complete at WP0", completed_loops_,
            (loop_count_ > 0 ? std::to_string(loop_count_).c_str() : "inf"));
          if (done) {
            publishZeroCmd();
            RCLCPP_INFO(this->get_logger(),
              "[MULTI] Cruise complete after %d loops", completed_loops_);
            completed_loops_ = 0;
            closing_loop_ = false;
            state_ = CruiseState::IDLE;
            return;
          }
          closing_loop_ = false;
        }

        advanceWaypoint();
      } else {
        publishZeroCmd();  // keep robot still; pathFollower already stopped via /stop=2
      }
      return;
    }
```

- [ ] **Step 2: Build**

```bash
cd cmu_planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 3: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): GO_TO_WAYPOINT arrival, /stop=2 seize, TURN_AT_WAYPOINT, WAIT_AT_WAYPOINT"
```

---

### Task 6: advanceWaypoint + loop_count semantics

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `waypoint_index_`, `waypoints_`, `closing_loop_`
- Produces: `advanceWaypoint()` — increments index, wraps with `closing_loop_`.
  **Loop counting is NOT here** — it lives in Task 5's `WAIT_AT_WAYPOINT`
  completion (arrival at WP0 + wait done), per the closed-loop semantics.

- [ ] **Step 1: Implement `advanceWaypoint()` (index + closing_loop_ only, NO counting)**

```cpp
  // Advance the index. When wrapping past the last waypoint, set
  // closing_loop_=true so the arriving-at-WP0 logic (Task 5's WAIT_AT_WAYPOINT
  // completion) knows this WP0 arrival closes a round. This function does NOT
  // count loops — counting happens only after the robot physically arrives at
  // WP0 AND finishes WP0's turn/wait (see Task 5 Step 3).
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
cd cmu_planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 3: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): advanceWaypoint with closing_loop_ loop-count semantics"
```

---

### Task 7: /stop handling in MULTI mode (reuse turning flags)

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: existing `stopCallback()`, `turning_internal_`, `ignore_next_internal_stop_`,
  `pending_stop_`; MULTI states
- Produces: MULTI mode subscribes `/stop`; stopCallback aborts to `IDLE` in any
  MULTI state (reusing the flag mechanism so internal `/stop=2` isn't mistaken for external).

- [ ] **Step 1: Subscribe /stop for MULTI too**

In the constructor, change the existing `/stop` subscription guard so it also
activates in MULTI mode:

```cpp
    if (this->get_parameter("repeat_enabled").as_bool() ||
        this->get_parameter("multi_enabled").as_bool()) {
      stop_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/stop", 10,
        std::bind(&CruiseController::stopCallback, this, std::placeholders::_1));
    }
```

- [ ] **Step 2: Generalize stopCallback for MULTI**

The existing `stopCallback()` already: consumes `ignore_next_internal_stop_`,
queues `pending_stop_` during `turning_internal_`, else `publishZeroCmd()` + resets
counters + `state_ = IDLE`. This is correct for MULTI. Add MULTI-specific reset:

```cpp
    // (existing body)
    publishZeroCmd();
    completed_loops_ = 0;
    closing_loop_ = false;
    pending_stop_ = false;
    active_goal_valid_ = false;
    RCLCPP_WARN(this->get_logger(),
      "[MULTI] Stop received, cruise aborted");
    state_ = CruiseState::IDLE;
```

(Keep the `turning_internal_` / `ignore_next_internal_stop_` handling unchanged.)

- [ ] **Step 3: Build**

```bash
cd cmu_planner && colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 4: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(multi): /stop abort in MULTI mode (reuse turning flag mechanism)"
```

---

### Task 8: Launch args (cruise.launch + system_real_robot.launch)

**Files:**
- Modify: `cmu_planner/src/local_planner/launch/cruise.launch`
- Modify: `cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch`

**Interfaces:**
- Consumes: nothing
- Produces: `multi_enabled`, `multi_source`, `multi_route_file`, `default_wait_time`
  launch args forwarded to the node (consumed by Task 2 params).

- [ ] **Step 1: cruise.launch — add args**

Replace `cruise.launch` with (add the four new args + params):

```xml
<launch>

  <arg name="max_yaw_rate" default="45.0"/>
  <arg name="min_yaw_rate" default="0.32"/>
  <arg name="yaw_kp" default="1.5"/>
  <arg name="yaw_tolerance" default="0.12"/>
  <arg name="goal_clear_range" default="0.5"/>
  <arg name="turn_angle" default="180.0"/>
  <arg name="repeat_enabled" default="false"/>
  <arg name="loop_count" default="-1"/>
  <arg name="multi_enabled" default="false"/>
  <arg name="multi_source" default="yaml"/>
  <arg name="multi_route_file" default=""/>
  <arg name="default_wait_time" default="2.0"/>

  <node pkg="local_planner" exec="cruiseController" name="cruise_controller" output="screen">
    <param name="max_yaw_rate" value="$(var max_yaw_rate)" />
    <param name="min_yaw_rate" value="$(var min_yaw_rate)" />
    <param name="yaw_kp" value="$(var yaw_kp)" />
    <param name="yaw_tolerance" value="$(var yaw_tolerance)" />
    <param name="goal_clear_range" value="$(var goal_clear_range)" />
    <param name="turn_angle" value="$(var turn_angle)" />
    <param name="repeat_enabled" value="$(var repeat_enabled)" />
    <param name="loop_count" value="$(var loop_count)" />
    <param name="multi_enabled" value="$(var multi_enabled)" />
    <param name="multi_source" value="$(var multi_source)" />
    <param name="multi_route_file" value="$(var multi_route_file)" />
    <param name="default_wait_time" value="$(var default_wait_time)" />
  </node>

</launch>
```

- [ ] **Step 2: Validate cruise.launch XML**

```bash
python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('cmu_planner/src/local_planner/launch/cruise.launch'); print('XML OK')"
```

- [ ] **Step 3: system_real_robot.launch — bind + declare + forward**

Add LaunchConfigurations after `min_yaw_rate`:

```python
  multi_enabled = LaunchConfiguration('multi_enabled')
  multi_source = LaunchConfiguration('multi_source')
  multi_route_file = LaunchConfiguration('multi_route_file')
  default_wait_time = LaunchConfiguration('default_wait_time')
```

Add declares after `declare_min_yaw_rate`:

```python
  declare_multi_enabled = DeclareLaunchArgument('multi_enabled', default_value='false', description='Enable multi-point cruise')
  declare_multi_source = DeclareLaunchArgument('multi_source', default_value='yaml', description='Waypoint source: yaml|rviz')
  declare_multi_route_file = DeclareLaunchArgument('multi_route_file', default_value='', description='Path to multi_route.yaml (default: package share)')
  declare_default_wait_time = DeclareLaunchArgument('default_wait_time', default_value='2.0', description='Default wait at each waypoint (s)')
```

Add to the `start_cruise` `launch_arguments`:

```python
      'multi_enabled': multi_enabled,
      'multi_source': multi_source,
      'multi_route_file': multi_route_file,
      'default_wait_time': default_wait_time,
```

Add to `ld.add_action` after `declare_min_yaw_rate`:

```python
  ld.add_action(declare_multi_enabled)
  ld.add_action(declare_multi_source)
  ld.add_action(declare_multi_route_file)
  ld.add_action(declare_default_wait_time)
```

- [ ] **Step 4: Validate system_real_robot.launch**

```bash
python3 -c "import ast; ast.parse(open('cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch').read()); print('launch OK')"
```

- [ ] **Step 5: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/src/local_planner/launch/cruise.launch \
        cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch
git commit -m "feat(multi): forward multi_enabled/multi_source/multi_route_file/default_wait_time"
```

---

### Task 9: 7multi.sh + RViz config

**Files:**
- Create: `cmu_planner/7multi.sh`
- Modify: `cmu_planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz`

**Interfaces:**
- Consumes: `system_real_robot.launch` args (`enableCruise`, `multi_enabled`, `multi_source`,
  `repeat_enabled`, `loop_count`, `rvizWaypointTopic`)
- Produces: runnable launcher; RViz displays `/multi_waypoints` and offers `Publish Point`→`/multi_waypoint_add`.

- [ ] **Step 1: Write 7multi.sh**

```bash
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

mode=${1:-yaml}
loop_count=${2:--1}

if [[ "$mode" != "yaml" && "$mode" != "rviz" ]]; then
  echo "Usage: bash 7multi.sh [yaml|rviz] [loop_count]"
  echo "Examples:"
  echo "  bash 7multi.sh yaml"
  echo "  bash 7multi.sh yaml 3"
  echo "  bash 7multi.sh rviz"
  echo "  bash 7multi.sh rviz 3"
  exit 1
fi

ros2 launch vehicle_simulator system_real_robot.launch \
  enableCruise:=true \
  multi_enabled:=true \
  repeat_enabled:=false \
  multi_source:=$mode \
  loop_count:=$loop_count \
  rvizWaypointTopic:=/way_point_cruise \
  2>&1 | grep --line-buffered -E 'MULTI|CRUISE|WAYPOINT|WARN|ERROR'
```

- [ ] **Step 2: Make executable**

```bash
chmod +x cmu_planner/7multi.sh
```

- [ ] **Step 3: Syntax check**

```bash
bash -n cmu_planner/7multi.sh && echo "syntax OK"
```

- [ ] **Step 4: RViz config — add PublishPoint tool**

In `vehicle_simulator.rviz`, find the `Tools:` block and add after the existing WaypointTool entry:

```yaml
    - Class: rviz_default_plugins/PublishPoint
      Name: Publish Point
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /multi_waypoint_add
```

- [ ] **Step 5: RViz config — add MarkerArray display**

In the `Displays:` section of `vehicle_simulator.rviz`, add a display block with:

```yaml
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: MultiWaypoints
      Namespaces:
        wp_sphere: true
        wp_text: true
        route: true
      Queue Size: 10
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /multi_waypoints
      Value: true
```

The Durability Policy must be **Transient Local** to match the publisher's
`transient_local()` QoS (R5/R14).

- [ ] **Step 6: Validate rviz file is still parseable YAML**

```bash
python3 -c "import yaml; yaml.safe_load(open('cmu_planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz')); print('rviz YAML OK')"
```

- [ ] **Step 7: Commit**

```bash
# from repo root (/home/yu/Codes_rk is the workspace root)
git add cmu_planner/7multi.sh cmu_planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz
git commit -m "feat(multi): add 7multi.sh launcher + RViz PublishPoint/MarkerArray config"
```

---

## Verification (end-to-end)

1. **Regression** — `multi_enabled=false` (default): `./3cruise.sh` and `./5repeat180.sh`
   behave unchanged.
2. **YAML mode** — `./7multi.sh yaml 1`: auto-starts WP0→WP1→WP2 (from default route),
   waits 2 s each, one closed loop then stops at WP0; `/multi_waypoints` shows spheres,
   WP labels, closed-loop line.
3. **RViz mode** — `./7multi.sh rviz`: robot still in `COLLECTING_WAYPOINTS`; click 3 points
   via Publish Point; markers appear incrementally; `ros2 service call /multi_start
   std_srvs/srv/Trigger "{}"` starts cruise. With <2 points the service rejects.
4. **Loop / stop** — `loop_count=3` does 3 closed loops; `ros2 topic pub /stop
   std_msgs/msg/Int8 "{data: 2}" --once` aborts in any state (mid-drive/turn/wait).
5. **Source exclusivity** — yaml mode ignores `/multi_waypoint_add` (throttled warn);
   rviz mode ignores YAML.
6. **Late RViz** — start RViz after the node; `/multi_waypoints` still visible
   (transient_local latched).
7. **Build** — full `colcon build --symlink-install --packages-select local_planner vehicle_simulator`.

## Out of Scope

- Runtime route editing after cruise starts (route frozen).
- YAML + RViz mixed sources.
- Pause/resume, auto-start on boot, multi-route switching.
- Changes to `localPlanner` / `pathFollower` travel logic or terrain stack.
