# Repeat Cruise Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend `cruiseController` with a repeat mode (loop back-and-forth infinitely or N round-trips), plus a `5repeat.sh` launcher that filters output to `[REPEAT]` logs.

**Architecture:** Modify only the existing `cruiseController.cpp` state machine. The `TURN_AT_START` completion handler gains a loop-back branch. A new `/stop` subscription handles external stop, with an internal `turning_internal_` flag to ignore the node's own `/stop=2` publishes during turns. `cruise.launch` and `system_real_robot.launch` pass two new params through.

**Tech Stack:** C++17, rclcpp, ROS 2 Humble, geometry_msgs, nav_msgs, std_msgs, launch XML/Python.

## Global Constraints

- Branch: `repeat` (already checked out; fork of `cruise`).
- `repeat_enabled=false` (default) MUST keep single-pass behavior byte-for-byte identical — `3cruise.sh` unaffected.
- `loop_count` unit = **round-trips** (去→掉头→回→掉头 = 1). `-1` = infinite, `N` = N round-trips, `1` = one round-trip.
- `completed_loops_` increments ONLY after the `TURN_AT_START` 180° turn completes.
- External stop via existing `/stop` topic, `data >= 2`. Self-published `/stop=2` during turns must be ignored via `turning_internal_`.
- Mid-cruise retarget (repeat mode only): replace dest, re-anchor start at current pose, reset counters, restart from `GO_TO_DEST`.
- Every phase transition logs `[REPEAT]`-prefixed lines; `5repeat.sh` greps `REPEAT`.
- Do NOT modify `localPlanner.cpp`, `pathFollower.cpp` travel logic, or terrain packages.

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `cmu_planner/src/local_planner/src/cruiseController.cpp` | Modify | Repeat params, members, `/stop` sub, retarget, loop-back, `[REPEAT]` logs |
| `cmu_planner/src/local_planner/launch/cruise.launch` | Modify | Declare/pass `repeat_enabled`, `loop_count` |
| `cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch` | Modify | Forward `repeat_enabled`, `loop_count` into cruise.launch include |
| `cmu_planner/5repeat.sh` | Create | Launch main system + cruise repeat, grep `REPEAT` |

---

### Task 1: Add repeat params and members to cruiseController.cpp

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: nothing (internal)
- Produces: member fields `repeat_enabled_`, `loop_count_`, `completed_loops_`, `turning_internal_`, `pending_stop_` — used by Tasks 2-4.

- [ ] **Step 1: Declare parameters in the constructor**

In the constructor body (after line 32 `goal_clear_range` declare), add:

```cpp
    this->declare_parameter<bool>("repeat_enabled", false);
    this->declare_parameter<int>("loop_count", -1);
```

- [ ] **Step 2: Add member variables**

In the member block (replace lines 240-244):

```cpp
  CruiseState state_;
  bool has_odom_;
  bool repeat_enabled_;
  int loop_count_;
  int completed_loops_ = 0;
  bool turning_internal_ = false;
  bool pending_stop_ = false;
  double start_x_, start_y_, dest_x_, dest_y_;
  double current_x_, current_y_, current_yaw_;
  double target_yaw_;
```

- [ ] **Step 3: Read params into members**

In the constructor after the declares, add:

```cpp
    repeat_enabled_ = this->get_parameter("repeat_enabled").as_bool();
    loop_count_ = this->get_parameter("loop_count").as_int();
```

- [ ] **Step 4: Build**

```bash
cd /home/yu/Codes_rk/cmu_planner
colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 5: Commit**

```bash
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(repeat): add repeat_enabled/loop_count params and loop members"
```

---

### Task 2: Add /stop subscription and callback

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `turning_internal_`, `pending_stop_`, `completed_loops_`, `state_`, `publishZeroCmd()`
- Produces: `stopCallback()` — sets `pending_stop_` or stops cruise. `stop_sub_` member.

- [ ] **Step 1: Add `stopCallback` method**

Insert after `waypointCallback()` (after line 116), before `publishZeroCmd()`:

```cpp
  void stopCallback(const std_msgs::msg::Int8::ConstSharedPtr msg)
  {
    if (msg->data < 2) return;
    if (turning_internal_) {
      pending_stop_ = true;
      RCLCPP_WARN(this->get_logger(),
        "[REPEAT] Stop during turn queued");
      return;
    }
    publishZeroCmd();
    completed_loops_ = 0;
    pending_stop_ = false;
    RCLCPP_INFO(this->get_logger(),
      "[REPEAT] Stop received, cruise aborted");
    state_ = CruiseState::IDLE;
  }
```

- [ ] **Step 2: Subscribe to /stop in constructor**

Add after `waypoint_sub_` creation (line 40), guarded so single mode does not subscribe:

```cpp
    if (this->get_parameter("repeat_enabled").as_bool()) {
      stop_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/stop", 10,
        std::bind(&CruiseController::stopCallback, this, std::placeholders::_1));
    }
```

- [ ] **Step 3: Add `stop_sub_` member**

Add to the subscription member block (near line 247):

```cpp
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stop_sub_;
```

- [ ] **Step 4: Build**

```bash
cd /home/yu/Codes_rk/cmu_planner
colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 5: Commit**

```bash
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(repeat): add /stop external stop subscription and callback"
```

---

### Task 3: Set turning_internal_ flag and loop-back in TURN states

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `repeat_enabled_`, `loop_count_`, `completed_loops_`, `pending_stop_`, `start_/dest_`, `turning_internal_`
- Produces: loop-back behavior — `TURN_AT_START` completion either goes to `GO_TO_DEST` (more loops) or `IDLE` (done/stopped).

- [ ] **Step 1: Set turning_internal_ in startTurn()**

Modify `startTurn()` (lines 150-161): add `turning_internal_ = true;` at the top:

```cpp
  void startTurn(CruiseState next_state)
  {
    turning_internal_ = true;   // ignore self-published /stop=2
    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 2;
    stop_pub_->publish(stop_msg);

    target_yaw_ = normalizeAngle(current_yaw_ + M_PI);
    state_ = next_state;
    RCLCPP_INFO(this->get_logger(),
      "[CRUISE] Starting 180-degree turn: %s, target_yaw=%.3f (current=%.3f)",
      stateName(next_state), target_yaw_, current_yaw_);
  }
```

- [ ] **Step 2: Clear flag in TURN_AT_DEST completion**

Modify the `TURN_AT_DEST` case (lines 208-216):

```cpp
    case CruiseState::TURN_AT_DEST:
      if (turnDone()) {
        turning_internal_ = false;
        publishZeroCmd();
        RCLCPP_INFO(this->get_logger(), "[CRUISE] Turn done, returning to start...");
        sendWaypointAndGo(start_x_, start_y_, CruiseState::RETURN_TO_START);
      } else {
        publishTurnCmd();
      }
      return;
```

- [ ] **Step 3: Loop-back in TURN_AT_START completion**

Replace the `TURN_AT_START` case (lines 228-236):

```cpp
    case CruiseState::TURN_AT_START:
      if (turnDone()) {
        turning_internal_ = false;
        publishZeroCmd();

        if (!repeat_enabled_) {
          RCLCPP_INFO(this->get_logger(), "[CRUISE] Cruise complete!");
          state_ = CruiseState::IDLE;
          return;
        }

        completed_loops_++;
        RCLCPP_INFO(this->get_logger(),
          "[REPEAT][LOOP] loop %d/%s complete",
          completed_loops_,
          (loop_count_ > 0 ? std::to_string(loop_count_).c_str() : "inf"));

        if (pending_stop_ || (loop_count_ > 0 && completed_loops_ >= loop_count_)) {
          int loops_done = completed_loops_;
          completed_loops_ = 0;
          pending_stop_ = false;
          RCLCPP_INFO(this->get_logger(),
            "[REPEAT] Cruise complete after %d loops", loops_done);
          state_ = CruiseState::IDLE;
        } else {
          RCLCPP_INFO(this->get_logger(),
            "[REPEAT][LOOP] loop %d start: (%.3f, %.3f) -> (%.3f, %.3f)",
            completed_loops_ + 1,
            start_x_, start_y_, dest_x_, dest_y_);
          state_ = CruiseState::GO_TO_DEST;
        }
      } else {
        publishTurnCmd();
      }
      return;
```

> **Note**: `start_x_/start_y_` are unchanged (this is the return-to-start leg's
> arrival), and `dest_x_/dest_y_` are unchanged, so no `/way_point` republish is
> needed — just set `state_ = GO_TO_DEST`.

- [ ] **Step 4: Add destination-reached and return logs (repeat only)**

Add `[REPEAT]`-prefixed lines in the GO_TO_DEST and RETURN_TO_START reach branches:

In `GO_TO_DEST` reach block (line 202-205), add a repeat log before startTurn:

```cpp
      if (dist_sq < goal_clear_range_sq) {
        if (repeat_enabled_) {
          RCLCPP_INFO(this->get_logger(),
            "[REPEAT] Destination reached, loop %d", completed_loops_ + 1);
        }
        RCLCPP_INFO(this->get_logger(), "[CRUISE] Destination reached, turning...");
        startTurn(CruiseState::TURN_AT_DEST);
      }
```

In `RETURN_TO_START` reach block (line 222-225), add a repeat log:

```cpp
      if (dist_sq < goal_clear_range_sq) {
        if (repeat_enabled_) {
          RCLCPP_INFO(this->get_logger(),
            "[REPEAT] Returning to start, loop %d", completed_loops_ + 1);
        }
        RCLCPP_INFO(this->get_logger(), "[CRUISE] Start reached, turning...");
        startTurn(CruiseState::TURN_AT_START);
      }
```

- [ ] **Step 5: Include `<string>` for std::to_string**

Add `#include <string>` to the includes at the top of the file.

- [ ] **Step 6: Build**

```bash
cd /home/yu/Codes_rk/cmu_planner
colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 7: Commit**

```bash
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(repeat): loop-back in TURN_AT_START, turning_internal_ flag, [REPEAT] logs"
```

---

### Task 4: Mid-cruise retarget in waypointCallback

**Files:**
- Modify: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `repeat_enabled_`, `state_`, `current_`, `dest_`, `start_`, counters
- Produces: retarget behavior — non-IDLE + repeat mode accepts new waypoint, resets, restarts.

- [ ] **Step 1: Add retarget branch to waypointCallback**

Replace `waypointCallback()` (lines 92-116):

```cpp
  void waypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!has_odom_) {
      RCLCPP_WARN(this->get_logger(),
        "No /state_estimation received, ignoring cruise waypoint");
      return;
    }

    // Repeat mode: accept new waypoint even while cruising (retarget + reset)
    if (repeat_enabled_ && state_ != CruiseState::IDLE) {
      dest_x_ = msg->point.x;
      dest_y_ = msg->point.y;
      start_x_ = current_x_;
      start_y_ = current_y_;
      completed_loops_ = 0;
      pending_stop_ = false;
      turning_internal_ = false;
      RCLCPP_INFO(this->get_logger(),
        "[REPEAT] New waypoint, loops reset: dest=(%.3f, %.3f), start=(%.3f, %.3f)",
        dest_x_, dest_y_, start_x_, start_y_);
      sendWaypointAndGo(dest_x_, dest_y_, CruiseState::GO_TO_DEST);
      return;
    }

    if (state_ != CruiseState::IDLE) {
      RCLCPP_WARN(this->get_logger(),
        "Already cruising, ignoring new waypoint");
      return;
    }

    // 收到目标时锁定起点（当前实时位置）
    start_x_ = current_x_;
    start_y_ = current_y_;

    dest_x_ = msg->point.x;
    dest_y_ = msg->point.y;
    RCLCPP_INFO(this->get_logger(),
      "[CRUISE][INPUT] start=(%.3f, %.3f), destination=(%.3f, %.3f)",
      start_x_, start_y_, dest_x_, dest_y_);
    sendWaypointAndGo(dest_x_, dest_y_, CruiseState::GO_TO_DEST);
  }
```

- [ ] **Step 2: Build**

```bash
cd /home/yu/Codes_rk/cmu_planner
colcon build --symlink-install --packages-select local_planner
```

Expected: builds clean.

- [ ] **Step 3: Commit**

```bash
git add cmu_planner/src/local_planner/src/cruiseController.cpp
git commit -m "feat(repeat): mid-cruise retarget with loop reset in repeat mode"
```

---

### Task 5: Pass repeat params through cruise.launch

**Files:**
- Modify: `cmu_planner/src/local_planner/launch/cruise.launch`

**Interfaces:**
- Consumes: nothing
- Produces: `repeat_enabled` and `loop_count` launch args forwarded to the node as params (consumed by Task 1).

- [ ] **Step 1: Add launch args**

Replace `cruise.launch` content:

```xml
<launch>

  <arg name="max_yaw_rate" default="45.0"/>
  <arg name="yaw_kp" default="1.5"/>
  <arg name="yaw_tolerance" default="0.12"/>
  <arg name="goal_clear_range" default="0.5"/>
  <arg name="repeat_enabled" default="false"/>
  <arg name="loop_count" default="-1"/>

  <node pkg="local_planner" exec="cruiseController" name="cruise_controller" output="screen">
    <param name="max_yaw_rate" value="$(var max_yaw_rate)" />
    <param name="yaw_kp" value="$(var yaw_kp)" />
    <param name="yaw_tolerance" value="$(var yaw_tolerance)" />
    <param name="goal_clear_range" value="$(var goal_clear_range)" />
    <param name="repeat_enabled" value="$(var repeat_enabled)" />
    <param name="loop_count" value="$(var loop_count)" />
  </node>

</launch>
```

- [ ] **Step 2: Validate XML syntax**

```bash
python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('/home/yu/Codes_rk/cmu_planner/src/local_planner/launch/cruise.launch'); print('XML OK')"
```

- [ ] **Step 3: Commit**

```bash
git add cmu_planner/src/local_planner/launch/cruise.launch
git commit -m "feat(repeat): forward repeat_enabled/loop_count in cruise.launch"
```

---

### Task 6: Forward repeat args through system_real_robot.launch

**Files:**
- Modify: `cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch`

**Interfaces:**
- Consumes: nothing
- Produces: `repeat_enabled`/`loop_count` launch args forwarded into the `cruise.launch` include (consumed by Task 5).

- [ ] **Step 1: Add LaunchConfiguration bindings**

After line 17 (`rvizWaypointTopic = LaunchConfiguration(...)`), add:

```python
  repeat_enabled = LaunchConfiguration('repeat_enabled')
  loop_count = LaunchConfiguration('loop_count')
```

- [ ] **Step 2: Add DeclareLaunchArgument**

After line 24, add:

```python
  declare_repeat_enabled = DeclareLaunchArgument('repeat_enabled', default_value='false', description='Enable repeat cruise mode')
  declare_loop_count = DeclareLaunchArgument('loop_count', default_value='-1', description='Round-trips (-1=infinite)')
```

- [ ] **Step 3: Forward into cruise include**

Modify the `start_cruise` block (lines 71-76):

```python
  start_cruise = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'cruise.launch')
    ),
    condition=IfCondition(enableCruise),
    launch_arguments={
      'repeat_enabled': repeat_enabled,
      'loop_count': loop_count,
    }.items()
  )
```

- [ ] **Step 4: Add actions**

After `ld.add_action(declare_rvizWaypointTopic)` (line 104), add:

```python
  ld.add_action(declare_repeat_enabled)
  ld.add_action(declare_loop_count)
```

- [ ] **Step 5: Validate syntax**

```bash
cd /home/yu/Codes_rk/cmu_planner
python3 -c "import ast; ast.parse(open('src/vehicle_simulator/launch/system_real_robot.launch').read()); print('launch OK')"
```

- [ ] **Step 6: Commit**

```bash
git add cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch
git commit -m "feat(repeat): forward repeat_enabled/loop_count in system_real_robot.launch"
```

---

### Task 7: Create 5repeat.sh

**Files:**
- Create: `cmu_planner/5repeat.sh`

**Interfaces:**
- Consumes: `system_real_robot.launch` args `enableCruise`, `rvizWaypointTopic`, `repeat_enabled`, `loop_count`
- Produces: a runnable script launching the full stack in repeat-cruise mode.

- [ ] **Step 1: Write the script**

```bash
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# loop_count defaults to -1 (infinite); pass a number for fixed round-trips
loop_count=${1:--1}

ros2 launch vehicle_simulator system_real_robot.launch \
  enableCruise:=true \
  rvizWaypointTopic:=/way_point_cruise \
  repeat_enabled:=true \
  loop_count:=$loop_count \
  2>&1 | grep --line-buffered REPEAT
```

- [ ] **Step 2: Make executable**

```bash
chmod +x /home/yu/Codes_rk/cmu_planner/5repeat.sh
```

- [ ] **Step 3: Verify it parses (no launch yet — requires hardware)**

```bash
bash -n /home/yu/Codes_rk/cmu_planner/5repeat.sh && echo "syntax OK"
```

- [ ] **Step 4: Commit**

```bash
git add cmu_planner/5repeat.sh
git commit -m "feat(repeat): add 5repeat.sh launcher for repeat cruise (grep REPEAT)"
```

---

## Verification (end-to-end)

1. **Regression** — `repeat_enabled=false` (default): run `./3cruise.sh`, confirm single pass unchanged (`[CRUISE]` logs, no `[REPEAT]`).
2. **Fixed count** — `./5repeat.sh 2`: robot does 2 round-trips then stops at start; logs `loop 1/2 complete`, `loop 2/2 complete`, `Cruise complete after 2 loops`.
3. **Infinite + stop** — `./5repeat.sh`: after it starts, run
   `ros2 topic pub /stop std_msgs/msg/Int8 "{data: 2}" --once`. If mid-turn, expect `[REPEAT] Stop during turn queued` then stop after the turn; otherwise `[REPEAT] Stop received, cruise aborted`.
4. **Retarget** — while repeating, RViz-pick a new point: expect `[REPEAT] New waypoint, loops reset`, robot re-anchors start at current pose and heads to new dest.
5. **Log filter** — `./5repeat.sh` shows only `[REPEAT]` lines; `./4debug.sh` (no grep) shows both `[CRUISE]` and `[REPEAT]`.

## Out of Scope

- Multiple waypoint sequences, pause/resume, persistent waypoint lists, auto-start on boot.
- Changes to `localPlanner`, `pathFollower` travel logic, terrain stack.
