# Repeat Cruise Design Spec

> Date: 2026-08-06
> Branch: `repeat` (forked from `cruise`)
> Status: Approved by user

## Goal

Extend the existing single-pass cruise (`cruiseController`) with a **repeat mode**:
after a waypoint is given, the robot patrols back and forth continuously (or for
a fixed number of round-trips). A new `5repeat.sh` launch script drives it and
filters output to structured `[REPEAT]` logs, mirroring `3cruise.sh`'s `[CRUISE]`
filter.

## Context

The current `cruiseController.cpp` (on branch `cruise`) implements a 5-state
machine:

```
IDLE → GO_TO_DEST → TURN_AT_DEST → RETURN_TO_START → TURN_AT_START → IDLE
```

- Travel legs reuse the existing `/way_point` → `localPlanner` → `/path` →
  `pathFollower` → `/cmd_vel` pipeline.
- Turns are yaw-closed-loop: `cruiseController` publishes `/stop=2` to make
  `pathFollower` stop publishing `/cmd_vel`, then drives `/cmd_vel` directly
  (P-control to `current_yaw + π`, clamped by `max_yaw_rate`, done when
  `|yaw_error| < yaw_tolerance`).
- `pathFollower` was already modified: when `safetyStop == 2` it does NOT
  publish `/cmd_vel` (fixes ~50 Hz zero-speed contending with ~20 Hz turn).

## Design Decisions (from brainstorming)

1. **Approach**: Modify the existing `cruiseController.cpp` state machine only.
   No new node, no launch-level loop. (Approach A — approved.)
2. **Trigger/stop**: `loop_count` default `-1` (infinite); positive N = N
   round-trips; `1` = single round-trip. Infinite mode stopped via existing
   `/stop` topic (`data=2`).
3. **Loop unit**: `loop_count` counts **round-trips** (去→掉头→回→掉头 = 1).
4. **Mid-cruise retarget**: supported in repeat mode — replaces destination,
   resets loop count, re-anchors start at current pose, restarts from
   `GO_TO_DEST`.
5. **`/stop` self-publish conflict**: cruiseController publishes `/stop=2`
   during its own turns. Since it also subscribes `/stop` for external stop,
   the two collide on the same topic/value. Resolved with an internal
   `turning_internal_` flag: stop messages received while `turning_internal_`
   is true are treated as the node's own publish and queued (`pending_stop_`);
   stop messages received outside a turn stop immediately. (Option "标志位忽略自发布" — approved.)

## Requirements

**R1 — Backward compatibility.** `repeat_enabled=false` (default) keeps
single-pass behavior identical. `3cruise.sh` unaffected.

**R2 — Loop semantics.** `repeat_enabled=true`:
- `loop_count=-1` → infinite back-and-forth.
- `loop_count=N` → N full round-trips.
- `loop_count=1` → one round-trip (same as single pass, but still emits
  `[REPEAT]` logs and honors repeat-mode stop/retarget).

**R3 — Counting.** `completed_loops_` is incremented ONLY after the
`TURN_AT_START` 180° turn completes (i.e. at the end of each round-trip).
- Not reached → `state_ = GO_TO_DEST` (start = unchanged `start_`, dest =
  unchanged `dest_`; no need to republish `/way_point`).
- Reached → `completed_loops_ = 0`, `state_ = IDLE`, log complete.

**R4 — External stop.** Subscribe `/stop` (`std_msgs/Int8`). `data >= 2`:
- The node's own `/stop=2` (published at `startTurn()`) is consumed via
  `ignore_next_internal_stop_` and NOT treated as external.
- Outside a turn → `publishZeroCmd()`, reset counters, `state_ = IDLE`, log
  abort.
- During a turn (`turning_internal_ == true`) → set `pending_stop_ = true`,
  log "queued"; a queued stop is honored at BOTH `TURN_AT_DEST` and
  `TURN_AT_START` completion via `consumePendingStop()` — the robot stops as
  soon as the current turn finishes, no return leg is driven after a stop.
- Single mode (`repeat_enabled=false`): do NOT subscribe/respond (keep current
  behavior).

**R5 — Mid-cruise retarget.** In repeat mode, a new `/way_point_cruise` message
while cruising:
- Replaces `dest_x_/dest_y_`.
- Re-anchors `start_x_/start_y_` = current pose.
- Resets `completed_loops_ = 0`, `pending_stop_ = false`.
- Restarts from `GO_TO_DEST` (re-publishes `/way_point` to `localPlanner`).
- Logs `[REPEAT] New waypoint, loops reset`.
- In single mode: unchanged (ignore new waypoint while not IDLE).

**R6 — Launch script.** New `5repeat.sh` reuses `system_real_robot.launch`
with `enableCruise:=true`, `repeat_enabled:=true`, `loop_count`, and
`rvizWaypointTopic:=/way_point_cruise`; filters output to `[REPEAT]`.

**R7 — Structured logs.** Every phase transition emits `[REPEAT]`-prefixed
logs: loop start, destination reached, return start, loop complete, all-done,
stop, retarget. `5repeat.sh` shows only these.

## File Changes

| File | Action | Details |
|------|--------|---------|
| `cmu_planner/src/local_planner/src/cruiseController.cpp` | Modify | New params (`repeat_enabled`, `loop_count`), members (`completed_loops_`, `turning_internal_`, `ignore_next_internal_stop_`, `pending_stop_`), `/stop` subscription + callback, retarget logic in `waypointCallback`, loop-back in `TURN_AT_START`, `consumePendingStop()` at both turn-completion points, `[REPEAT]` logs. |
| `cmu_planner/src/local_planner/launch/cruise.launch` | Modify | Declare/pass `repeat_enabled`, `loop_count` to node. |
| `cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch` | Modify | Add `repeat_enabled`, `loop_count` launch args; forward to `cruise.launch` include. |
| `cmu_planner/5repeat.sh` | Create | Reuse `system_real_robot.launch`, pass repeat args, grep `[REPEAT]`. |

## State Machine Detail

```
                          ┌────────────────────────────────┐
                          │  completed_loops_++            │
                          ▼                                │
 GO_TO_DEST → TURN_AT_DEST → RETURN_TO_START → TURN_AT_START
     ▲                                                    │
     │     not done && !pending_stop_ ────────────────────┘
     │
     └───── done || pending_stop_ → completed_loops_=0, IDLE
```

### cruiseController.cpp changes

New params (declare with defaults):
```cpp
this->declare_parameter<bool>("repeat_enabled", false);
this->declare_parameter<int>("loop_count", -1);
```

New members:
```cpp
int completed_loops_ = 0;
bool turning_internal_ = false;
bool ignore_next_internal_stop_ = false;
bool pending_stop_ = false;
```

`startTurn()` — set flags; consume the node's own /stop=2 on the next callback:
```cpp
void startTurn(CruiseState next_state) {
  if (repeat_enabled_) {
    turning_internal_ = true;
    ignore_next_internal_stop_ = true;   // consume self-published /stop=2 below
  }
  auto stop_msg = std_msgs::msg::Int8();
  stop_msg.data = 2;
  stop_pub_->publish(stop_msg);
  target_yaw_ = normalizeAngle(current_yaw_ + M_PI);
  state_ = next_state;
  ...
}
```

`consumePendingStop()` — called at BOTH turn-completion points:
```cpp
bool consumePendingStop() {
  if (!pending_stop_) return false;
  publishZeroCmd();
  pending_stop_ = false;
  completed_loops_ = 0;
  RCLCPP_WARN(..., "[REPEAT] Queued stop executed after turn");
  state_ = IDLE;
  return true;
}
```

`turnDone()` transitions — clear flags, honor queued stop at both points:
```cpp
case CruiseState::TURN_AT_DEST:
  if (turnDone()) {
    turning_internal_ = false;
    ignore_next_internal_stop_ = false;
    publishZeroCmd();
    if (repeat_enabled_ && consumePendingStop()) return;   // stop here
    ... sendWaypointAndGo(start_x_, start_y_, RETURN_TO_START);
  } else publishTurnCmd();
case CruiseState::TURN_AT_START:
  if (turnDone()) {
    turning_internal_ = false;
    ignore_next_internal_stop_ = false;
    publishZeroCmd();
    if (!repeat_enabled_) {            // single mode: unchanged
      state_ = IDLE; return;
    }
    if (consumePendingStop()) return;  // stop here, no more loops
    completed_loops_++;
    if (loop_count_ > 0 && completed_loops_ >= loop_count_) {
      int loops_done = completed_loops_;
      completed_loops_ = 0;
      RCLCPP_INFO(..., "[REPEAT] Cruise complete after %d loops", loops_done);
      state_ = IDLE;
    } else {
      RCLCPP_INFO(..., "[REPEAT][LOOP] loop %d start", completed_loops_ + 1);
      state_ = GO_TO_DEST;             // start_/dest_ unchanged
    }
  } else publishTurnCmd();
```

`/stop` subscription (only when `repeat_enabled_`):
```cpp
void stopCallback(const std_msgs::msg::Int8::ConstSharedPtr msg) {
  if (msg->data < 2) return;
  if (ignore_next_internal_stop_) {   // consume the node's own /stop=2
    ignore_next_internal_stop_ = false;
    return;
  }
  if (turning_internal_) {
    pending_stop_ = true;             // real external stop during turn → queue
    RCLCPP_WARN(..., "[REPEAT] External stop queued during turn");
    return;
  }
  publishZeroCmd();
  completed_loops_ = 0; pending_stop_ = false;
  RCLCPP_WARN(..., "[REPEAT] Stop received, cruise aborted");
  state_ = IDLE;
}
```

**Why `ignore_next_internal_stop_` is required**: `turning_internal_` stays true
for the whole turn, so the node's own `/stop=2` (published at `startTurn()`)
would otherwise be misread as an external stop, setting `pending_stop_` on every
turn and aborting the cruise after each one. `ignore_next_internal_stop_`
consumes exactly that one self-publish; subsequent `/stop` messages during the
turn are genuine external stops and get queued.

`waypointCallback()` retarget (repeat mode):
```cpp
if (repeat_enabled_ && state_ != CruiseState::IDLE) {
  dest_x_ = msg->point.x; dest_y_ = msg->point.y;
  start_x_ = current_x_; start_y_ = current_y_;
  completed_loops_ = 0; pending_stop_ = false;
  turning_internal_ = false;
  ignore_next_internal_stop_ = false;
  RCLCPP_INFO(..., "[REPEAT] New waypoint, loops reset");
  sendWaypointAndGo(dest_x_, dest_y_, CruiseState::GO_TO_DEST);
  return;
}
// single mode: existing behavior (warn + ignore if not IDLE)
```

`[REPEAT]` logs to add:
- `[REPEAT][LOOP] loop N start: (sx,sy) -> (dx,dy)`
- `[REPEAT] Destination reached, loop N`
- `[REPEAT] Returning to start, loop N`
- `[REPEAT][LOOP] loop N complete`
- `[REPEAT] Cruise complete after N loops`
- `[REPEAT] Stop received, cruise aborted`
- `[REPEAT] Stop during turn queued`
- `[REPEAT] New waypoint, loops reset`

### cruise.launch changes

```xml
<arg name="repeat_enabled" default="false"/>
<arg name="loop_count" default="-1"/>
<node ... cruiseController ...>
  <param name="repeat_enabled" value="$(var repeat_enabled)"/>
  <param name="loop_count" value="$(var loop_count)"/>
  ...
</node>
```

### system_real_robot.launch changes

```python
repeat_enabled = LaunchConfiguration('repeat_enabled')
loop_count = LaunchConfiguration('loop_count')
declare_repeat_enabled = DeclareLaunchArgument('repeat_enabled', default_value='false', ...)
declare_loop_count = DeclareLaunchArgument('loop_count', default_value='-1', ...)
start_cruise = IncludeLaunchDescription(..., launch_arguments={
    'repeat_enabled': repeat_enabled,
    'loop_count': loop_count,
}.items())
ld.add_action(declare_repeat_enabled)
ld.add_action(declare_loop_count)
```

### 5repeat.sh

```bash
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source install/setup.bash
loop_count=${1:--1}
ros2 launch vehicle_simulator system_real_robot.launch \
  enableCruise:=true \
  rvizWaypointTopic:=/way_point_cruise \
  repeat_enabled:=true \
  loop_count:=$loop_count \
  2>&1 | grep --line-buffered REPEAT
```

Usage: `./5repeat.sh` (infinite) or `./5repeat.sh 3` (3 round-trips).

## Error Handling / Edge Cases

- Stop during turn → queued, applied after turn.
- `loop_count=0` → treated as 0 trips: `completed_loops_(0) >= loop_count_(0)` is
  true on first completion → immediately IDLE. Acceptable (no crash).
- `repeat_enabled=false` → no `/stop` subscription, no retarget, no loop-back:
  identical to current cruise.
- NaN/Inf waypoint or missing odom → existing `has_odom_` guard in
  `waypointCallback` already handles; unchanged.

## Testing / Verification

1. Build: `colcon build --symlink-install --packages-select local_planner vehicle_simulator`
2. **Regression**: `./3cruise.sh` — single pass unchanged.
3. **Fixed count**: `./5repeat.sh 2` — two round-trips then stop at start.
4. **Infinite + stop**: `./5repeat.sh`, then
   `ros2 topic pub /stop std_msgs/msg/Int8 "{data: 2}" --once` — stops
   immediately (or after current turn if mid-turn).
5. **Retarget mid-cruise**: while repeating, RViz-pick a new point — verify
   dest replaced, start re-anchored, loops reset, restarts.
6. **Log check**: `grep REPEAT` shows all 8 log lines in expected order.

## Out of Scope

- Multiple waypoint sequences (only one dest, back-and-forth).
- Pause/resume.
- Persistent waypoint lists / auto-start on boot.
- Any change to `localPlanner`, `pathFollower` travel logic, or terrain stack.
