# Multi-Point Cruise Design Spec

> Date: 2026-08-10
> Branch: `multi` (forked from `cruise`)
> Status: Approved by user (revised per 12-constraint feedback)

## Goal

Extend `cruiseController` with a **multi-point patrol** mode (`multi_enabled=true`):
the robot visits a route of N waypoints in order, waits at each point, and loops
back to WP0 for N rounds (or infinitely). Waypoints come from **exactly one of two
mutually exclusive sources** chosen at launch: a YAML route file, or RViz manual
placement. RViz2 shows the full route via MarkerArray.

## Context

Current `cruiseController.cpp` (branch `cruise`) implements single-pass (`repeat_enabled=false`)
and back-and-forth (`repeat_enabled=true`) patrol via a 5-state machine:
`IDLE → GO_TO_DEST → TURN_AT_DEST → RETURN_TO_START → TURN_AT_START`.
Turn control is yaw-closed-loop; `/cmd_vel` contention with `pathFollower` is handled by
publishing `/stop=2` (pathFollower stops publishing) and taking over `/cmd_vel` directly.
The `/stop` self-publish is consumed via `turning_internal_` / `ignore_next_internal_stop_` /
`pending_stop_` flags.

MULTI generalizes this: a waypoint **queue** replaces the single `dest`/`start` pair.
`N=2` with two opposite points is the degenerate back-and-forth case.

## Design Decisions (from brainstorming + 12-constraint revision)

1. **Approach**: extend `cruiseController.cpp` (single node, no new controller). Approved.
2. **Waypoint source is EXCLUSIVE, chosen at launch** — never merged:
   - `multi_source=yaml`: load `multi_route.yaml`, auto-start from WP0.
     `/multi_waypoint_add` is ignored (throttled warning).
   - `multi_source=rviz`: start in `COLLECTING_WAYPOINTS`, robot stays still,
     RViz `Publish Point` appends waypoints, `/multi_start` (Trigger service)
     begins cruising. YAML waypoints are completely ignored.
   - Both modes share the same runtime params (`loop_count`, `default_wait_time`,
     `goal_clear_range`, yaw/turn params) — only the waypoint *source* differs.
3. **Stop semantics**: `loop_count=-1` = infinite loop; `N` = N full route rounds.
   External `/stop=2` aborts in any state (reuses repeat's flag mechanism).
4. **Wait at each waypoint**: `default_wait_time` node param (2.0 s), overridable
   per-waypoint in YAML. Waiting uses ROS time-diff (no `sleep()`), so `/stop`
   stays responsive during waits.
5. **Turn**: per-waypoint `turn_angle` (deg); if non-zero, turn in place after
   arrival before waiting. YAML's last waypoint typically sets `turn_angle: 180.0`
   to face back to WP0 for the loop. RViz mode points default `turn_angle=0`.
6. **Visualization**: `/multi_waypoints` MarkerArray (map frame) — SPHERE per
   waypoint, TEXT_VIEW_FACING labels (WP0/WP1/...), LINE_STRIP connecting the
   route (last→first included for closed loop), current waypoint highlighted.
   Both modes publish it.
7. **Backward compatibility**: `multi_enabled=false` (default) keeps SINGLE/REPEAT
   behavior byte-for-byte identical. `7multi.sh` forces `repeat_enabled:=false`.

## Requirements

**R1 — Launch script `7multi.sh`.** New `cmu_planner/7multi.sh` (executable):

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

Usage: `bash 7multi.sh yaml` / `bash 7multi.sh yaml 3` / `bash 7multi.sh rviz` /
`bash 7multi.sh rviz 3`. New launch arg `multi_source` with legal values `yaml|rviz`;
`cruiseController` validates it at startup (invalid → error exit).

**R2 — Mutually exclusive sources.** At startup `cruiseController` reads
`multi_source`:
- `yaml` → `loadYaml()` → `publishMarkers()` → `GO_TO_WAYPOINT` (WP0).
  `/multi_waypoint_add` subscription ignored; on receipt log throttled
  `[MULTI] Ignoring RViz waypoint because multi_source=yaml`.
- `rviz` → `waypoints_` empty → `COLLECTING_WAYPOINTS`. RViz points append;
  route frozen once cruising starts.

**R3 — RViz input topic.** `/multi_waypoint_add` (`geometry_msgs/PointStamped`).
RViz `rviz_default_plugins/PublishPoint` tool configured to publish there
(added to `vehicle_simulator.rviz`). Coordinates are map-frame (RViz fixed frame
is map). Only handled when `multi_source=rviz` and state is `COLLECTING_WAYPOINTS`.
Do NOT reuse the custom WaypointTool (it also publishes `/joy` and drives
`/way_point`); `/way_point` and `/way_point_cruise` behavior unchanged.

**R4 — COLLECTING_WAYPOINTS + /multi_start.** RViz mode starts still in
`COLLECTING_WAYPOINTS`. Each `/multi_waypoint_add` appends + republishes markers.
`/multi_start` = `std_srvs/srv/Trigger`:
- `waypoints_.size() < 2` → `success=false`, `message="At least 2 waypoints are required"`.
- `>= 2` → `success=true`, `waypoint_index_=0`, `completed_loops_=0`, → `GO_TO_WAYPOINT` (WP0).
After cruising starts, `/multi_waypoint_add` is ignored:
`[MULTI] Route already started; RViz waypoint ignored`. Route is frozen in v1.

**R5 — MarkerArray** `/multi_waypoints` (`visualization_msgs/MarkerArray`, map frame):
- SPHERE: one per waypoint (position).
- TEXT_VIEW_FACING: `WP0`, `WP1`, ... labels.
- LINE_STRIP: connects route in order; for closed loop include last→first.
- Current executing waypoint highlighted (distinct color/size).
YAML mode publishes full route at startup; RViz mode publishes incrementally during
collection. Both republish on any change.

**R6 — Wait param is a node param, not YAML-only.** `default_wait_time` is a
`cruiseController` ROS param (default 2.0 s) passed via `cruise.launch` and
`system_real_robot.launch`. RViz points get `wait_time = default_wait_time`.

**R7 — YAML file carries route only.** `multi_route.yaml` has no runtime params
(those are ROS params):

```yaml
multi_cruise:
  waypoints:
    - x: 1.0
      y: 2.0
    - x: 5.0
      y: 2.0
    - x: 9.0
      y: 2.0
      turn_angle: 180.0
```

Per-waypoint optional overrides: `turn_angle` (default 0.0), `wait_time`
(default = `default_wait_time`; `0.0` = no wait). File lives at
`cmu_planner/src/local_planner/config/multi_route.yaml` (default value of
`multi_route_file` param).

**R8 — Waypoint struct.**

```cpp
struct Waypoint {
  double x, y;
  double turn_angle = 0.0;
  double wait_time = 2.0;   // initialized from default_wait_time param
};
```

**R9 — MULTI state machine.**

```
IDLE
 ↓ (yaml: loadYaml→publishMarkers)  (rviz: COLLECTING_WAYPOINTS→/multi_start)
GO_TO_WAYPOINT            # drive to waypoints_[waypoint_index_]
 ↓ reached
TURN_AT_WAYPOINT          # only if currentWaypoint.turn_angle != 0
 ↓ turn done
WAIT_AT_WAYPOINT          # skip if wait_time <= 0
 ↓ (now - wait_start_time_) >= wait_time
advanceWaypoint()         # waypoint_index_++; if past last: completed_loops_++, index=0
 ↓
GO_TO_WAYPOINT
```

After the last waypoint: `completed_loops_++`. If `loop_count==-1 || completed_loops_ < loop_count`
→ back to WP0; else → `IDLE` + `publishZeroCmd()`.

YAML mode never enters `COLLECTING_WAYPOINTS`; RViz mode skips YAML load.
**Both modes validate `waypoints_.size() >= 2` before starting cruise** — YAML mode
at startup (error log + stay IDLE if the file has <2 points), RViz mode in
`/multi_start` (Trigger `success=false`).

**R10 — Arrival sequence (fixed order).**

```
GO_TO_WAYPOINT → arrive → stop (publishZeroCmd)
→ TURN_AT_WAYPOINT (if turn_angle != 0)
→ WAIT_AT_WAYPOINT (wait_time, via time-diff, no sleep; /stop responsive)
→ sendWaypointAndGo(next /way_point) + /stop=0 → pathFollower resumes
```

No non-zero motion commands are published to pathFollower during the wait.

**R11 — Control authority (unchanged mechanism).**
- Normal cruise: `pathFollower → /cmd_vel`.
- In-place turn: `cruiseController → /stop=2` (pathFollower stops publishing) →
  `cruiseController` drives `/cmd_vel` directly.
- Wait: remain stopped.
- Resume: publish next `/way_point` + `/stop=0` → pathFollower regains `/cmd_vel`.
No second controller contends for `/cmd_vel`.

MULTI mode MUST subscribe `/stop` (like REPEAT) and reuse/generalize
`turning_internal_` / `ignore_next_internal_stop_` / `pending_stop_` so the node's
own internal `/stop=2` is not mistaken for an external stop.

**R12 — Coordinate separation.** `waypoints_` are stored in map frame. The current
waypoint is converted to the planner's coordinate frame when sent via `/way_point`.
RViz-added points never bypass the state machine to `localPlanner`.

**R13 — New dependencies.** `local_planner` adds:
- `yaml-cpp` (parse `multi_route.yaml`)
- `visualization_msgs` (MarkerArray)
- `std_srvs` (`Trigger` service)

in both `CMakeLists.txt` (`find_package` + `ament_target_dependencies`) and
`package.xml` (`<depend>`).

**R14 — RViz config.** `vehicle_simulator.rviz` adds:
- MarkerArray display subscribing `/multi_waypoints`.
- `rviz_default_plugins/PublishPoint` tool with topic `/multi_waypoint_add`
  (for RViz mode).

## File Changes

| File | Action | Details |
|------|--------|---------|
| `cmu_planner/src/local_planner/src/cruiseController.cpp` | Modify | `Waypoint` struct; `waypoints_`, `waypoint_index_`, `completed_loops_`; MULTI states (`COLLECTING_WAYPOINTS`, `GO_TO_WAYPOINT`, `TURN_AT_WAYPOINT`, `WAIT_AT_WAYPOINT`); `/multi_waypoint_add` sub; `/multi_start` service; `/multi_waypoints` MarkerArray pub; `loadYaml()`; `publishMarkers()`; `advanceWaypoint()`; params (`multi_enabled`, `multi_source`, `multi_route_file`, `default_wait_time`); `[MULTI]` logs. |
| `cmu_planner/src/local_planner/launch/cruise.launch` | Modify | Add `multi_enabled`, `multi_source`, `multi_route_file`, `default_wait_time` args → node params. |
| `cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch` | Modify | Forward `multi_enabled`, `multi_source`, `multi_route_file`, `default_wait_time` into cruise include. |
| `cmu_planner/src/local_planner/CMakeLists.txt` | Modify | Add yaml-cpp, visualization_msgs, std_srvs. |
| `cmu_planner/src/local_planner/package.xml` | Modify | Add yaml-cpp, visualization_msgs, std_srvs depends. |
| `cmu_planner/src/local_planner/config/multi_route.yaml` | Create | Default YAML route (route only). |
| `cmu_planner/7multi.sh` | Create | Executable launcher (yaml|rviz mode + loop_count). |
| `cmu_planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz` | Modify | MarkerArray display + PublishPoint tool. |

## Testing / Verification

1. Build: `colcon build --symlink-install --packages-select local_planner vehicle_simulator`.
2. **Regression**: `multi_enabled=false` → `3cruise.sh` / `5repeat180.sh` unchanged.
3. **YAML mode**: `bash 7multi.sh yaml 1` — auto-starts WP0→...→last, waits 2 s each,
   one full round then stops at WP0; `/multi_waypoints` shows full route + labels.
4. **RViz mode**: `bash 7multi.sh rviz` — robot still; click 3 points; markers appear
   incrementally; `ros2 service call /multi_start std_srvs/srv/Trigger "{}"` starts.
   With <2 points the service rejects.
5. **Loop / stop**: `loop_count=3` does 3 rounds; external `/stop=2` aborts in any
   state (mid-drive, mid-turn, mid-wait).
6. **Source exclusivity**: `yaml` mode ignores `/multi_waypoint_add` (throttled warn);
   `rviz` mode ignores YAML.
7. **Wait override**: YAML `wait_time: 5.0` waits 5 s; `0.0` no wait.

## Out of Scope (v1)

- Runtime route editing after cruise starts (route frozen).
- YAML + RViz mixed sources.
- Pause/resume, auto-start on boot, multi-route file switching.
- Changes to `localPlanner` / `pathFollower` travel logic or terrain stack.
