# Unified Explicit Cruise Start (SINGLE / REPEAT / MULTI) — Implementation Plan

> **For agentic workers:** This plan was executed directly (not via subagent-driven-development). It documents the design, the implementation as committed, and the verification status. Commits: `29ab9d8`, `3d43ec2`, `5303dbb`.

**Goal:** All cruise modes — SINGLE, REPEAT, MULTI-RViz, MULTI-YAML — prepare their target/route but stay parked until the operator fires a single `Start Multi` RViz tool (which calls `/multi_start`). The robot only moves after that explicit grant.

**Architecture:** `CruiseController` uses one shared `CruiseState::READY_TO_START` for "goal/route prepared, motion not yet authorized": SINGLE/REPEAT enter it on waypoint click, MULTI-YAML enters it after localization, MULTI-RViz stays in `COLLECTING_WAYPOINTS`. `/multi_start` is created unconditionally and `startService()` dispatches by mode. A new `waypoint_rviz_plugin/MultiStartTool` is a thin trigger that fires `/multi_start` and `Q_EMIT close()`s back to the default tool.

**Tech Stack:** ROS 2 Humble, rviz_common Tool API, std_srvs (`srv::Trigger`), C++17, ament_cmake.

## Context / current behavior (before this change)

- `CruiseState` had no `READY_TO_START`. SINGLE/REPEAT started moving the instant a waypoint arrived (`waypointCallback` → `sendWaypointAndGo`). MULTI-YAML auto-started in the WAIT_LOCALIZATION gate (`controlLoop` → `beginMultiCruise` when `multi_source_=="yaml" && waypoints_.size()>=2`). MULTI-RViz already parked at `COLLECTING_WAYPOINTS` but required an external caller for `/multi_start` (hence `8multi_start.sh`).
- `/multi_start` was created only under `multi_enabled_`, and `startService()` only accepted `rviz + COLLECTING_WAYPOINTS`.

## Implemented changes (as committed)

### `planner/src/local_planner/src/cruiseController.cpp` — commit `29ab9d8`

- `CruiseState::READY_TO_START = 10` + `stateName()` entry.
- `/multi_start` service now created **unconditionally** (constructor); MULTI-only publishers (`/multi_waypoints`, `/multi_waypoint_add`, `/cruise_autonomy`) stay under `multi_enabled_`.
- `waypointCallback()` (SINGLE/REPEAT): stores `start_/dest_` (REPEAT also resets loop fields) and enters `READY_TO_START` — no `sendWaypointAndGo`, no motion. Running-state retarget for REPEAT now matches only the running states (`GO_TO_DEST/TURN_AT_DEST/RETURN_TO_START/TURN_AT_START`); a re-click while parked at `READY_TO_START` updates the destination and stays parked.
- `controlLoop()` WAIT_LOCALIZATION gate: yaml → `READY_TO_START` (never `beginMultiCruise`; <2 waypoints keeps it in WAIT with a throttled warning); rviz → `COLLECTING_WAYPOINTS` unchanged. Explicit `case READY_TO_START: return;` in the main switch.
- `startService()` unified dispatch:
  - `multi_enabled_`: `(rviz && COLLECTING_WAYPOINTS) || (yaml && READY_TO_START)`, plus existing ≥2-waypoints / `has_odom_` / map-mode TF checks → `/cruise_autonomy=true` + `beginMultiCruise()`. Already running → `"Multi cruise already running"`.
  - `repeat_enabled_`: requires `READY_TO_START` → resets loop fields → `sendWaypointAndGo(dest, GO_TO_DEST)`.
  - else (SINGLE): requires `READY_TO_START` → `sendWaypointAndGo(dest, GO_TO_DEST)`.

### `planner/src/waypoint_rviz_plugin/` — commit `3d43ec2`

- New `MultiStartTool` (`include/multi_start_tool.hpp`, `src/multi_start_tool.cpp`): plain `rviz_common::Tool` (not PoseTool). `onInitialize()` creates a `std_srvs::srv::Trigger` client for `/multi_start`. `activate()` checks `service_is_ready()` (WARN if unavailable), `async_send_request` with INFO/WARN on `success`/`message`, then `Q_EMIT close()` to return to the default tool. No state/legality checks in the plugin.
- `CMakeLists.txt`: `find_package(std_srvs REQUIRED)`, `multi_start_tool.hpp` in both `HDR_FILES` lists, `multi_start_tool.cpp` in `SRC_FILES`, `std_srvs` in `ament_target_dependencies` and `ament_export_dependencies`.
- `package.xml`: `<depend>std_srvs</depend>`.
- `plugin_description.xml`: registered `waypoint_rviz_plugin/MultiStartTool` (base `rviz_common::Tool`).

### RViz configs — commit `5303dbb`

- `planner/src/vehicle_simulator/rviz/vehicle_simulator.rviz` and `cruise_map.rviz`: added `waypoint_rviz_plugin/MultiStartTool` (`Name: Start Multi`) to the Tools section — so `3cruise.sh`, `5repeat180.sh`, and all `7multi.sh` modes see it.

## Final state machine

```
SINGLE:      IDLE →(click waypoint)→ READY_TO_START →(Start Multi)→ GO_TO_DEST → … (unchanged run logic)
REPEAT:      IDLE →(click destination)→ READY_TO_START →(Start Multi)→ GO_TO_DEST → TURN → RETURN → TURN → loop (unchanged)
MULTI-RViz:  WAIT_LOCALIZATION → COLLECTING_WAYPOINTS →(Start Multi)→ GO_TO_WAYPOINT
MULTI-YAML:  loadYaml → WAIT_LOCALIZATION → READY_TO_START →(Start Multi)→ GO_TO_WAYPOINT
```

`/multi_start` is now the single start gate for all modes; `8multi_start.sh` remains a valid CLI/debug fallback.

## Global constraints honored

- No `SLAM/` changes; no `localPlanner.cpp` / `pathFollower.cpp` changes; no `3cruise.sh` / `5repeat180.sh` / `7multi.sh` changes.
- Marker comments (`// ################################` / `// C++: <desc>` etc.) on every new/modified block; no END markers.
- No new ready flags, topics, services, lifecycle, TF guards, or watchdog — only the shared `READY_TO_START` state and the thin trigger tool.
- `/cruise_autonomy=true` only on successful MULTI start; SINGLE/REPEAT never touch it.

## Edge cases addressed

1. RViz waypoint <2 + Start → rejected (`At least 2 waypoints are required`).
2. YAML <2 waypoints → stays in `WAIT_LOCALIZATION`, never startable.
3. No `/state_estimation` → WAIT gate blocks reaching a startable state; `startService()` re-checks `has_odom_`.
4. odin_map mode without relocalization TF → gate + `startService()` reject.
5. YAML never auto-starts (`beginMultiCruise()` has exactly one call site: `startService`).
6. RViz keeps accepting waypoints in `COLLECTING_WAYPOINTS`.
7. YAML route immutable from RViz (existing `addMultiWaypoint` rejection unchanged).
8. Start while running → `"Multi cruise already running"`, no re-init.
9. `/stop` → `state_=IDLE` (unchanged). Restart requires re-clicking a goal; restart-to-ready is intentionally out of scope.
10. Autonomy lock timing unchanged.

## Verification

- Build: `cd planner && source /opt/ros/humble/setup.bash && colcon build --symlink-install` — **10 packages succeeded**, no errors.
- Plugin registration: installed `plugin_description.xml` contains `MultiStartTool`; the shared library exports `registerPlugin<MultiStartTool, rviz_common::Tool>` and the Q_OBJECT metacall symbols. (Note: `ros2 pkg plugins` does not exist in Humble.)
- Headless smoke (no robot), all PASS:
  - SINGLE: waypoint → `[CRUISE] Destination prepared ... press Start Multi` (no motion) → `/multi_start` success "Starting single cruise" → `GO_TO_DEST, publish /way_point`.
  - MULTI-YAML: after odom → `[MULTI] YAML route ready (3 waypoints); press Start Multi` (no auto-start) → `/multi_start` "Starting multi cruise" → `Cruise started` + `Going to WP0`.
  - REPEAT re-click regression: two clicks both logged `Destination prepared` (dest updated to 9.0), **no** `New waypoint, loops reset`, no premature start → `/multi_start` → `GO_TO_DEST` to the last dest.
- On-robot (PENDING-ROBOT): RViz toolbar `Start Multi` click firing + return-to-default-tool; the four launch flows; odin_map relocalization TF gate live; `/multi_waypoints` markers; `8multi_start.sh` fallback.

## Risks / notes

- No SINGLE/REPEAT run-loop logic (turn/return/loop/stop) changed; only "when to start".
- Test-environment caveat: a leftover cruiseController node can squat the `/multi_start` service name and answer stale — kill stray nodes before testing.
- The original plan scope (MULTI-only) was deliberately broadened to all modes per the user's directive; this document reflects the final implemented design.
