#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# ################################
# Bash: launch multi-point cruise with yaml/rviz source and odin_odom/odin_map frame
# ################################
# RViz display frame and MULTI internal planning frame are independent:
# - Default multi_frame is "odin_odom" (no map / no relocalization needed).
# - RViz Fixed Frame is "odin_odom" (no-map mode) or "odin_map" (map mode).
# - Clicked PointStamped waypoints are automatically transformed to multi_frame.
mode=${1:-rviz}
loop_count=${2:--1}
frame=${3:-odin_odom}

if [[ "$mode" != "yaml" && "$mode" != "rviz" ]]; then
  echo "Usage: bash 7multi.sh [yaml|rviz] [loop_count] [odin_odom|odin_map]"
  echo "Examples:"
  echo "  bash 7multi.sh                  # default: RViz clicks, odin_odom frame (no map needed)"
  echo "  bash 7multi.sh rviz 3           # RViz clicks, odin_odom frame, 3 loops"
  echo "  bash 7multi.sh yaml             # YAML route, odin_odom frame (no map needed)"
  echo "  bash 7multi.sh yaml 3           # YAML route, odin_odom frame, 3 loops"
  echo "  bash 7multi.sh yaml -1 odin_map # YAML route in prebuilt map frame (needs Odin relocalization)"
  echo "  bash 7multi.sh rviz -1 odin_map # Map-mode MULTI (needs Odin relocalization):"
  echo "                                  # requires SLAM/3run_relocalization.sh; /overall_map"
  echo "                                  # is published by the SLAM stack; opens cruise_map.rviz"
  echo "                                  # (Fixed Frame=odin_map, OverallMap enabled)."
  echo "Note: legacy 'odom'/'map' arguments are accepted and normalized."
  echo "      RViz Fixed Frame is 'odin_odom' (no-map) or 'odin_map'"
  echo "      (map mode); clicked waypoints are transformed to multi_frame."
  exit 1
fi

# ################################
# Bash: normalize legacy odom/map frames to odin_odom/odin_map
# ################################
if [[ "$frame" == "odom" ]]; then frame=odin_odom; fi
if [[ "$frame" == "map" ]]; then frame=odin_map; fi
if [[ "$frame" != "odin_odom" && "$frame" != "odin_map" ]]; then
  echo "Error: frame must be 'odin_odom' or 'odin_map' (got '$frame'; legacy 'odom'/'map' accepted)"
  exit 1
fi

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

ros2 launch vehicle_simulator system_real_robot.launch \
  enableCruise:=true \
  multi_enabled:=true \
  repeat_enabled:=false \
  multi_source:=$mode \
  multi_frame:=$frame \
  loop_count:=$loop_count \
  rvizWaypointTopic:=/way_point_cruise \
  "${map_args[@]}" \
  2>&1 | grep --line-buffered -E 'MULTI|CRUISE|WAYPOINT|WARN|ERROR'
