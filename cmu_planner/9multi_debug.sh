#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# ################################
# Bash: launch multi-point cruise debug with filtered output
# ################################
mode=${1:-rviz}
loop_count=${2:--1}
frame=${3:-odin_odom}

if [[ "$mode" != "yaml" && "$mode" != "rviz" ]]; then
  echo "Usage: bash 9multi_debug.sh [yaml|rviz] [loop_count] [odin_odom|odin_map]"
  echo "  bash 9multi_debug.sh                  # default: RViz clicks, odin_odom frame"
  echo "  bash 9multi_debug.sh yaml 3           # YAML route, odin_odom frame, 3 loops"
  echo "  bash 9multi_debug.sh rviz -1 odin_map # RViz clicks, odin_map frame (relocalization)"
  echo "Note: legacy 'odom'/'map' arguments are accepted and normalized."
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

ros2 launch vehicle_simulator system_real_robot.launch \
  enableCruise:=true \
  multi_enabled:=true \
  repeat_enabled:=false \
  multi_source:=$mode \
  multi_frame:=$frame \
  loop_count:=$loop_count \
  rvizWaypointTopic:=/way_point_cruise \
  2>&1 | grep --line-buffered -E 'MULTI|CRUISE|WAYPOINT|WARN|ERROR'
