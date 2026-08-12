#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# ################################
# Bash: launch multi-point cruise debug with filtered output
# ################################
mode=${1:-rviz}
loop_count=${2:--1}
frame=${3:-odom}

if [[ "$mode" != "yaml" && "$mode" != "rviz" ]]; then
  echo "Usage: bash 9multi_debug.sh [yaml|rviz] [loop_count] [odom|map]"
  echo "  bash 9multi_debug.sh             # default: RViz clicks, odom frame"
  echo "  bash 9multi_debug.sh yaml 3      # YAML route, odom frame, 3 loops"
  echo "  bash 9multi_debug.sh rviz -1 map # RViz clicks, map frame (relocalization)"
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
