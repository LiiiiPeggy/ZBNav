#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# ################################
# Bash: launch multi-point cruise with yaml/rviz waypoint source
# ################################
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
