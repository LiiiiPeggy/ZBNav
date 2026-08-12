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
  # map mode: set RViz Fixed Frame to 'map' before clicking waypoints (odom-frame clicks are rejected)
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
