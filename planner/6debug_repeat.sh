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
  loop_count:=$loop_count
