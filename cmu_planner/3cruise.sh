#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch vehicle_simulator system_real_robot.launch \
  enableCruise:=true \
  rvizWaypointTopic:=/way_point_cruise
