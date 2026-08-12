#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash

# ################################
# Bash: start multi-point cruise via /multi_start service
# ################################
# Prerequisite: waypoints already placed in RViz (>=2), run in another terminal:
#   bash 8multi_start.sh
# Response: success: true, message: Starting multi cruise
ros2 service call /multi_start std_srvs/srv/Trigger "{}"
