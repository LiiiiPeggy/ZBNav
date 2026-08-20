#!/bin/bash
set -e

source install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch
