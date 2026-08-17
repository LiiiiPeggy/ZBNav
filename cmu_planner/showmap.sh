#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# ################################
# Bash: show saved odin map point cloud on /overall_map
# ################################
# pcd_to_pointcloud resolves file_name relative to cwd — cd into SLAM workspace
cd "$(dirname "$0")/../SLAM"

ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="src/odin_ros_driver/map/map_20260807_151455.pcd" \
  -p tf_frame:=odin_map \
  -p publishing_period_ms:=10000 \
  -r cloud_pcd:=/overall_map
