#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ################################
# Bash: publish saved Odin map for standalone visualization
# ################################
cd "$SCRIPT_DIR"

ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="src/odin_ros_driver/map/map_20260807_151455.pcd" \
  -p tf_frame:=odin_map \
  -p publishing_period_ms:=10000 \
  -r cloud_pcd:=/overall_map &

# ################################
# Bash: clean up map publisher when standalone viewer exits
# ################################
cleanup() {
  pkill -f 'pcd_to_pointcloud' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Launch RViz immediately: the publisher is a periodic volatile publisher,
# so subscribing now catches the first ~10 s tick (no sleep).
ros2 run rviz2 rviz2 -d \
  "$SCRIPT_DIR/src/odin_ros_driver/config/overall_map.rviz"
