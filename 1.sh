#!/bin/bash
# ============================================================
# ZBNav 完整部署与运行脚本
# 工作目录: ~/Codes_rk
# ============================================================

# ============================================================
# 0. Docker 与可视化工具
# ============================================================
# docker start nav_gui && docker exec -it nav_gui bash
# exit

# Foxglove Bridge (可选)
# sudo apt install ros-humble-foxglove-bridge
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml


# ============================================================
# 1. 系统依赖 (一次性)
# ============================================================
sudo apt install -y libpcap-dev ros-humble-pcl-conversions libpcl-dev libeigen3-dev \
  libgoogle-glog-dev libtbb-dev ros-humble-pcl-ros libceres-dev


# ============================================================
# 2. 编译 Supports (Livox SDK + ROS2 Driver)
# ============================================================
# Livox-SDK2
cd ~/Codes_rk/supports/Livox-SDK2
mkdir -p build && cd build && cmake .. && make -j && sudo make install

# livox_ros_driver2
cd ~/Codes_rk/supports/ws_livox/src/livox_ros_driver2
chmod 777 build.sh && ./build.sh humble
cd ~/Codes_rk/supports/ws_livox && source install/setup.bash


# ============================================================
# 3. 编译 robosense_ws (LiDAR 驱动)
# ============================================================
cd ~/Codes_rk/robosense_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash


# ============================================================
# 4. 编译 rs_converter_ws (格式转换)
# ============================================================
cd ~/Codes_rk/rs_converter_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash


# ============================================================
# 5. 编译 SLAM (basic → super_lio → odin_ros_driver)
# ============================================================
source ~/Codes_rk/supports/ws_livox/install/setup.bash
cd ~/Codes_rk/SLAM
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select basic
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select super_lio
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select odin_ros_driver
source install/setup.bash


# ============================================================
# 6. 编译 cmu_planner (规划栈)
# ============================================================
cd ~/Codes_rk/cmu_planner
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip vehicle_simulator velodyne_simulator velodyne_gazebo_plugins velodyne_description
source install/setup.bash


# ============================================================
# 7. 启动节点
# ============================================================

# --- 方案 A: RoboSense LiDAR + super_lio + CMU 规划 ---
# 终端1: LiDAR 驱动 + 格式转换
source ~/Codes_rk/robosense_ws/install/setup.bash
ros2 launch rslidar_sdk start.py &
sleep 2
source ~/Codes_rk/rs_converter_ws/install/setup.bash
ros2 launch rs_converter rs_converter.launch.py &

# 终端2: super_lio
source ~/Codes_rk/SLAM/install/setup.bash
ros2 launch super_lio velodyne.py    # LiDAR: /velodyne_points, IMU: /Devices/Imu/Data

# 终端3: CMU 规划 (已移除 super_lio_bridge，topic 由 remap 直连)
source ~/Codes_rk/cmu_planner/install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch


# --- 方案 B: Odin 自带 SLAM + CMU 规划 (推荐实机) ---
# 终端1: Odin ROS 驱动 (topic 已 remap → /state_estimation + /registered_scan)
source ~/Codes_rk/SLAM/install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py

# 终端2: CMU 规划
source ~/Codes_rk/cmu_planner/install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch


# --- 方案 C: RoboSense LiDAR + super_lio (仅 SLAM，无规划) ---
source ~/Codes_rk/robosense_ws/install/setup.bash
ros2 launch rslidar_sdk start.py &
sleep 2
source ~/Codes_rk/rs_converter_ws/install/setup.bash
ros2 launch rs_converter rs_converter.launch.py &
sleep 2
source ~/Codes_rk/SLAM/install/setup.bash
ros2 launch super_lio velodyne.py


# ============================================================
# 8. rosbag 录制与回放
# ============================================================
# 录制
cd ~/Codes_rk/Datasets
ros2 bag record -o my_bag --storage mcap /velodyne_points /Devices/Imu/Data /state_estimation /registered_scan

# 回放
ros2 bag play my_bag --storage mcap

# 回放时处理 QoS (如果 bag 的 QoS 和节点不匹配)
ros2 bag play my_bag --storage mcap --qos-profile-overrides-path ~/Codes_rk/SLAM/src/odin_ros_driver/script/rosbag2_qos.yaml


# ============================================================
# 9. 运行时数据频率检查
# ============================================================
echo ""
echo "==================== 数据链路检查 ===================="
echo ""

# 9.1 检查关键 topic 是否存在
echo ">>> 关键 Topic 列表:"
ros2 topic list 2>/dev/null | grep -E "state_estimation|registered_scan|path|terrain_map|cmd_vel|way_point|joy|velodyne_points|rslidar_points"

# 9.2 逐个检查频率 (Ctrl+C 停止)
echo ""
echo ">>> 检查各 Topic 频率 (每 3 秒采样):"
echo ""

check_hz() {
    local topic=$1
    local expected=$2
    echo -n "  $topic (期望 $expected): "
    local result=$(timeout 4 ros2 topic hz "$topic" 2>&1 | tail -1)
    if [ -z "$result" ]; then
        echo "❌ 无数据或无发布者"
    else
        echo "$result"
    fi
}

# Odin / super_lio 输出 (remap 后汇入 CMU)
check_hz "/state_estimation"    "~10 Hz"
check_hz "/registered_scan"     "~10 Hz"

# CMU 规划输出
check_hz "/path"                 "~10 Hz"
check_hz "/terrain_map"          "1-5 Hz"
check_hz "/terrain_map_ext"      "1-5 Hz"
check_hz "/cmd_vel"              "手柄触发后 10-50 Hz"

# LiDAR 原始数据
check_hz "/rslidar_points"       "~10 Hz"
check_hz "/velodyne_points"      "~10 Hz"

# 手柄
check_hz "/joy"                  "20-50 Hz"

# 9.3 检查 QoS 匹配
echo ""
echo ">>> QoS 检查:"
for t in /state_estimation /registered_scan /path; do
    echo "  $t:"
    ros2 topic info "$t" 2>/dev/null | grep -E "Publisher|Subscription|Reliability" | sed 's/^/    /'
done

# 9.4 检查内容非空
echo ""
echo ">>> 内容抽查 (echo --once):"
ros2 topic echo /state_estimation --once --field pose 2>/dev/null | head -3
ros2 topic echo /registered_scan --once --field header 2>/dev/null | head -3

echo ""
echo "==================== 检查完毕 ===================="
