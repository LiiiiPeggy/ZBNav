#!/bin/bash
# ============================================================
# ZBNav 完整部署与运行脚本
# 工作目录: ~/work/wyx/lqp
# ============================================================

# ============================================================
# 0. Docker 与可视化工具
# ============================================================
# MobaXterm SSH X11 转发：容器 --net=host 下访问不到 localhost 隧道，
# 需获取 MobaXterm 所在 Windows 的真实 IP 作为 DISPLAY 地址。
xhost + 2>/dev/null
DISPLAY_IP=$(echo $SSH_CLIENT | awk '{print $1}')
if [ -z "$DISPLAY_IP" ]; then
    DISPLAY_IP="localhost"
fi
echo "DISPLAY_IP=$DISPLAY_IP"

# 删除旧容器
docker stop nav_gui 2>/dev/null; docker rm nav_gui 2>/dev/null

# -------- 版本1：狗1（旧版本 3dnav 镜像） --------
# 重新创建容器，挂载 Xauthority 文件并传递 DISPLAY
# docker run -it \
#     --net=host \
#     --privileged \
#     -e DISPLAY=${DISPLAY_IP}:0.0 \
#     -e QT_X11_NO_MITSHM=1 \
#     -e ROS_DOMAIN_ID=71 \
#     -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/env/ros2/rk3588_eth_binding.xml \
#     -e TZ=Asia/Shanghai \
#     -e LANG=en_US.UTF-8 \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v $HOME/.Xauthority:/root/.Xauthority:ro \
#     -v /home/siasun/panda3_2026_06_16/panda3/docker/env/ros2:/root/env/ros2:ro \
#     -v /home/siasun/panda3_2026_06_16/panda3/src/wyx:/root/work/wyx \
#     --name nav_gui \
#     3dnav \
#     bash

# -------- 版本2：狗2（当前版本） --------
docker run -it \
    --net=host \
    --privileged \
    -e DISPLAY=${DISPLAY_IP}:0.0 \
    -e QT_X11_NO_MITSHM=1 \
    -e ROS_DOMAIN_ID=60 \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/env/ros2/rk3588_eth_binding.xml \
    -e TZ=Asia/Shanghai \
    -e LANG=en_US.UTF-8 \
    -e OMP_NUM_THREADS=1 \
    -e MKL_NUM_THREADS=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -v /dev:/dev:rw \
    -v /home/siasun/panda3_2026_07_17/panda3/docker/env/ros2:/root/env/ros2:ro \
    -v /home/siasun/panda3_2026_07_17/panda3/src/wyx:/root/work/wyx \
    --name nav_gui \
    192.168.1.29/robot-dog/bottom-control:2026-2-04-1 \
    bash

# 后续启动已创建的容器
docker start nav_gui
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

docker exec -it nav_gui bash
# exit

# Foxglove Bridge (可选)
# sudo apt install ros-humble-foxglove-bridge
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml


# ============================================================
# 1. 系统依赖 (一次性)
# ============================================================
sudo apt update && apt install -y x11-apps libpcap-dev ros-humble-pcl-conversions libpcl-dev libeigen3-dev libgoogle-glog-dev libtbb-dev ros-humble-pcl-ros libceres-dev ros-humble-rmw-cyclonedds-cpp usbutils
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============================================================
# 2. 编译 Supports (Livox SDK + ROS2 Driver)
# ============================================================
# Livox-SDK2
cd ~/work/wyx/lqp/supports/Livox-SDK2
mkdir -p build && cd build && cmake .. && make -j && sudo make install

# livox_ros_driver2
cd ~/work/wyx/lqp/supports/ws_livox/src/livox_ros_driver2
chmod 777 build.sh && ./build.sh humble
cd ~/work/wyx/lqp/supports/ws_livox && source install/setup.bash


# ============================================================
# 3. 编译 robosense_ws (LiDAR 驱动)
# ============================================================
cd ~/work/wyx/lqp/robosense_ws
colcon build --symlink-install 
source install/setup.bash


# ============================================================
# 4. 编译 rs_converter_ws (格式转换)
# ============================================================
cd ~/work/wyx/lqp/rs_converter_ws
colcon build --symlink-install 
source install/setup.bash


# ============================================================
# 5. 编译 SLAM (basic → super_lio → odin_ros_driver)
# ============================================================
source ~/work/wyx/lqp/supports/ws_livox/install/setup.bash
cd ~/work/wyx/lqp/SLAM/src/odin_ros_driver/script
bash ./build_ros2.sh

cd ~/work/wyx/lqp/SLAM
colcon build --symlink-install  --packages-select basic
colcon build --symlink-install  --packages-select super_lio
source install/setup.bash


# ============================================================
# 6. 编译 cmu_planner (规划栈)
# ============================================================
cd ~/work/wyx/lqp/cmu_planner
colcon build --symlink-install
source install/setup.bash


# ============================================================
# 7. 启动节点
# ============================================================

# --- 方案 A: RoboSense LiDAR + super_lio + CMU 规划 ---
# 终端1: LiDAR 驱动 + 格式转换
source ~/work/wyx/lqp/robosense_ws/install/setup.bash
ros2 launch rslidar_sdk start.py &
sleep 2
source ~/work/wyx/lqp/rs_converter_ws/install/setup.bash
ros2 launch rs_converter rs_converter.launch.py &

# 终端2: super_lio
source ~/work/wyx/lqp/SLAM/install/setup.bash
ros2 launch super_lio velodyne.py    # LiDAR: /velodyne_points, IMU: /Devices/Imu/Data

# 终端3: CMU 规划 (已移除 super_lio_bridge，topic 由 remap 直连)
source ~/work/wyx/lqp/cmu_planner/install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch


# --- 方案 B: Odin 自带 SLAM + CMU 规划 (推荐实机) ---
# 终端1: Odin ROS 驱动 (topic 已 remap → /state_estimation + /registered_scan)
source ~/work/wyx/lqp/SLAM/install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py

# 终端2: CMU 规划
source ~/work/wyx/lqp/cmu_planner/install/setup.bash
ros2 launch vehicle_simulator system_real_robot.launch


# --- 方案 C: RoboSense LiDAR + super_lio (仅 SLAM，无规划) ---
source ~/work/wyx/lqp/robosense_ws/install/setup.bash
ros2 launch rslidar_sdk start.py &
sleep 2
source ~/work/wyx/lqp/rs_converter_ws/install/setup.bash
ros2 launch rs_converter rs_converter.launch.py &
sleep 2
source ~/work/wyx/lqp/SLAM/install/setup.bash
ros2 launch super_lio velodyne.py


# ============================================================
# 8. rosbag 录制与回放
# ============================================================
# 录制
cd ~/work/wyx/lqp/Datasets
ros2 bag record -o my_bag --storage mcap /velodyne_points /Devices/Imu/Data /state_estimation /registered_scan

# 回放
ros2 bag play my_bag --storage mcap

# 回放时处理 QoS (如果 bag 的 QoS 和节点不匹配)
ros2 bag play my_bag --storage mcap --qos-profile-overrides-path ~/work/wyx/lqp/SLAM/src/odin_ros_driver/script/rosbag2_qos.yaml


# ============================================================
# 9. 运行时数据频率检查
# ============================================================
# SLAM 输出 → CMU 规划输入
ros2 topic hz /state_estimation      # 期望 ~10 Hz
ros2 topic hz /registered_scan       # 期望 ~10 Hz

# CMU 规划输出
ros2 topic hz /path                  # 期望 ~10 Hz
ros2 topic hz /terrain_map            # 期望 1-5 Hz
ros2 topic hz /terrain_map_ext        # 期望 1-5 Hz
ros2 topic hz /cmd_vel               # 期望 有手柄控制时 10-50 Hz

# 原始 LiDAR 数据
ros2 topic hz /rslidar_points        # 期望 ~10 Hz
ros2 topic hz /velodyne_points       # 期望 ~10 Hz

# 手柄
ros2 topic hz /joy                   # 期望 20-50 Hz