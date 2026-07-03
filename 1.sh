sudo apt install ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml


# LiDAR 数据采集与转换
cd ~/work/wyx/lqp/Codes/robosense_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch rslidar_sdk start.py

cd ~/work/wyx/lqp/Codes/rs_converter_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch rs_converter rs_converter.launch.py

# SLAM 定位建图里程计
# 1. 编译安装 Livox-SDK2（生成 liblivox_lidar_sdk_shared.so）
cd ~/work/wyx/lqp/Codes/support/Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j
sudo make install

# 2. 编译 livox_ros_driver2
cd ~/work/wyx/lqp/Codes/supports/ws_livox/src/livox_ros_driver2
./build.sh humble
cd ~/work/wyx/lqp/Codes/supports/ws_livox
source install/setup.bash

# 3. 编译 Super-LIO
source install/setup.bash
source ~/work/wyx/lqp/Codes/supports/ws_livox/install/setup.bash
cd ~/work/wyx/lqp/Codes/Super-LIO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch super_lio velodyne.py 


cd ~/work/wyx/lqp/Datasets
ros2 bag record -o test /velodyne_points /Devices/Imu/Data