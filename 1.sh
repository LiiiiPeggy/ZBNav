docker start nav_gui
docker exec -it nav_gui bash
exit

sudo apt install ros-humble-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml


# LiDAR 数据采集与转换
sudo apt-get install libpcap-dev
cd ~/work/wyx/lqp/robosense_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch rslidar_sdk start.py

sudo apt install ros-humble-pcl-conversions libpcl-dev libeigen3-dev
cd ~/work/wyx/lqp/rs_converter_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch rs_converter rs_converter.launch.py

# SLAM 定位建图里程计
# 1. 编译安装 Livox-SDK2（生成 liblivox_lidar_sdk_shared.so）
cd ~/work/wyx/lqp/support/Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j
sudo make install

# 2. 编译 livox_ros_driver2
cd ~/work/wyx/lqp/supports/ws_livox/src/livox_ros_driver2
chmod 777 build.sh
./build.sh humble
cd ~/work/wyx/lqp/supports/ws_livox
source install/setup.bash

sudo apt install libgoogle-glog-dev libtbb-dev ros-humble-pcl-ros
# 3. 编译 SLAM
source install/setup.bash
source ~/work/wyx/lqp/supports/ws_livox/install/setup.bash
cd ~/work/wyx/lqp/SLAM
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch super_lio velodyne.py 


cd ~/work/wyx/lqp/Datasets
ros2 bag record -o my_bag --storage mcap /velodyne_points /Devices/Imu/Data
ros2 bag play my_bag --storage mcap

cd ~/work/wyx/lqp/cmu_planner
colcon build --symlink-install \
  --packages-skip vehicle_simulator velodyne_simulator velodyne_gazebo_plugins velodyne_description
ros2 launch super_lio_bridge bringup.launch.py


cd ~/work/wyx/lqp/odin_ws/src/odin_ros_driver/script
bash ./build_ros2.sh
cd ~/work/wyx/lqp/odin_ws
source install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py