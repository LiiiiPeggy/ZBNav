# OriGit - 工作区修改记录

> 生成时间：2026-07-13
> 排除：仿真日志文件 (metrics_*.txt, trajectory_*.txt)

---

## 一、已修改文件 (7)

### 1. [planner/src/local_planner/launch/local_planner.launch](planner/src/local_planner/launch/local_planner.launch)

| 参数 | 原值 | 新值 | 说明 |
|------|------|------|------|
| `twoWayDrive` | `true` | `false` | 禁用双向行驶 |
| `autonomyMode` | `true` | `false` | 禁用自主模式，切换为手动/外部控制 |

### 2. [planner/src/local_planner/src/localPlanner.cpp](planner/src/local_planner/src/localPlanner.cpp)

- 变量 `adjacentRange` 添加注释：`// 规划半径，m`
- 变量 `pathScale` 添加注释：`// 规划模板缩放`
- 数组 `clearPathList` 添加注释：`//可通行路径列表`
- 数组 `pathPenaltyList` 添加注释：`//路径惩罚列表`
- 添加注释掉的调试输出：`// std::cout << "++++++++++++" << newLaserCloud << newTerrainCloud << std::endl;`
- `relativeGoalDis` 添加注释：`//目标点的距离`
- `joyDir` 添加注释：`//目标点的角度`
- `#if PLOTPATHSET == 1` / `#endif` 宏缩进调整为顶格

### 3. [planner/src/local_planner/src/pathFollower.cpp](planner/src/local_planner/src/pathFollower.cpp)

- 发布速度的话题名从 `/cmd_vel` 改为 `/cmd_vel_stamped`
  ```cpp
  // 原：
  auto pubSpeed = nh->create_publisher<...>("/cmd_vel", 5);
  // 改为：
  auto pubSpeed = nh->create_publisher<...>("/cmd_vel_stamped", 5);
  ```

### 4. [planner/src/vehicle_simulator/CMakeLists.txt](planner/src/vehicle_simulator/CMakeLists.txt)

- 新增编译目标 `vehicleSimulatorAdapter`，链接依赖：`rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions gazebo_ros gazebo_msgs`
- 新增安装规则：`vehicleSimulatorAdapter DESTINATION lib/${PROJECT_NAME}`

### 5. [planner/src/vehicle_simulator/launch/system_indoor.launch](planner/src/vehicle_simulator/launch/system_indoor.launch)

- 新增 `start_vehicle_simulator_adapter` IncludeLaunchDescription，引用 `vehicle_simulator_adapter.launch`
- 原先启动的 `start_vehicle_simulator` 替换为 `start_vehicle_simulator_adapter`
- `start_joy` 被注释掉（禁用手柄节点）

### 6. [planner/src/vehicle_simulator/launch/vehicle_simulator.launch](planner/src/vehicle_simulator/launch/vehicle_simulator.launch)

| 参数 | 原值 | 新值 | 说明 |
|------|------|------|------|
| `adjustZ` | `true` | `false` | 禁用自动高度调整 |
| `adjustIncl` | `true` | `false` | 禁用自动倾角调整 |

### 7. [planner/src/vehicle_simulator/src/vehicleSimulator.cpp](planner/src/vehicle_simulator/src/vehicleSimulator.cpp)

- 订阅速度的话题名从 `/cmd_vel` 改为 `/cmd_vel_stamped`
  ```cpp
  // 原：
  auto subSpeed = nh->create_subscription<...>("/cmd_vel", 5, speedHandler);
  // 改为：
  auto subSpeed = nh->create_subscription<...>("/cmd_vel_stamped", 5, speedHandler);
  ```

---

## 二、新增文件 (3)

### 1. [planner/README_sim.md](planner/README_sim.md)

导航仿真指南文档，内容包括：
- 环境要求（Ubuntu 22.04, ROS2 Humble, GCC 11+）
- 依赖安装说明（gazebo-ros, joy）
- 构建步骤（planner + farplanner_ws）
- 运行指南（多终端启动、launch 参数表）
- 话题数据流图
- 故障排查表

### 2. [planner/src/vehicle_simulator/launch/vehicle_simulator_adapter.launch](planner/src/vehicle_simulator/launch/vehicle_simulator_adapter.launch)

新的 launch 文件，用于启动 `vehicleSimulatorAdapter` 节点。特点：
- 注释掉了 Gazebo 仿真相关节点（`start_gazebo`, `lidar_state_publisher`, `spawn_lidar`, `spawn_robot`, `spawn_camera`）
- 通过 `TimerAction` 延时 5 秒启动 `vehicleSimulatorAdapter`
- 默认 world_name 为 `garage`，robot_model 为 `panda3v2`

### 3. [planner/src/vehicle_simulator/src/vehicleSimulatorAdapter.cpp](planner/src/vehicle_simulator/src/vehicleSimulatorAdapter.cpp)

新的 ROS2 适配器节点（`VehicleSimulatorAdapter`），实现功能：

| 功能 | 话题 | 说明 |
|------|------|------|
| TF 广播 | `map` → `base_link` | 从 `/base_odom` 读取里程计，发布 TF 变换 |
| 里程计转发 | `/state_estimation` | 将 base_link 里程计转换为 velodyne 坐标系输出 |
| 点云坐标变换 | `/registered_scan` | 将 `/velodyne_points` 变换到 map 坐标系 |
| cmd_vel 桥接 | `/cmd_vel_stamped` → `/cmd_vel` | TwistStamped → Twist 类型转换 |
| 里程计历史 | 内存 | 保存最近 400 帧里程计数据 |
