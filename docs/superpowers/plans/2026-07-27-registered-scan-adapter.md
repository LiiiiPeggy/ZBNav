# Registered Scan Adapter Node Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a C++ ROS 2 node that converts Odin's `/odin1/cloud_slam` (PointXYZRGB, `frame_id: odom`) to `/registered_scan` (PointXYZI), with configurable close-range point filtering.

**Architecture:** Single standard `ament_cmake` executable in the `odin_ros_driver` package. Subscribes to `/odin1/cloud_slam` + `/state_estimation` (for sensor position), filters points closer than `scan_min_range`, converts XYZRGB→XYZI with `intensity=0`, publishes to `/registered_scan`. Uses PCL for point cloud processing.

**Tech Stack:** C++17, ROS 2 Humble, PCL, rclcpp, pcl_conversions

## Global Constraints

- Do NOT modify CMU Planner packages (local_planner, terrain_analysis, terrain_analysis_ext)
- Use standard ROS 2 C++ node pattern matching existing odin_ros_driver style
- Output intensity MUST be `0.0f` (not RGB-encoded value)
- Distance filtering MUST use sensor position from `/state_estimation` (points are in `odom` frame)
- Delete the Python relay script created earlier at `script/xyzrgb_to_xyzi.py`

## File Map

| Action | File | Purpose |
|--------|------|---------|
| Create | `SLAM/src/odin_ros_driver/src/registered_scan_adapter_node.cpp` | Adapter node source |
| Modify | `SLAM/src/odin_ros_driver/CMakeLists.txt` | Add executable + install target |
| Modify | `SLAM/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py` | Replace cloud_slam remap with adapter node |
| Delete | `SLAM/src/odin_ros_driver/script/xyzrgb_to_xyzi.py` | Remove unused Python relay |

---

### Task 1: Create the adapter node source file

**Files:**
- Create: `SLAM/src/odin_ros_driver/src/registered_scan_adapter_node.cpp`

**Interfaces:**
- Consumes: `/odin1/cloud_slam` (PointCloud2, fields: x/y/z/rgb, frame_id: odom), `/state_estimation` (Odometry)
- Produces: `/registered_scan` (PointCloud2, fields: x/y/z/intensity)
- Parameters: `scan_min_range` (double, default 0.0), `input_topic` (string), `output_topic` (string)

- [ ] **Step 1: Write the source file**

```cpp
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class RegisteredScanAdapter : public rclcpp::Node
{
public:
  RegisteredScanAdapter()
  : Node("registered_scan_adapter"),
    sensor_x_(0.0), sensor_y_(0.0), sensor_z_(0.0),
    odom_received_(false)
  {
    scan_min_range_ =
      this->declare_parameter<double>("scan_min_range", 0.0);

    input_topic_ =
      this->declare_parameter<std::string>(
        "input_topic", "/odin1/cloud_slam");

    output_topic_ =
      this->declare_parameter<std::string>(
        "output_topic", "/registered_scan");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();

    publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, qos);

    subscription_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_,
        qos,
        std::bind(
          &RegisteredScanAdapter::cloudCallback,
          this,
          std::placeholders::_1));

    odom_sub_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimation",
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        std::bind(
          &RegisteredScanAdapter::odomCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Point cloud adapter: %s -> %s, scan_min_range=%.3f m",
      input_topic_.c_str(),
      output_topic_.c_str(),
      scan_min_range_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    sensor_x_ = msg->pose.pose.position.x;
    sensor_y_ = msg->pose.pose.position.y;
    sensor_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
    pcl::fromROSMsg(*msg, input_cloud);

    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    output_cloud.header = input_cloud.header;
    output_cloud.is_dense = false;
    output_cloud.points.reserve(input_cloud.points.size());

    const float min_range_sq =
      static_cast<float>(scan_min_range_ * scan_min_range_);
    const bool do_filter = (scan_min_range_ > 0.0);
    const float sx = static_cast<float>(sensor_x_);
    const float sy = static_cast<float>(sensor_y_);
    const float sz = static_cast<float>(sensor_z_);

    for (const auto & input_point : input_cloud.points) {
      if (!std::isfinite(input_point.x) ||
          !std::isfinite(input_point.y) ||
          !std::isfinite(input_point.z))
      {
        continue;
      }

      if (do_filter && odom_received_) {
        // Points are in odom frame — subtract sensor position
        const float dx = input_point.x - sx;
        const float dy = input_point.y - sy;
        const float dz = input_point.z - sz;
        const float range_sq = dx * dx + dy * dy + dz * dz;

        if (range_sq < min_range_sq) {
          continue;
        }
      }

      pcl::PointXYZI output_point;
      output_point.x = input_point.x;
      output_point.y = input_point.y;
      output_point.z = input_point.z;
      output_point.intensity = 0.0f;

      output_cloud.points.push_back(output_point);
    }

    output_cloud.width =
      static_cast<std::uint32_t>(output_cloud.points.size());
    output_cloud.height = 1;

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header = msg->header;

    publisher_->publish(output_msg);
  }

  double scan_min_range_;
  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    subscription_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    publisher_;

  double sensor_x_, sensor_y_, sensor_z_;
  bool odom_received_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RegisteredScanAdapter>());
  rclcpp::shutdown();
  return 0;
}
```

---

### Task 2: Update CMakeLists.txt — add executable build target and install

**Files:**
- Modify: `SLAM/src/odin_ros_driver/CMakeLists.txt`

- [ ] **Step 1: Add build target**

After line 397 (the `)` closing `ament_target_dependencies` of `image_overlay_node`), insert:

```cmake

    add_executable(registered_scan_adapter_node
      src/registered_scan_adapter_node.cpp
    )

    ament_target_dependencies(registered_scan_adapter_node
      rclcpp
      sensor_msgs
      nav_msgs
      pcl_conversions
    )

    target_link_libraries(registered_scan_adapter_node
      ${PCL_LIBRARIES}
    )
```

- [ ] **Step 2: Add to install(TARGETS ...) list**

In the `install(TARGETS` block (~lines 401-413), add `registered_scan_adapter_node` to the target list:

```cmake
    install(TARGETS
        host_sdk_sample
        pcd2depth_ros2_node
        cloud_reprojection_ros2_node
        image_overlay_node
        registered_scan_adapter_node
        depth_image_ros2_node_lib
        pointcloud_depth_converter_ros2
        cloud_reprojector_ros2
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
```

---

### Task 3: Update launch file

**Files:**
- Modify: `SLAM/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py`

- [ ] **Step 1: Remove cloud_slam remap from host_sdk_node (line 41)**

```python
# Before:
        remappings=[
            ('odin1/odometry', '/state_estimation'),
            ('odin1/cloud_slam', '/registered_scan'),
        ]

# After:
        remappings=[
            ('odin1/odometry', '/state_estimation'),
        ]
```

- [ ] **Step 2: Add adapter node definition**

After the `image_overlay_node` block (~line 82), insert:

```python
    # Registered scan adapter — XYZRGB→XYZI conversion + close-range filter
    registered_scan_adapter_node = Node(
        package='odin_ros_driver',
        executable='registered_scan_adapter_node',
        name='registered_scan_adapter',
        output='screen',
        parameters=[{
            'scan_min_range': 0.5,
            'input_topic': '/odin1/cloud_slam',
            'output_topic': '/registered_scan',
        }],
    )
```

- [ ] **Step 3: Add to LaunchDescription**

Insert after `ld.add_action(host_sdk_node)`:

```python
    ld.add_action(host_sdk_node)
    ld.add_action(registered_scan_adapter_node)
```

---

### Task 4: Delete Python relay script

**Files:**
- Delete: `SLAM/src/odin_ros_driver/script/xyzrgb_to_xyzi.py`

- [ ] **Step 1: Delete the file**

```bash
rm SLAM/src/odin_ros_driver/script/xyzrgb_to_xyzi.py
```

---

### Task 5: Build and verify

- [ ] **Step 1: Build the package**

```bash
cd ~/work/wyx/lqp/SLAM
source ~/work/wyx/lqp/supports/ws_livox/install/setup.bash
colcon build --symlink-install --packages-select odin_ros_driver
source install/setup.bash
```

Expected: Build succeeds with no errors.

- [ ] **Step 2: Launch and verify**

```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

In another terminal:

```bash
# Verify output fields
ros2 topic echo /registered_scan --once --field fields
# Expected: x, y, z, intensity — no 'rgb' field

# Verify only one publisher
ros2 topic info /registered_scan -v
# Expected: 1 publisher (registered_scan_adapter)

# Verify no intensity warnings
ros2 launch vehicle_simulator system_real_robot.launch
# terrain_analysis output should NOT show "failed to find field 'intensity'"
```

- [ ] **Step 3: Verify min_range filtering**

```bash
# With scan_min_range=0.5, verify points within 0.5m are excluded
ros2 topic echo /registered_scan --once --field width
# Compare with raw odin1/cloud_slam point count
ros2 topic echo /odin1/cloud_slam --once --field width
# registered_scan should have fewer points if any were within 0.5m
```

---

### Task 6: Commit

```bash
cd ~/Codes_rk
git add SLAM/src/odin_ros_driver/src/registered_scan_adapter_node.cpp
git add SLAM/src/odin_ros_driver/CMakeLists.txt
git add SLAM/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py
git rm SLAM/src/odin_ros_driver/script/xyzrgb_to_xyzi.py
git commit -m "Add registered_scan_adapter_node: XYZRGB→XYZI + close-range filter

- Converts Odin /odin1/cloud_slam (PointXYZRGB) to /registered_scan (PointXYZI)
- Filters points closer than scan_min_range (default 0.5m)
- Uses /state_estimation for sensor position (points are in odom frame)
- Replaces direct cloud_slam→registered_scan remap

Co-Authored-By: Claude <noreply@anthropic.com>"
```
