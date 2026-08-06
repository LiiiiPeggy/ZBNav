# Patrol Cruise Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a patrol cruise controller node that drives the robot to a given waypoint, turns 180° in place, returns to the start position, and turns 180° again.

**Architecture:** A standalone C++ ROS2 node (`cruiseController`) that manages a 4-state patrol cycle. It publishes waypoints to `/way_point` for the travel phases, and uses the existing `/stop` topic to silence pathFollower during 180° in-place turns (publishing `/cmd_vel` directly for pure rotation).

**Tech Stack:** C++17, rclcpp, geometry_msgs, nav_msgs, std_msgs

## Global Constraints

- C++17 standard (matches odin_ros_driver)
- Node goes in `cmu_planner/src/local_planner/`
- Uses existing topics: `/way_point`, `/cmd_vel`, `/stop`, `/state_estimation`
- No changes to existing localPlanner or pathFollower logic
- Launch via `system_real_robot.launch`

---

## Current System Context

```
waypoint_tool → /way_point (PointStamped) → localPlanner.goalHandler → joyDir
                                                      ↓
                                            /path → pathFollower → /cmd_vel → robot

/stop (Int8) → pathFollower.stopHandler:
  1 → vehicleSpeed = 0 (stop moving, can still rotate)
  2 → vehicleSpeed = 0 AND vehicleYawRate = 0 (full stop)
  0 → normal operation
```

Key parameters:
- `maxYawRate = 45.0` deg/s → full 180° turn takes ~4 seconds
- `goalClearRange = 0.5` m → reached when within this distance
- pathFollower publishes to `/cmd_vel` (Twist) on topic `/cmd_vel`

---

## File Structure

| File | Action | Purpose |
|------|--------|---------|
| `cmu_planner/src/local_planner/src/cruiseController.cpp` | Create | Main node |
| `cmu_planner/src/local_planner/CMakeLists.txt` | Modify | Add executable |
| `cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch` | Modify | Add node to launch |

---

### Task 1: Create cruiseController node

**Files:**
- Create: `cmu_planner/src/local_planner/src/cruiseController.cpp`

**Interfaces:**
- Consumes: `/state_estimation` (Odometry), `/way_point` input from user
- Produces: `/way_point` (PointStamped, to localPlanner), `/stop` (Int8, to pathFollower), `/cmd_vel` (Twist, to robot)

**State machine:**

```
IDLE ──(activate)──→ GO_TO_DEST ──(arrived)──→ TURN_180
                                                    │
                                              (turn done)
                                                    │
START ←──(turn done)── TURN_180 ←──(arrived)── RETURN_TO_START
  │
  └──→ IDLE (or loop back to GO_TO_DEST)
```

- [ ] **Step 1: Create the cruiseController.cpp**

```cpp
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>

enum class CruiseState {
  IDLE = 0,
  GO_TO_DEST = 1,
  TURN_AT_DEST = 2,
  RETURN_TO_START = 3,
  TURN_AT_START = 4,
};

class CruiseController : public rclcpp::Node
{
public:
  CruiseController()
  : Node("cruise_controller"),
    state_(CruiseState::IDLE),
    start_x_(0.0), start_y_(0.0),
    dest_x_(0.0), dest_y_(0.0),
    turn_start_time_(0.0),
    turn_done_(false)
  {
    // Parameters
    double max_yaw_rate =
      this->declare_parameter<double>("max_yaw_rate", 45.0);
    turn_duration_ = M_PI / (max_yaw_rate * M_PI / 180.0);
    double goal_clear_range =
      this->declare_parameter<double>("goal_clear_range", 0.5);
    goal_clear_range_sq_ = goal_clear_range * goal_clear_range;

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10,
      std::bind(&CruiseController::odomCallback, this, std::placeholders::_1));

    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point_cruise", 10,
      std::bind(&CruiseController::waypointCallback, this, std::placeholders::_1));

    // Publishers
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/way_point", 10);
    stop_pub_ = this->create_publisher<std_msgs::msg::Int8>(
      "/stop", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    // Control timer (20 Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CruiseController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "Cruise controller ready. Send waypoint to /way_point_cruise to start.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    if (state_ == CruiseState::IDLE) {
      // Continuously update start position while idle
      start_x_ = current_x_;
      start_y_ = current_y_;
    }
  }

  void waypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (state_ != CruiseState::IDLE) {
      RCLCPP_WARN(this->get_logger(),
        "Already cruising, ignoring new waypoint");
      return;
    }
    dest_x_ = msg->point.x;
    dest_y_ = msg->point.y;
    RCLCPP_INFO(this->get_logger(),
      "Starting cruise: (%.1f, %.1f) → (%.1f, %.1f)",
      start_x_, start_y_, dest_x_, dest_y_);
    sendWaypointAndGo(dest_x_, dest_y_, CruiseState::GO_TO_DEST);
  }

  void sendWaypointAndGo(double x, double y, CruiseState next_state)
  {
    geometry_msgs::msg::PointStamped wp;
    wp.header.stamp = this->now();
    wp.header.frame_id = "map";
    wp.point.x = x;
    wp.point.y = y;
    wp.point.z = 0.0;
    waypoint_pub_->publish(wp);

    // Release pathFollower stop
    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 0;
    stop_pub_->publish(stop_msg);

    state_ = next_state;
    RCLCPP_INFO(this->get_logger(), "Waypoint set to (%.1f, %.1f)", x, y);
  }

  void startTurn(CruiseState next_state)
  {
    // Stop pathFollower from publishing cmd_vel
    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 2;  // full stop: vehicleSpeed=0, vehicleYawRate=0
    stop_pub_->publish(stop_msg);

    turn_start_time_ = this->now().seconds();
    turn_done_ = false;
    state_ = next_state;
    RCLCPP_INFO(this->get_logger(), "Starting 180-degree turn...");
  }

  void controlLoop()
  {
    double dx, dy, dist_sq;

    switch (state_) {
    case CruiseState::IDLE:
      return;

    case CruiseState::GO_TO_DEST:
      dx = current_x_ - dest_x_;
      dy = current_y_ - dest_y_;
      dist_sq = dx * dx + dy * dy;
      if (dist_sq < goal_clear_range_sq_) {
        RCLCPP_INFO(this->get_logger(), "Destination reached, turning...");
        startTurn(CruiseState::TURN_AT_DEST);
      }
      return;

    case CruiseState::TURN_AT_DEST:
      publishTurnCmd();
      if (turnDone()) {
        RCLCPP_INFO(this->get_logger(), "Turn done, returning to start...");
        sendWaypointAndGo(start_x_, start_y_, CruiseState::RETURN_TO_START);
      }
      return;

    case CruiseState::RETURN_TO_START:
      dx = current_x_ - start_x_;
      dy = current_y_ - start_y_;
      dist_sq = dx * dx + dy * dy;
      if (dist_sq < goal_clear_range_sq_) {
        RCLCPP_INFO(this->get_logger(), "Start reached, turning...");
        startTurn(CruiseState::TURN_AT_START);
      }
      return;

    case CruiseState::TURN_AT_START:
      publishTurnCmd();
      if (turnDone()) {
        RCLCPP_INFO(this->get_logger(), "Cruise complete!");
        state_ = CruiseState::IDLE;
      }
      return;
    }
  }

  void publishTurnCmd()
  {
    double max_yaw_rate =
      this->get_parameter("max_yaw_rate").as_double();
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = max_yaw_rate * M_PI / 180.0;
    cmd_vel_pub_->publish(cmd);
  }

  bool turnDone()
  {
    return (this->now().seconds() - turn_start_time_) >= turn_duration_;
  }

  // State
  CruiseState state_;
  double start_x_, start_y_, dest_x_, dest_y_;
  double current_x_, current_y_;
  double turn_start_time_;
  double turn_duration_;
  double goal_clear_range_sq_;
  bool turn_done_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CruiseController>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Add to CMakeLists.txt**

In `cmu_planner/src/local_planner/CMakeLists.txt`, add after the pathFollower executable block:

```cmake
add_executable(cruiseController
  src/cruiseController.cpp
)
ament_target_dependencies(cruiseController
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)
```

Add `cruiseController` to the `install(TARGETS ...)` list.

- [ ] **Step 3: Add to launch file**

In `system_real_robot.launch`, add after terrain nodes:

```python
cruise_controller_node = Node(
    package='local_planner',
    executable='cruiseController',
    name='cruise_controller',
    output='screen',
    parameters=[{
        'max_yaw_rate': 45.0,
        'goal_clear_range': 0.5,
    }]
)
```

Add `ld.add_action(cruise_controller_node)` to LaunchDescription.

- [ ] **Step 4: Build**

```bash
cd ~/work/wyx/lqp/cmu_planner
colcon build --symlink-install --packages-select local_planner
source install/setup.bash
```

- [ ] **Step 5: Verify manually**

```bash
# Publish a waypoint to start cruise (topic: /way_point_cruise, not /way_point)
ros2 topic pub /way_point_cruise geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 5.0, y: 0.0, z: 0.0}}" --once
```

Expected behavior:
1. Robot goes to (5, 0, 0)
2. Turns 180° in place (~4s at 45°/s)
3. Returns to start
4. Turns 180° again
5. Stops, ready for next command

- [ ] **Step 6: Commit**

```bash
git add cmu_planner/src/local_planner/src/cruiseController.cpp \
        cmu_planner/src/local_planner/CMakeLists.txt \
        cmu_planner/src/vehicle_simulator/launch/system_real_robot.launch
git commit -m "feat: add cruise controller for patrol round-trip with U-turns"
```
