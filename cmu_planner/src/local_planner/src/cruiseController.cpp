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
    current_x_(0.0), current_y_(0.0),
    current_yaw_(0.0),
    target_yaw_(0.0)
  {
    this->declare_parameter<double>("max_yaw_rate", 45.0);
    this->declare_parameter<double>("yaw_kp", 1.5);
    this->declare_parameter<double>("yaw_tolerance", 0.12);
    this->declare_parameter<double>("goal_clear_range", 0.5);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10,
      std::bind(&CruiseController::odomCallback, this, std::placeholders::_1));

    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point_cruise", 10,
      std::bind(&CruiseController::waypointCallback, this, std::placeholders::_1));

    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/way_point", 10);
    stop_pub_ = this->create_publisher<std_msgs::msg::Int8>(
      "/stop", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CruiseController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "Cruise controller ready. Send waypoint to /way_point_cruise to start.");
  }

private:
  double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);

    if (state_ == CruiseState::IDLE) {
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
      "Starting cruise: (%.1f, %.1f) -> (%.1f, %.1f)",
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

    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 0;
    stop_pub_->publish(stop_msg);

    state_ = next_state;
    RCLCPP_INFO(this->get_logger(), "Waypoint set to (%.1f, %.1f)", x, y);
  }

  void startTurn(CruiseState next_state)
  {
    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 2;
    stop_pub_->publish(stop_msg);

    target_yaw_ = normalizeAngle(current_yaw_ + M_PI);
    state_ = next_state;
    RCLCPP_INFO(this->get_logger(),
      "Starting 180-degree turn, target_yaw=%.2f (current=%.2f)",
      target_yaw_, current_yaw_);
  }

  void publishTurnCmd()
  {
    double yaw_error = normalizeAngle(target_yaw_ - current_yaw_);

    double max_yaw_rate =
      this->get_parameter("max_yaw_rate").as_double() * M_PI / 180.0;
    double yaw_kp = this->get_parameter("yaw_kp").as_double();

    double wz = yaw_kp * yaw_error;
    if (wz > max_yaw_rate) wz = max_yaw_rate;
    if (wz < -max_yaw_rate) wz = -max_yaw_rate;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = wz;
    cmd_vel_pub_->publish(cmd);
  }

  bool turnDone()
  {
    double yaw_error = normalizeAngle(target_yaw_ - current_yaw_);
    return std::fabs(yaw_error) <
      this->get_parameter("yaw_tolerance").as_double();
  }

  void controlLoop()
  {
    double dx, dy, dist_sq;
    double goal_clear_range = this->get_parameter("goal_clear_range").as_double();
    double goal_clear_range_sq = goal_clear_range * goal_clear_range;

    switch (state_) {
    case CruiseState::IDLE:
      return;

    case CruiseState::GO_TO_DEST:
      dx = current_x_ - dest_x_;
      dy = current_y_ - dest_y_;
      dist_sq = dx * dx + dy * dy;
      if (dist_sq < goal_clear_range_sq) {
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
      if (dist_sq < goal_clear_range_sq) {
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

  CruiseState state_;
  double start_x_, start_y_, dest_x_, dest_y_;
  double current_x_, current_y_, current_yaw_;
  double target_yaw_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CruiseController>());
  rclcpp::shutdown();
  return 0;
}
