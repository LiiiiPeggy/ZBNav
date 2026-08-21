#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>

// ################################
// C++: add MULTI mode includes
// ################################
#include <vector>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/bool.hpp>

enum class CruiseState {
  IDLE = 0,
  GO_TO_DEST = 1,
  TURN_AT_DEST = 2,
  RETURN_TO_START = 3,
  TURN_AT_START = 4,
  // ################################
  // C++: extend state enum for MULTI states
  // ################################
  WAIT_LOCALIZATION = 5,
  COLLECTING_WAYPOINTS = 6,
  GO_TO_WAYPOINT = 7,
  TURN_AT_WAYPOINT = 8,
  WAIT_AT_WAYPOINT = 9,
  // ################################
  // C++: add ready-to-start state shared by all cruise modes
  // ################################
  READY_TO_START = 10,
};

// ################################
// C++: add MULTI waypoint struct
// ################################
struct Waypoint {
  double x, y;
  double turn_angle = 0.0;
  double wait_time = 2.0;
};

class CruiseController : public rclcpp::Node
{
public:
  CruiseController()
  : Node("cruise_controller"),
    // ################################
    // C++: init tf2 buffer and listener
    // ################################
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    state_(CruiseState::IDLE),
    has_odom_(false),
    start_x_(0.0), start_y_(0.0),
    dest_x_(0.0), dest_y_(0.0),
    current_x_(0.0), current_y_(0.0),
    current_yaw_(0.0),
    target_yaw_(0.0)
  {
    this->declare_parameter<double>("max_yaw_rate", 45.0);
    this->declare_parameter<double>("min_yaw_rate", 0.32);
    this->declare_parameter<double>("yaw_kp", 1.5);
    this->declare_parameter<double>("yaw_tolerance", 0.12);
    this->declare_parameter<double>("goal_clear_range", 0.5);
    this->declare_parameter<double>("turn_angle", 180.0);
    this->declare_parameter<bool>("repeat_enabled", false);
    this->declare_parameter<int>("loop_count", -1);

    // ################################
    // C++: declare MULTI parameters
    // ################################
    this->declare_parameter<bool>("multi_enabled", false);
    this->declare_parameter<std::string>("multi_source", "yaml");
    this->declare_parameter<std::string>("multi_frame", "odin_odom");
    this->declare_parameter<std::string>("planning_frame", "odin_odom");
    this->declare_parameter<std::string>("global_frame", "odin_map");
    this->declare_parameter<std::string>("multi_route_file", "");
    this->declare_parameter<double>("default_wait_time", 2.0);

    turn_angle_ = this->get_parameter("turn_angle").as_double();
    min_yaw_rate_ = this->get_parameter("min_yaw_rate").as_double();
    repeat_enabled_ = this->get_parameter("repeat_enabled").as_bool();
    loop_count_ = this->get_parameter("loop_count").as_int();

    // ################################
    // C++: read MULTI parameters and validate
    // ################################
    multi_enabled_ = this->get_parameter("multi_enabled").as_bool();
    multi_source_ = this->get_parameter("multi_source").as_string();
    multi_frame_ = this->get_parameter("multi_frame").as_string();
    planning_frame_ = this->get_parameter("planning_frame").as_string();
    global_frame_ = this->get_parameter("global_frame").as_string();
    std::string mrf = this->get_parameter("multi_route_file").as_string();
    multi_route_file_ = mrf.empty()
      ? ament_index_cpp::get_package_share_directory("local_planner") + "/config/multi_route.yaml"
      : mrf;
    default_wait_time_ = this->get_parameter("default_wait_time").as_double();

    // Validate multi_source
    if (multi_source_ != "yaml" && multi_source_ != "rviz") {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] Invalid multi_source='%s' (must be 'yaml' or 'rviz')", multi_source_.c_str());
      throw std::runtime_error("Invalid multi_source");
    }

    // ################################
    // C++: normalize legacy multi_frame values to odin_ namespace
    // ################################
    if (multi_frame_ == "odom") { multi_frame_ = "odin_odom"; }
    else if (multi_frame_ == "map") { multi_frame_ = "odin_map"; }
    if (multi_frame_ != "odin_odom" && multi_frame_ != "odin_map") {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] Invalid multi_frame='%s' (must be 'odin_odom' or 'odin_map')", multi_frame_.c_str());
      throw std::runtime_error("Invalid multi_frame");
    }
    RCLCPP_INFO(this->get_logger(),
      "[FRAME] planning_frame=%s global_frame=%s",
      planning_frame_.c_str(), global_frame_.c_str());

    // multi + repeat is an invalid combination
    if (multi_enabled_ && repeat_enabled_) {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] multi_enabled && repeat_enabled is invalid; refusing to start");
      throw std::runtime_error("multi_enabled && repeat_enabled");
    }

    // 防呆：loop_count 只接受 -1（无限）或正整数（趟数）
    if (loop_count_ == 0 || loop_count_ < -1) {
      RCLCPP_WARN(this->get_logger(),
        "[REPEAT] Invalid loop_count=%d, using 1", loop_count_);
      loop_count_ = 1;
    }

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 10,
      std::bind(&CruiseController::odomCallback, this, std::placeholders::_1));

    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point_cruise", 10,
      std::bind(&CruiseController::waypointCallback, this, std::placeholders::_1));

    // ################################
    // C++: subscribe /stop in MULTI mode too
    // ################################
    if (this->get_parameter("repeat_enabled").as_bool() ||
        this->get_parameter("multi_enabled").as_bool()) {
      stop_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/stop", 10,
        std::bind(&CruiseController::stopCallback, this, std::placeholders::_1));
    }

    // ################################
    // C++: create MULTI marker pub sub and service
    // ################################
    if (multi_enabled_) {
      markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/multi_waypoints", rclcpp::QoS(10).reliable().transient_local());
      add_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/multi_waypoint_add", 10,
        std::bind(&CruiseController::addWaypointCallback, this, std::placeholders::_1));
      // ################################
      // C++: publish cruise autonomy lock for MULTI mode
      // ################################
      // localPlanner/pathFollower subscribe to release /joy control while
      // MULTI runs; only MULTI ever publishes (SINGLE/REPEAT unaffected).
      cruise_autonomy_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/cruise_autonomy", 10);
    }
    // ################################
    // C++: always create /multi_start for unified SINGLE/REPEAT/MULTI start
    // ################################
    // SINGLE/REPEAT/MULTI all park at READY_TO_START and need the same
    // explicit "Start Multi" trigger; the service is therefore unconditional.
    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/multi_start",
      std::bind(&CruiseController::startService, this, std::placeholders::_1, std::placeholders::_2));

    // ################################
    // C++: MULTI setup enter WAIT_LOCALIZATION
    // ################################
    // MULTI mode setup — MUST be after markers_pub_ creation (Step 7).
    // BOTH sources enter WAIT_LOCALIZATION first; the gate then dispatches
    // to READY_TO_START (yaml) or COLLECTING_WAYPOINTS (rviz). Neither begins
    // moving — an explicit /multi_start is required for every cruise mode.
    // The gate requires /state_estimation always, and the relocalization TF
    // only when multi_frame_=="map".
    if (multi_enabled_) {
      if (multi_source_ == "yaml") {
        loadYaml();
        // ################################
        // C++: keep invalid YAML routes idle instead of WAIT_LOCALIZATION
        // ################################
        if (waypoints_.size() < 2) {
          RCLCPP_ERROR(this->get_logger(),
            "[MULTI] multi_route.yaml has <2 waypoints (%zu); staying IDLE",
            waypoints_.size());
          state_ = CruiseState::IDLE;
          publishMarkers();
        } else {
          state_ = CruiseState::WAIT_LOCALIZATION;
          publishMarkers();
        }
      } else {
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] RViz mode: waiting for pose, then collect waypoints");
        state_ = CruiseState::WAIT_LOCALIZATION;
        publishMarkers();
      }
    }

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
      "Cruise controller ready. Prepare waypoint/route, then press Start Multi.");
  }

private:
  const char * stateName(CruiseState s)
  {
    switch (s) {
      case CruiseState::IDLE: return "IDLE";
      case CruiseState::GO_TO_DEST: return "GO_TO_DEST";
      case CruiseState::TURN_AT_DEST: return "TURN_AT_DEST";
      case CruiseState::RETURN_TO_START: return "RETURN_TO_START";
      case CruiseState::TURN_AT_START: return "TURN_AT_START";
      // ################################
      // C++: name MULTI states
      // ################################
      case CruiseState::WAIT_LOCALIZATION: return "WAIT_LOCALIZATION";
      case CruiseState::COLLECTING_WAYPOINTS: return "COLLECTING_WAYPOINTS";
      case CruiseState::GO_TO_WAYPOINT: return "GO_TO_WAYPOINT";
      case CruiseState::TURN_AT_WAYPOINT: return "TURN_AT_WAYPOINT";
      case CruiseState::WAIT_AT_WAYPOINT: return "WAIT_AT_WAYPOINT";
      case CruiseState::READY_TO_START: return "READY_TO_START";
      default: return "UNKNOWN";
    }
  }

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
    has_odom_ = true;
  }

  void waypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!has_odom_) {
      RCLCPP_WARN(this->get_logger(),
        "No /state_estimation received, ignoring cruise waypoint");
      return;
    }

    // ################################
    // C++: route /way_point_cruise clicks into the MULTI list
    // ################################
    if (multi_enabled_) {
      addMultiWaypoint(*msg);
      return;
    }

    // ################################
    // C++: resolve goal frame to planning frame before use
    // ################################
    geometry_msgs::msg::PointStamped goal;
    if (!transformGoalToPlanningFrame(*msg, goal)) {
      return;
    }

    // ################################
    // C++: running-state retarget is only for REPEAT while actually cruising
    // ################################
    // Matches only the running states — READY_TO_START is NOT "already
    // cruising", so a re-click while parked must not fire sendWaypointAndGo().
    const bool running_state =
      state_ == CruiseState::GO_TO_DEST ||
      state_ == CruiseState::TURN_AT_DEST ||
      state_ == CruiseState::RETURN_TO_START ||
      state_ == CruiseState::TURN_AT_START;
    if (repeat_enabled_ && running_state) {
      dest_x_ = goal.point.x;
      dest_y_ = goal.point.y;
      start_x_ = current_x_;
      start_y_ = current_y_;
      completed_loops_ = 0;
      pending_stop_ = false;
      turning_internal_ = false;
      RCLCPP_INFO(this->get_logger(),
        "[REPEAT] New waypoint, loops reset: dest=(%.3f, %.3f), start=(%.3f, %.3f)",
        dest_x_, dest_y_, start_x_, start_y_);
      sendWaypointAndGo(dest_x_, dest_y_, CruiseState::GO_TO_DEST);
      return;
    }

    if (state_ != CruiseState::IDLE && state_ != CruiseState::READY_TO_START) {
      RCLCPP_WARN(this->get_logger(),
        "Already cruising, ignoring new waypoint");
      return;
    }

    // ################################
    // C++: prepare destination but hold at READY_TO_START until /multi_start
    // ################################
    // Both the first click (IDLE) and a re-click while parked (READY_TO_START)
    // update the pending destination and STAY parked — no /way_point, no motion.
    start_x_ = current_x_;
    start_y_ = current_y_;
    dest_x_ = goal.point.x;
    dest_y_ = goal.point.y;
    if (repeat_enabled_) {
      completed_loops_ = 0;
      pending_stop_ = false;
      turning_internal_ = false;
    }
    state_ = CruiseState::READY_TO_START;
    RCLCPP_INFO(this->get_logger(),
      "%s Destination prepared: start=(%.3f, %.3f), dest=(%.3f, %.3f); press Start Multi to begin",
      repeat_enabled_ ? "[REPEAT]" : "[CRUISE]",
      start_x_, start_y_, dest_x_, dest_y_);
  }

  void stopCallback(const std_msgs::msg::Int8::ConstSharedPtr msg)
  {
    if (msg->data < 2) return;

    // Consume the node's own /stop=2 published at startTurn().
    // Without this, the self-publish would be treated as an external
    // stop and every turn would immediately abort the cruise.
    if (ignore_next_internal_stop_) {
      ignore_next_internal_stop_ = false;
      return;
    }

    if (turning_internal_) {
      pending_stop_ = true;
      RCLCPP_WARN(this->get_logger(),
        "[REPEAT] External stop queued during turn");
      return;
    }
    // ################################
    // C++: reset MULTI state on external stop
    // ################################
    publishZeroCmd();
    completed_loops_ = 0;
    closing_loop_ = false;
    pending_stop_ = false;
    active_goal_valid_ = false;
    // Release the autonomy lock on MULTI abort (guard: REPEAT/SINGLE never lock)
    if (multi_enabled_) {
      std_msgs::msg::Bool autonomy_msg;
      autonomy_msg.data = false;
      cruise_autonomy_pub_->publish(autonomy_msg);
    }
    RCLCPP_WARN(this->get_logger(),
      "[CRUISE] Stop received, cruise aborted");
    state_ = CruiseState::IDLE;
  }

  void publishZeroCmd()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }

  void sendWaypointAndGo(double x, double y, CruiseState next_state)
  {
    geometry_msgs::msg::PointStamped wp;
    wp.header.stamp = this->now();
    wp.header.frame_id = planning_frame_;
    wp.point.x = x;
    wp.point.y = y;
    wp.point.z = 0.0;
    waypoint_pub_->publish(wp);

    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 0;
    stop_pub_->publish(stop_msg);

    state_ = next_state;
    RCLCPP_INFO(this->get_logger(),
      "[CRUISE][WAYPOINT] phase=%s, publish /way_point: x=%.3f, y=%.3f",
      stateName(next_state), x, y);
  }

  void startTurn(CruiseState next_state)
  {
    if (repeat_enabled_) {
      turning_internal_ = true;
      ignore_next_internal_stop_ = true;  // consume self-published /stop=2 below
    }
    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 2;
    stop_pub_->publish(stop_msg);

    target_yaw_ = normalizeAngle(current_yaw_ + turn_angle_ * M_PI / 180.0);
    state_ = next_state;
    RCLCPP_INFO(this->get_logger(),
      "[CRUISE] Starting %.0f-degree turn: %s, target_yaw=%.3f (current=%.3f)",
      turn_angle_, stateName(next_state), target_yaw_, current_yaw_);
  }

  bool consumePendingStop()
  {
    if (!pending_stop_) {
      return false;
    }
    publishZeroCmd();
    pending_stop_ = false;
    completed_loops_ = 0;
    RCLCPP_WARN(this->get_logger(),
      "[REPEAT] Queued stop executed after turn");
    state_ = CruiseState::IDLE;
    return true;
  }

  // ################################
  // C++: seize /cmd_vel from pathFollower for MULTI turns and waits
  // ################################
  // Publish /stop=2 to seize /cmd_vel from pathFollower, but FIRST set
  // ignore_next_internal_stop_ so the node's own /stop=2 is not mistaken
  // for an external stop. turning_internal_ is deliberately NOT set, so an
  // external /stop=2 still aborts immediately in any MULTI state.
  void seizeControlForMulti()
  {
    ignore_next_internal_stop_ = true;
    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 2;
    stop_pub_->publish(stop_msg);
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

    // ################################
    // C++: compensate robot minimum effective yaw rate
    // ################################
    // 底层角速度死区：|angular.z| <= min_yaw_rate 时实际不转。
    // P 控制收敛到死区以内会停转，导致掉头永远到不了 yaw_tolerance。
    // 将非零角速度提升到死区边界，保留符号。
    if (fabs(wz) < min_yaw_rate_ && fabs(wz) > 0.0) {
      wz = (wz > 0.0) ? min_yaw_rate_ : -min_yaw_rate_;
    }

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

    // ################################
    // C++: MULTI localization gate dispatch
    // ################################
    // MULTI: pose gate — odom mode needs only /state_estimation (no map,
    // no relocalization); map mode additionally waits for the relocalization TF.
    if (multi_enabled_ && state_ == CruiseState::WAIT_LOCALIZATION) {
      if (!has_odom_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[MULTI] Waiting for /state_estimation...");
        return;
      }
      if (multi_frame_ == "odin_map" &&
          !tf_buffer_.canTransform(planning_frame_, global_frame_, tf2::TimePointZero)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[MULTI] Waiting for odin_map -> odin_odom relocalization TF...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
        "[MULTI] Pose ready (frame=%s), proceeding", multi_frame_.c_str());
      if (multi_source_ == "yaml") {
        if (waypoints_.size() < 2) {
          // ################################
          // C++: defensive guard — invalid YAML routes go idle (normally caught at startup)
          // ################################
          RCLCPP_ERROR(this->get_logger(),
            "[MULTI] YAML route invalid: <2 waypoints");
          state_ = CruiseState::IDLE;
          return;
        }
        // ################################
        // C++: hold YAML route at READY_TO_START until explicit /multi_start
        // ################################
        state_ = CruiseState::READY_TO_START;
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] YAML route ready (%zu waypoints); press Start Multi or call /multi_start to begin",
          waypoints_.size());
        publishMarkers();
      } else {
        state_ = CruiseState::COLLECTING_WAYPOINTS;
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] RViz mode: click waypoints (any frame, auto-converted to %s), then press Start Multi",
          multi_frame_.c_str());
        publishMarkers();
      }
      return;
    }

    switch (state_) {
    case CruiseState::READY_TO_START:
      // ################################
      // C++: parked until explicit /multi_start grants motion
      // ################################
      // Explicit case (do not rely on switch fall-through): robot holds still.
      return;

    case CruiseState::IDLE:
      return;

    case CruiseState::GO_TO_DEST:
      dx = current_x_ - dest_x_;
      dy = current_y_ - dest_y_;
      dist_sq = dx * dx + dy * dy;
      if (dist_sq < goal_clear_range_sq) {
        if (repeat_enabled_) {
          RCLCPP_INFO(this->get_logger(),
            "[REPEAT] Destination reached, loop %d", completed_loops_ + 1);
        }
        RCLCPP_INFO(this->get_logger(), "[CRUISE] Destination reached, turning...");
        startTurn(CruiseState::TURN_AT_DEST);
      }
      return;

    case CruiseState::TURN_AT_DEST:
      if (turnDone()) {
        turning_internal_ = false;
        ignore_next_internal_stop_ = false;
        publishZeroCmd();

        // A stop requested during this turn stops HERE, not after the
        // return leg — satisfies "stop immediately once turn completes".
        if (repeat_enabled_ && consumePendingStop()) {
          return;
        }

        RCLCPP_INFO(this->get_logger(), "[CRUISE] Turn done, returning to start...");
        sendWaypointAndGo(start_x_, start_y_, CruiseState::RETURN_TO_START);
      } else {
        publishTurnCmd();
      }
      return;

    case CruiseState::RETURN_TO_START:
      dx = current_x_ - start_x_;
      dy = current_y_ - start_y_;
      dist_sq = dx * dx + dy * dy;
      if (dist_sq < goal_clear_range_sq) {
        if (repeat_enabled_) {
          RCLCPP_INFO(this->get_logger(),
            "[REPEAT] Returning to start, loop %d", completed_loops_ + 1);
        }
        RCLCPP_INFO(this->get_logger(), "[CRUISE] Start reached, turning...");
        startTurn(CruiseState::TURN_AT_START);
      }
      return;

    case CruiseState::TURN_AT_START:
      if (turnDone()) {
        turning_internal_ = false;
        ignore_next_internal_stop_ = false;
        publishZeroCmd();

        if (!repeat_enabled_) {
          RCLCPP_INFO(this->get_logger(), "[CRUISE] Cruise complete!");
          state_ = CruiseState::IDLE;
          return;
        }

        // A stop queued during this turn stops HERE immediately.
        if (consumePendingStop()) {
          return;
        }

        completed_loops_++;
        RCLCPP_INFO(this->get_logger(),
          "[REPEAT][LOOP] loop %d/%s complete",
          completed_loops_,
          (loop_count_ > 0 ? std::to_string(loop_count_).c_str() : "inf"));

        if (loop_count_ > 0 && completed_loops_ >= loop_count_) {
          int loops_done = completed_loops_;
          completed_loops_ = 0;
          RCLCPP_INFO(this->get_logger(),
            "[REPEAT] Cruise complete after %d loops", loops_done);
          state_ = CruiseState::IDLE;
        } else {
          RCLCPP_INFO(this->get_logger(),
            "[REPEAT][LOOP] loop %d start: (%.3f, %.3f) -> (%.3f, %.3f)",
            completed_loops_ + 1,
            start_x_, start_y_, dest_x_, dest_y_);
          // 必须重新调用 sendWaypointAndGo()：掉头开始时发布了 /stop=2，
          // pathFollower 的 safetyStop 保持 2 会完全停止发布 /cmd_vel；
          // 此函数会重发 /way_point 并发布 /stop=0 恢复 pathFollower 控制。
          sendWaypointAndGo(dest_x_, dest_y_, CruiseState::GO_TO_DEST);
        }
      } else {
        publishTurnCmd();
      }
      return;

    // ################################
    // C++: add MULTI running state cases
    // ################################
    case CruiseState::GO_TO_WAYPOINT: {
      if (!active_goal_valid_) {
        startNextWaypoint();
        return;
      }
      double dx = current_x_ - gx_odom_;
      double dy = current_y_ - gy_odom_;
      double goal_clear_range = this->get_parameter("goal_clear_range").as_double();
      if (dx * dx + dy * dy < goal_clear_range * goal_clear_range) {
        active_goal_valid_ = false;
        // seize /cmd_vel BEFORE any turn/wait; publishZeroCmd to hold still
        seizeControlForMulti();
        publishZeroCmd();
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] Reached WP%zu", waypoint_index_);

        const Waypoint & w = waypoints_[waypoint_index_];
        if (fabs(w.turn_angle) > 1e-3) {
          RCLCPP_INFO(this->get_logger(),
            "[MULTI] Turning %.0f deg at WP%zu", w.turn_angle, waypoint_index_);
          target_yaw_ = normalizeAngle(current_yaw_ + w.turn_angle * M_PI / 180.0);
          state_ = CruiseState::TURN_AT_WAYPOINT;
        } else {
          wait_start_time_ = this->now().seconds();
          state_ = CruiseState::WAIT_AT_WAYPOINT;
        }
      }
      return;
    }

    case CruiseState::TURN_AT_WAYPOINT: {
      if (turnDone()) {
        publishZeroCmd();
        wait_start_time_ = this->now().seconds();
        state_ = CruiseState::WAIT_AT_WAYPOINT;
        RCLCPP_INFO(this->get_logger(), "[MULTI] Turn done at WP%zu", waypoint_index_);
      } else {
        publishTurnCmd();
      }
      return;
    }

    case CruiseState::WAIT_AT_WAYPOINT: {
      const Waypoint & w = waypoints_[waypoint_index_];
      double elapsed = this->now().seconds() - wait_start_time_;
      if (elapsed >= w.wait_time) {
        RCLCPP_INFO(this->get_logger(),
          "[MULTI] Wait done at WP%zu (%.1fs), advancing", waypoint_index_, w.wait_time);

        // ################################
        // C++: count a closed loop only after arriving at WP0 and finishing its wait
        // ################################
        // One round = physically drive WPN→WP0, arrive at WP0, and finish WP0's
        // turn/wait. Only then completed_loops_++. loop_count=1 must end parked at WP0.
        if (waypoint_index_ == 0 && closing_loop_) {
          completed_loops_++;
          bool done = (loop_count_ > 0 && completed_loops_ >= loop_count_);
          RCLCPP_INFO(this->get_logger(),
            "[MULTI] Loop %d/%s complete at WP0", completed_loops_,
            (loop_count_ > 0 ? std::to_string(loop_count_).c_str() : "inf"));
          if (done) {
            publishZeroCmd();
            RCLCPP_INFO(this->get_logger(),
              "[MULTI] Cruise complete after %d loops", completed_loops_);
            // Release the autonomy lock when the finite loop count is reached
            std_msgs::msg::Bool autonomy_msg;
            autonomy_msg.data = false;
            cruise_autonomy_pub_->publish(autonomy_msg);
            completed_loops_ = 0;
            closing_loop_ = false;
            state_ = CruiseState::IDLE;
            return;
          }
          closing_loop_ = false;
        }

        advanceWaypoint();
      } else {
        publishZeroCmd();  // keep robot still; pathFollower already stopped via /stop=2
      }
      return;
    }
    }
  }

  // ################################
  // C++: transform any PointStamped into the target frame
  // ################################
  // Generic coordinate conversion for RViz clicks: RViz Fixed Frame may
  // differ from multi_frame_; the click is converted here so that
  // waypoints_ always stores coordinates in multi_frame_.
  bool transformPointToFrame(
    const geometry_msgs::msg::PointStamped & input,
    const std::string & target_frame,
    geometry_msgs::msg::PointStamped & output)
  {
    // Case A: frames already match — no TF lookup needed
    if (input.header.frame_id == target_frame) {
      output = input;
      return true;
    }
    // Case B: convert via the live TF tree (e.g. map -> odom in odom mode)
    try {
      geometry_msgs::msg::TransformStamped t =
        tf_buffer_.lookupTransform(target_frame, input.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(input, output, t);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(),
        "[MULTI] Cannot transform clicked waypoint %s -> %s: %s",
        input.header.frame_id.c_str(), target_frame.c_str(), e.what());
      return false;
    }
  }

  // ################################
  // C++: resolve a received goal frame into the planning frame
  // ################################
  // Policy: empty -> assume planning frame (warn); planning frame -> copy;
  // global frame -> tf2 transform (reject if TF missing); motion-control
  // bare "map" -> reject; any other frame -> transform if possible, else reject.
  bool transformGoalToPlanningFrame(
    const geometry_msgs::msg::PointStamped & input,
    geometry_msgs::msg::PointStamped & output)
  {
    if (input.header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "[CRUISE] goal has empty frame; assuming %s (legacy compat, no transform)",
        planning_frame_.c_str());
      // Legacy compatibility: the coordinates are used as-is, but the output
      // MUST be explicitly tagged with the planning frame. This is an
      // assumption, NOT a coordinate transform.
      output = input;
      output.header.frame_id = planning_frame_;
      return true;
    }
    if (input.header.frame_id == planning_frame_) {
      RCLCPP_INFO(this->get_logger(),
        "[CRUISE] goal received frame=%s x=%.3f y=%.3f", planning_frame_.c_str(),
        input.point.x, input.point.y);
      RCLCPP_INFO(this->get_logger(),
        "[CRUISE] goal already in planning frame, no TF required");
      output = input;
      return true;
    }
    if (input.header.frame_id == "map") {
      RCLCPP_WARN(this->get_logger(),
        "[CRUISE] goal frame 'map' belongs to motion-control TF tree; use %s or %s",
        planning_frame_.c_str(), global_frame_.c_str());
      return false;
    }
    if (input.header.frame_id == global_frame_) {
      RCLCPP_INFO(this->get_logger(),
        "[CRUISE] goal received frame=%s x=%.3f y=%.3f", global_frame_.c_str(),
        input.point.x, input.point.y);
    }
    try {
      geometry_msgs::msg::TransformStamped t =
        tf_buffer_.lookupTransform(planning_frame_, input.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(input, output, t);
      RCLCPP_INFO(this->get_logger(),
        "[CRUISE] transformed %s -> %s: x=%.3f y=%.3f",
        input.header.frame_id.c_str(), planning_frame_.c_str(), output.point.x, output.point.y);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(),
        "[CRUISE] goal rejected: %s -> %s TF unavailable (%s)",
        input.header.frame_id.c_str(), planning_frame_.c_str(), e.what());
      return false;
    }
  }

  // ################################
  // C++: transform global-frame waypoint to planning frame
  // ################################
  bool transformToOdom(double mx, double my, double & ox, double & oy)
  {
    try {
      // Explicit lookup + doTransform (NOT buffer_.transform), and do NOT set
      // header.stamp to tf2::TimePointZero — use the lookup's own timepoint.
      geometry_msgs::msg::TransformStamped t_global_planning =
        tf_buffer_.lookupTransform(planning_frame_, global_frame_, tf2::TimePointZero);

      geometry_msgs::msg::PointStamped map_pt;
      map_pt.header.frame_id = global_frame_;
      map_pt.point.x = mx;
      map_pt.point.y = my;
      map_pt.point.z = 0.0;

      geometry_msgs::msg::PointStamped odom_pt;
      tf2::doTransform(map_pt, odom_pt, t_global_planning);
      ox = odom_pt.point.x;
      oy = odom_pt.point.y;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[MULTI] TF transform failed: %s", e.what());
      return false;
    }
  }

  // ################################
  // C++: load multi waypoints from yaml
  // ################################
  void loadYaml()
  {
    try {
      YAML::Node root = YAML::LoadFile(multi_route_file_);
      YAML::Node wps = root["multi_cruise"]["waypoints"];
      for (const auto & wp : wps) {
        Waypoint w;
        w.x = wp["x"].as<double>();
        w.y = wp["y"].as<double>();
        w.turn_angle = wp["turn_angle"] ? wp["turn_angle"].as<double>() : 0.0;
        w.wait_time = wp["wait_time"] ? wp["wait_time"].as<double>() : default_wait_time_;
        waypoints_.push_back(w);
      }
      RCLCPP_INFO(this->get_logger(),
        "[MULTI] Loaded %zu waypoints from %s (frame=%s)",
        waypoints_.size(), multi_route_file_.c_str(), multi_frame_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
        "[MULTI] Failed to load %s: %s", multi_route_file_.c_str(), e.what());
    }
  }

  // ################################
  // C++: publish MULTI route and waypoint markers
  // ################################
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    auto header = [this]() {
      std_msgs::msg::Header h;
      h.frame_id = multi_frame_;
      h.stamp = this->now();
      return h;
    };

    // LINE_STRIP connecting all waypoints (closed loop: last→first)
    visualization_msgs::msg::Marker line;
    line.header = header();
    line.ns = "route";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.05;
    line.color.r = 1.0f; line.color.g = 1.0f; line.color.b = 0.0f; line.color.a = 1.0f;
    line.pose.orientation.w = 1.0;
    for (const auto & w : waypoints_) {
      geometry_msgs::msg::Point p;
      p.x = w.x; p.y = w.y; p.z = 0.0;
      line.points.push_back(p);
    }
    if (waypoints_.size() >= 2) {
      geometry_msgs::msg::Point p0;
      p0.x = waypoints_[0].x; p0.y = waypoints_[0].y; p0.z = 0.0;
      line.points.push_back(p0);  // close the loop
    }
    arr.markers.push_back(line);

    for (size_t i = 0; i < waypoints_.size(); i++) {
      // SPHERE
      visualization_msgs::msg::Marker sphere;
      sphere.header = header();
      sphere.ns = "wp_sphere";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = waypoints_[i].x;
      sphere.pose.position.y = waypoints_[i].y;
      sphere.pose.position.z = 0.0;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.4; sphere.scale.y = 0.4; sphere.scale.z = 0.4;
      // highlight current
      bool is_current = (multi_enabled_ && state_ == CruiseState::GO_TO_WAYPOINT &&
                         i == waypoint_index_);
      if (is_current) {
        sphere.color.r = 0.0f; sphere.color.g = 1.0f; sphere.color.b = 0.0f;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.7;
      } else {
        sphere.color.r = 1.0f; sphere.color.g = 1.0f; sphere.color.b = 1.0f;
      }
      sphere.color.a = 1.0f;
      arr.markers.push_back(sphere);

      // TEXT_VIEW_FACING
      visualization_msgs::msg::Marker text;
      text.header = header();
      text.ns = "wp_text";
      text.id = static_cast<int>(i);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = waypoints_[i].x;
      text.pose.position.y = waypoints_[i].y;
      text.pose.position.z = 0.6;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.5;
      text.color.r = 0.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
      text.text = "WP" + std::to_string(i);
      arr.markers.push_back(text);
    }

    markers_pub_->publish(arr);
  }

  // ################################
  // C++: implement MULTI cruise start sequence
  // ################################
  void beginMultiCruise()
  {
    waypoint_index_ = 0;
    completed_loops_ = 0;
    closing_loop_ = false;
    RCLCPP_INFO(this->get_logger(),
      "[MULTI] Cruise started: %zu waypoints, loop_count=%d, frame=%s",
      waypoints_.size(), loop_count_, multi_frame_.c_str());
    startNextWaypoint();
  }

  void startNextWaypoint()
  {
    if (waypoint_index_ >= waypoints_.size()) {
      RCLCPP_ERROR(this->get_logger(), "[MULTI] waypoint_index_ out of range");
      state_ = CruiseState::IDLE;
      return;
    }
    // Enter GO_TO_WAYPOINT FIRST and clear active_goal_valid_ BEFORE any
    // coordinate work, so that if map-mode transform fails the controlLoop
    // retries next tick (GO_TO_WAYPOINT sees active_goal_valid_==false and
    // calls startNextWaypoint()).
    state_ = CruiseState::GO_TO_WAYPOINT;
    active_goal_valid_ = false;

    const Waypoint & w = waypoints_[waypoint_index_];
    if (multi_frame_ == "odin_odom") {
      // odom mode (default): waypoint coordinates ARE odom coordinates.
      // No TF, no map, no relocalization required.
      gx_odom_ = w.x;
      gy_odom_ = w.y;
    } else {  // multi_frame_ == "odin_map"
      if (!transformToOdom(w.x, w.y, gx_odom_, gy_odom_)) {
        // transformToOdom() already logs the throttled failure detail
        return;  // stay in GO_TO_WAYPOINT, retry on next tick
      }
    }
    active_goal_valid_ = true;
    RCLCPP_INFO(this->get_logger(),
      "[MULTI] Going to WP%zu: (%s frame) (%.3f, %.3f) -> %s (%.3f, %.3f)",
      waypoint_index_, multi_frame_.c_str(), w.x, w.y, planning_frame_.c_str(), gx_odom_, gy_odom_);
    sendMultiWaypointAndGo(gx_odom_, gy_odom_);
    publishMarkers();
  }

  // ################################
  // C++: advance waypoint index with closing_loop_ wrap
  // ################################
  // Advance the index. When wrapping past the last waypoint, set
  // closing_loop_=true so the arriving-at-WP0 logic (Task 5's WAIT_AT_WAYPOINT
  // completion) knows this WP0 arrival closes a round. This function does NOT
  // count loops — counting happens only after the robot physically arrives at
  // WP0 AND finishes WP0's turn/wait (see Task 5 Step 3).
  void advanceWaypoint()
  {
    waypoint_index_++;
    if (waypoint_index_ >= waypoints_.size()) {
      waypoint_index_ = 0;
      closing_loop_ = true;
    }
    startNextWaypoint();
  }

  // MULTI publishes /way_point in planning_frame_ (odin_odom) — goals are
  // always forwarded to localPlanner in the planning frame. SINGLE/REPEAT
  // sendWaypointAndGo publishes in planning_frame_ too.
  void sendMultiWaypointAndGo(double x, double y)
  {
    geometry_msgs::msg::PointStamped wp;
    wp.header.stamp = this->now();
    wp.header.frame_id = planning_frame_;
    wp.point.x = x;
    wp.point.y = y;
    wp.point.z = 0.0;
    waypoint_pub_->publish(wp);

    auto stop_msg = std_msgs::msg::Int8();
    stop_msg.data = 0;
    stop_pub_->publish(stop_msg);

    RCLCPP_INFO(this->get_logger(),
      "[MULTI][WAYPOINT] publish /way_point (%s): x=%.3f, y=%.3f",
      planning_frame_.c_str(), x, y);
  }

  // ################################
  // C++: unify MULTI waypoint addition for all input channels
  // ################################
  bool addMultiWaypoint(const geometry_msgs::msg::PointStamped & msg)
  {
    if (multi_source_ == "yaml") {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[MULTI] Ignoring RViz waypoint because multi_source=yaml");
      return false;
    }
    if (state_ != CruiseState::COLLECTING_WAYPOINTS) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[MULTI] Route already started; RViz waypoint ignored");
      return false;
    }
    if (msg.header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "[MULTI] Waypoint has empty frame_id; ignored");
      return false;
    }
    // ################################
    // C++: reject waypoints in the motion-control map frame
    // ################################
    if (msg.header.frame_id == "map") {
      RCLCPP_WARN(this->get_logger(),
        "[MULTI] Waypoint frame 'map' belongs to motion-control TF tree; use %s or %s",
        planning_frame_.c_str(), global_frame_.c_str());
      return false;
    }
    geometry_msgs::msg::PointStamped converted;
    if (!transformPointToFrame(msg, multi_frame_, converted)) {
      RCLCPP_WARN(this->get_logger(),
        "[MULTI] Cannot add waypoint: TF %s -> %s unavailable",
        msg.header.frame_id.c_str(), multi_frame_.c_str());
      return false;
    }
    Waypoint w;
    w.x = converted.point.x;
    w.y = converted.point.y;
    w.turn_angle = 0.0;
    w.wait_time = default_wait_time_;
    waypoints_.push_back(w);
    publishMarkers();
    if (msg.header.frame_id == multi_frame_) {
      RCLCPP_INFO(this->get_logger(),
        "[MULTI] Added WP%zu at (%.3f, %.3f) in %s frame; %zu total",
        waypoints_.size() - 1, w.x, w.y, multi_frame_.c_str(), waypoints_.size());
    } else {
      RCLCPP_INFO(this->get_logger(),
        "[MULTI] Added WP%zu: input %s=(%.3f, %.3f) -> stored %s=(%.3f, %.3f); %zu total",
        waypoints_.size() - 1, msg.header.frame_id.c_str(), msg.point.x, msg.point.y,
        multi_frame_.c_str(), w.x, w.y, waypoints_.size());
    }
    return true;
  }

  // ################################
  // C++: handle RViz waypoint add and start service
  // ################################
  void addWaypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    addMultiWaypoint(*msg);
  }

  void startService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    // ################################
    // C++: unified explicit start for SINGLE / REPEAT / MULTI
    // ################################
    // The mode decides what starts; only the "explicit authorization" concept
    // is shared. MULTI keeps its existing route/odom/TF checks; SINGLE/REPEAT
    // rely on the has_odom_ gate in waypointCallback that prepared the goal.
    if (multi_enabled_) {
      // MULTI: rviz COLLECTING_WAYPOINTS or yaml READY_TO_START, both >=2 waypoints
      const bool startable =
        (multi_source_ == "rviz" && state_ == CruiseState::COLLECTING_WAYPOINTS) ||
        (multi_source_ == "yaml" && state_ == CruiseState::READY_TO_START);
      if (!startable) {
        const bool running =
          state_ == CruiseState::GO_TO_WAYPOINT ||
          state_ == CruiseState::TURN_AT_WAYPOINT ||
          state_ == CruiseState::WAIT_AT_WAYPOINT;
        res->success = false;
        res->message = running
          ? "Multi cruise already running"
          : "/multi_start not valid in current state";
        RCLCPP_WARN(this->get_logger(), "[MULTI] %s", res->message.c_str());
        return;
      }
      if (waypoints_.size() < 2) {
        res->success = false;
        res->message = "At least 2 waypoints are required";
        RCLCPP_WARN(this->get_logger(), "[MULTI] %s", res->message.c_str());
        return;
      }
      if (!has_odom_) {
        res->success = false;
        res->message = "/state_estimation not available";
        RCLCPP_WARN(this->get_logger(), "[MULTI] %s", res->message.c_str());
        return;
      }
      // ################################
      // C++: gate /multi_start on relocalization TF only in map mode
      // ################################
      // odom mode starts with /state_estimation only; map mode additionally
      // needs the relocalization TF (re-checked at start time).
      if (multi_frame_ == "odin_map" &&
          !tf_buffer_.canTransform(planning_frame_, global_frame_, tf2::TimePointZero)) {
        res->success = false;
        res->message = "odin_map -> odin_odom TF not available";
        RCLCPP_WARN(this->get_logger(), "[MULTI] %s", res->message.c_str());
        return;
      }
      res->success = true;
      res->message = "Starting multi cruise";
      RCLCPP_INFO(this->get_logger(), "[MULTI] /multi_start accepted; starting cruise");
      // ################################
      // C++: lock cruise autonomy before MULTI waypoints drive
      // ################################
      // With the lock set, localPlanner/pathFollower ignore /joy (real PS3
      // controller jitter must not clear autonomyMode during MULTI).
      std_msgs::msg::Bool autonomy_msg;
      autonomy_msg.data = true;
      cruise_autonomy_pub_->publish(autonomy_msg);
      beginMultiCruise();
      return;
    }

    if (repeat_enabled_) {
      if (state_ != CruiseState::READY_TO_START) {
        res->success = false;
        res->message = "Repeat cruise is not ready to start";
        RCLCPP_WARN(this->get_logger(), "[REPEAT] %s", res->message.c_str());
        return;
      }
      res->success = true;
      res->message = "Starting repeat cruise";
      RCLCPP_INFO(this->get_logger(), "[REPEAT] /multi_start accepted; starting cruise");
      completed_loops_ = 0;
      pending_stop_ = false;
      turning_internal_ = false;
      sendWaypointAndGo(dest_x_, dest_y_, CruiseState::GO_TO_DEST);
      return;
    }

    // SINGLE
    if (state_ != CruiseState::READY_TO_START) {
      res->success = false;
      res->message = "Single cruise is not ready to start";
      RCLCPP_WARN(this->get_logger(), "[CRUISE] %s", res->message.c_str());
      return;
    }
    res->success = true;
    res->message = "Starting single cruise";
    RCLCPP_INFO(this->get_logger(), "[CRUISE] /multi_start accepted; starting cruise");
    sendWaypointAndGo(dest_x_, dest_y_, CruiseState::GO_TO_DEST);
  }

  // ################################
  // C++: add tf2 buffer and listener members
  // ################################
  // Declared before state_ so init order matches the member-init list
  // (tf_buffer_ before tf_listener_; listener needs a live buffer).
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  CruiseState state_;
  bool has_odom_;
  double turn_angle_;
  double min_yaw_rate_;
  bool repeat_enabled_;
  int loop_count_;
  int completed_loops_ = 0;
  bool turning_internal_ = false;
  bool ignore_next_internal_stop_ = false;
  bool pending_stop_ = false;
  double start_x_, start_y_, dest_x_, dest_y_;
  double current_x_, current_y_, current_yaw_;
  double target_yaw_;

  // ################################
  // C++: add MULTI state members
  // ################################
  // MULTI members
  // waypoints_ coordinates always belong to multi_frame_ (never mixed frames)
  std::vector<Waypoint> waypoints_;
  size_t waypoint_index_ = 0;
  // completed_loops_ and loop_count_ ALREADY EXIST from REPEAT — reuse, do NOT redeclare
  bool closing_loop_ = false;
  bool multi_enabled_ = false;
  std::string multi_source_;
  std::string multi_frame_;
  std::string planning_frame_;
  std::string global_frame_;
  std::string multi_route_file_;
  double default_wait_time_ = 2.0;
  double gx_odom_ = 0.0, gy_odom_ = 0.0;
  bool active_goal_valid_ = false;
  double wait_start_time_ = 0.0;

  // ################################
  // C++: add MULTI marker pub sub and service members
  // ################################
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr add_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cruise_autonomy_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr stop_sub_;
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
