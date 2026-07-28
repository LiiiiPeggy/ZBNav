#include <memory>
#include <deque>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"


#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

class VehicleSimulatorAdapter : public rclcpp::Node
{
public:
  VehicleSimulatorAdapter()
  : Node("vehicleSimulator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "sensor");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/base_odom");

    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_state_estimation_ = declare_parameter<bool>("publish_state_estimation", true);
    publish_registered_scan_ = declare_parameter<bool>("publish_registered_scan", true);
    enable_cmd_vel_bridge_ = declare_parameter<bool>("enable_cmd_vel_bridge", true);
    tf_tolerance_ = this->declare_parameter<double>("tf_tolerance", 0.02);

    state_pub_ = create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&VehicleSimulatorAdapter::odomCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points",
      rclcpp::SensorDataQoS(),
      std::bind(&VehicleSimulatorAdapter::scanCallback, this, std::placeholders::_1));

    if (enable_cmd_vel_bridge_) {
      cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

      cmd_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_stamped",
        10,
        std::bind(&VehicleSimulatorAdapter::cmdStampedCallback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(), "vehicleSimulator adapter started");
  }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // msg 来自 /base_odom，对应 T_map_base_link
    tf2::Transform T_map_base;
    T_map_base.setOrigin(tf2::Vector3(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z
    ));

    T_map_base.setRotation(tf2::Quaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    ));

    // 1. TF 树发布 map -> base_link
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_map_base;
      rclcpp::Time tf_stamp(msg->header.stamp);
      // tf_stamp = tf_stamp + rclcpp::Duration::from_seconds(tf_tolerance_);

      tf_map_base.header.stamp = msg->header.stamp;

      // tf_map_base.header.stamp = msg->header.stamp + rclcpp::Duration::from_seconds(tf_tolerance_);
      tf_map_base.header.frame_id = "map";
      tf_map_base.child_frame_id = "base_link";
      tf_map_base.transform = tf2::toMsg(T_map_base);

      tf_broadcaster_->sendTransform(tf_map_base);
    }

    // 2. 查询 base_link -> velodyne
    geometry_msgs::msg::TransformStamped tf_base_velodyne_msg;

    try {
      tf_base_velodyne_msg = tf_buffer_.lookupTransform(
        "base_link",
        "velodyne_base_link",
        tf2::TimePointZero,
        tf2::durationFromSec(0.05)
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Cannot lookup base_link -> velodyne: %s",
        ex.what()
      );
      return;
    }

    tf2::Transform T_base_velodyne;
    tf2::fromMsg(tf_base_velodyne_msg.transform, T_base_velodyne);

    // 3. 计算 velodyne 在 map 下的里程计
    tf2::Transform T_map_velodyne = T_map_base * T_base_velodyne;

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "map";
    out.child_frame_id = "velodyne";

    out.pose.pose.position.x = T_map_velodyne.getOrigin().x();
    out.pose.pose.position.y = T_map_velodyne.getOrigin().y();
    out.pose.pose.position.z = T_map_velodyne.getOrigin().z();
    out.pose.pose.orientation = tf2::toMsg(T_map_velodyne.getRotation());

    // 先沿用 base_link 的 twist；如果下游强依赖 velodyne 点速度，再补 omega × r
    out.twist = msg->twist;

    if (publish_state_estimation_) {
      state_pub_->publish(out);
    }

    last_odom_ = out;
    has_odom_ = true;
    pushOdomHistory(out);
  }


  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scan)
  {
    if (!publish_registered_scan_) {
      return;
    }

    if (scan->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Input point cloud has empty frame_id");
      return;
    }

    try {
      auto tf = tf_buffer_.lookupTransform(
        map_frame_,
        scan->header.frame_id,
        scan->header.stamp,
        rclcpp::Duration::from_seconds(0.05));

      sensor_msgs::msg::PointCloud2 registered_scan;
      tf2::doTransform(*scan, registered_scan, tf);
      registered_scan.header.frame_id = map_frame_;

      scan_pub_->publish(registered_scan);
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "PointCloud transform failed: %s", ex.what());
    }
  }

  void cmdStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    geometry_msgs::msg::Twist cmd = msg->twist;
    cmd_pub_->publish(cmd);
  }

  void pushOdomHistory(const nav_msgs::msg::Odometry &odom)
  {
    odom_history_.push_back(odom);
    while (odom_history_.size() > 400) {
      odom_history_.pop_front();
    }
  }

private:
  std::string map_frame_;
  std::string base_frame_;
  std::string sensor_frame_;
  std::string odom_topic_;

  bool publish_tf_{true};
  bool publish_state_estimation_{true};
  bool publish_registered_scan_{true};
  bool enable_cmd_vel_bridge_{false};

  bool has_odom_{false};
  nav_msgs::msg::Odometry last_odom_;
  std::deque<nav_msgs::msg::Odometry> odom_history_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double tf_tolerance_ = 0.02;
  // message_filters::Subscriber<sensor_msgs::msg::PointCloud2> scan_sub_;
  // std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> scan_filter_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleSimulatorAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}