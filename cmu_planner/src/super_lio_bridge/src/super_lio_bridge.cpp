#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class SuperLioBridge : public rclcpp::Node
{
public:
  SuperLioBridge() : Node("super_lio_bridge")
  {
    // Declare parameters
    this->declare_parameter("odom_input_topic", "/lio/odom");
    this->declare_parameter("cloud_input_topic", "/lio/cloud_world");
    this->declare_parameter("state_output_topic", "/state_estimation");
    this->declare_parameter("cloud_output_topic", "/registered_scan");
    this->declare_parameter("global_frame", "world");
    this->declare_parameter("child_frame", "imu");
    this->declare_parameter("publish_tf", false);

    // Get parameters
    std::string odom_input_topic, cloud_input_topic, state_output_topic, cloud_output_topic;
    this->get_parameter("odom_input_topic", odom_input_topic);
    this->get_parameter("cloud_input_topic", cloud_input_topic);
    this->get_parameter("state_output_topic", state_output_topic);
    this->get_parameter("cloud_output_topic", cloud_output_topic);
    this->get_parameter("global_frame", global_frame_);
    this->get_parameter("child_frame", child_frame_);
    this->get_parameter("publish_tf", publish_tf_);

    // Publishers — forward odometry and point cloud unchanged
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(state_output_topic, 10);
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_output_topic, rclcpp::SensorDataQoS());

    // Subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_input_topic, rclcpp::SensorDataQoS(),
      std::bind(&SuperLioBridge::odomCallback, this, std::placeholders::_1));

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_input_topic, rclcpp::SensorDataQoS(),
      std::bind(&SuperLioBridge::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Super-LIO Bridge started: %s→%s, %s→%s (frame=%s→%s)",
      odom_input_topic.c_str(), state_output_topic.c_str(),
      cloud_input_topic.c_str(), cloud_output_topic.c_str(),
      global_frame_.c_str(), child_frame_.c_str());

    if (!publish_tf_) {
      RCLCPP_INFO(this->get_logger(), "TF publishing disabled — Super-LIO owns world→imu TF");
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    auto out = *msg;
    out.header.frame_id = global_frame_;
    out.child_frame_id = child_frame_;
    pub_odom_->publish(out);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    auto out = *msg;
    out.header.frame_id = global_frame_;
    pub_cloud_->publish(out);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;

  std::string global_frame_;
  std::string child_frame_;
  bool publish_tf_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SuperLioBridge>());
  rclcpp::shutdown();
  return 0;
}
