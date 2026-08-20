#include <multi_waypoint_tool.hpp>

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace waypoint_rviz_plugin
{
// ################################
// C++: implement MULTI waypoint RViz tool
// ################################
// MultiWaypointTool reuses PoseTool's XY-plane projection: the mouse ray
// is intersected with the RViz Fixed Frame XY plane (z=0), so waypoints
// can be placed where no point cloud exists. It only publishes
// PointStamped clicks on /multi_waypoint_add — no /joy, no /way_point.
MultiWaypointTool::MultiWaypointTool()
: rviz_default_plugins::tools::PoseTool()
{
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/multi_waypoint_add",
    "The topic on which to publish MULTI cruise waypoints.",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

MultiWaypointTool::~MultiWaypointTool() = default;

void MultiWaypointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  setName("Multi Waypoint");
  updateTopic();
}

void MultiWaypointTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  pub_ = raw_node->template create_publisher<geometry_msgs::msg::PointStamped>(
    topic_property_->getStdString(), 10);
  clock_ = raw_node->get_clock();
}

// ################################
// C++: publish XY-plane click as MULTI waypoint
// ################################
void MultiWaypointTool::onPoseSet(double x, double y, double theta)
{
  (void)theta;

  geometry_msgs::msg::PointStamped waypoint;
  // frame_id follows the current RViz Fixed Frame (e.g. "odom"), never
  // hardcoded — cruiseController converts it to multi_frame_ if needed.
  waypoint.header.frame_id = context_->getFixedFrame().toStdString();
  waypoint.header.stamp = clock_->now();
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = 0.0;

  pub_->publish(waypoint);
}
}  // namespace waypoint_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::MultiWaypointTool, rviz_common::Tool)
