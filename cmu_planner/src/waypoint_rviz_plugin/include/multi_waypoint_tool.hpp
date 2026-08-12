#ifndef MULTI_WAYPOINT_TOOL_H
#define MULTI_WAYPOINT_TOOL_H

#include <QObject>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/tool.hpp>

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace waypoint_rviz_plugin
{
// ################################
// C++: add MULTI waypoint RViz tool
// ################################
class MultiWaypointTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT
public:
  MultiWaypointTool();

  ~MultiWaypointTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Clock::SharedPtr clock_;
  rviz_common::properties::StringProperty * topic_property_;
};
}  // namespace waypoint_rviz_plugin

#endif  // MULTI_WAYPOINT_TOOL_H
