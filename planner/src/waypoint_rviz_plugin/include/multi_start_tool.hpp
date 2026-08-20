#ifndef MULTI_START_TOOL_H
#define MULTI_START_TOOL_H

#include <QObject>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/tool.hpp>

namespace waypoint_rviz_plugin
{
// ################################
// C++: add Start Multi trigger RViz tool
// ################################
class MultiStartTool : public rviz_common::Tool
{
  Q_OBJECT
public:
  MultiStartTool();

  ~MultiStartTool() override;

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};
}  // namespace waypoint_rviz_plugin

#endif  // MULTI_START_TOOL_H
