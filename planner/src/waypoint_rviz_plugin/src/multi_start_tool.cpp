#include <multi_start_tool.hpp>

#include <rviz_common/display_context.hpp>

namespace waypoint_rviz_plugin
{
// ################################
// C++: implement Start Multi trigger RViz tool
// ################################
// Thin trigger only: firing activation calls /multi_start; every legality
// check lives in CruiseController::startService(). The tool never reads
// state, never touches /joy, /way_point, or /multi_waypoint_add. After
// firing it Q_EMITs close() so RViz returns to the default tool.
MultiStartTool::MultiStartTool() = default;

MultiStartTool::~MultiStartTool() = default;

void MultiStartTool::onInitialize()
{
  setName("Start Multi");
  auto raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  node_ = raw_node;
  client_ = node_->create_client<std_srvs::srv::Trigger>("/multi_start");
}

void MultiStartTool::activate()
{
  // ################################
  // C++: fire /multi_start once on tool activation
  // ################################
  if (!client_ || !client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(),
      "[MULTI] /multi_start service not available (is cruiseController running?)");
  } else {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_->async_send_request(req,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          auto res = future.get();
          if (res->success) {
            RCLCPP_INFO(node_->get_logger(), "[MULTI] %s", res->message.c_str());
          } else {
            RCLCPP_WARN(node_->get_logger(), "[MULTI] %s", res->message.c_str());
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node_->get_logger(),
            "[MULTI] /multi_start service call failed: %s", e.what());
        }
      });
  }
  // ################################
  // C++: return to the default tool so camera/nav tools stay usable
  // ################################
  // rviz_common::Tool::close() tells the ToolManager to switch back.
  Q_EMIT close();
}

void MultiStartTool::deactivate() {}
}  // namespace waypoint_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::MultiStartTool, rviz_common::Tool)
