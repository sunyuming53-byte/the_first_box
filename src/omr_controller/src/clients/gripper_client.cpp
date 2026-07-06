#include "omr_controller/clients/gripper_client.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp_action/client.hpp>

namespace omr_controller {

GripperClient::GripperClient(rclcpp::Node* node, const std::string& action_name)
    : node_(node) {
  client_ = rclcpp_action::create_client<CommandAction>(node_, action_name);
}

bool GripperClient::open(double force_pct) { return sendGoal(1.0, force_pct); }

bool GripperClient::close(double force_pct) { return sendGoal(0.0, force_pct); }

void GripperClient::stop() {
  if (active_goal_) {
    client_->async_cancel_goal(active_goal_);
    active_goal_.reset();
  }
}

bool GripperClient::sendGoal(double position, double force_pct) {
  if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available");
    return false;
  }

  auto goal = CommandAction::Goal();
  goal.command.position = position;
  goal.command.max_effort = force_pct / 100.0;

  auto send_goal_options = rclcpp_action::Client<CommandAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(node_->get_logger(), "Gripper goal rejected");
          active_goal_.reset();
          return;
        }
        active_goal_ = goal_handle;
      };
  send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult& result) {
        (void)result;
        active_goal_.reset();
      };

  client_->async_send_goal(goal, send_goal_options);
  return true;
}

}  // namespace omr_controller
