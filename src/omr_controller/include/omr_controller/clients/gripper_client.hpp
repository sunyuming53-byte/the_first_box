#pragma once

#include <memory>
#include <string>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace omr_controller {

class GripperClient {
public:
    explicit GripperClient(rclcpp::Node::SharedPtr node,
                           const std::string& action_name = "/gripper/follow_joint_trajectory");

    GripperClient(const GripperClient&) = delete;
    GripperClient& operator=(const GripperClient&) = delete;
    GripperClient(GripperClient&&) = delete;
    GripperClient& operator=(GripperClient&&) = delete;

    bool open(double force_pct = 50);
    bool close(double force_pct = 50);
    void stop();

private:
    using CommandAction = control_msgs::action::GripperCommand;
    using ActionClient = rclcpp_action::Client<CommandAction>;
    using GoalHandle = rclcpp_action::ClientGoalHandle<CommandAction>;

    bool sendGoal(double position, double force_pct);

    rclcpp::Node::SharedPtr node_;
    ActionClient::SharedPtr client_;
    GoalHandle::SharedPtr active_goal_;
};

}  // namespace omr_controller
