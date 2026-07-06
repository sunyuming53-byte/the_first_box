#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace omr_controller::test {

// ---------------------------------------------------------------------------
// create_mock_jtc_server
// ---------------------------------------------------------------------------
/// Create a mock FollowJointTrajectory action server that auto-accepts and
/// immediately succeeds every goal with error_code == SUCCESSFUL.
inline auto create_mock_jtc_server(rclcpp::Node* node, const std::string& name)
    -> std::shared_ptr<rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>> {
    using Action = control_msgs::action::FollowJointTrajectory;

    auto handle_goal = [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>&) {
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted =
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>& goal_handle) {
            auto result = std::make_shared<Action::Result>();
            result->error_code = Action::Result::SUCCESSFUL;
            goal_handle->succeed(result);
        };

    return rclcpp_action::create_server<Action>(node, name, handle_goal, handle_cancel,
                                                handle_accepted);
}

// ---------------------------------------------------------------------------
// create_mock_gripper_server
// ---------------------------------------------------------------------------
/// Create a mock GripperCommand action server that auto-accepts and
/// immediately succeeds every goal.
inline auto create_mock_gripper_server(rclcpp::Node* node, const std::string& name)
    -> std::shared_ptr<rclcpp_action::Server<control_msgs::action::GripperCommand>> {
    using Action = control_msgs::action::GripperCommand;

    auto handle_goal = [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>&) {
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted =
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>& goal_handle) {
            auto result = std::make_shared<Action::Result>();
            goal_handle->succeed(result);
        };

    return rclcpp_action::create_server<Action>(node, name, handle_goal, handle_cancel,
                                                handle_accepted);
}

// ---------------------------------------------------------------------------
// create_mock_joint_state_publisher
// ---------------------------------------------------------------------------
/// Create a transient publisher on `topic`, publish a single JointState
/// message with the given positions/names, and return the publisher handle.
/// The caller must keep the returned publisher alive.
inline auto create_mock_joint_state_publisher(
    rclcpp::Node* node, const std::string& topic, const std::vector<double>& positions,
    const std::vector<std::string>& joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5",
                                                   "joint6"})
    -> std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> {
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>(topic, rclcpp::QoS(10));

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->now();
    msg.name = joint_names;
    msg.position = positions;

    pub->publish(msg);
    return pub;
}

// ---------------------------------------------------------------------------
// wait_for_action_server
// ---------------------------------------------------------------------------
/// Block until a FollowJointTrajectory action server appears at `name`, or
/// `timeout` elapses. Returns true if the server was found.
inline bool wait_for_action_server(rclcpp::Node* node, const std::string& name,
                                   std::chrono::seconds timeout = std::chrono::seconds(5)) {
    using Action = control_msgs::action::FollowJointTrajectory;
    auto client = rclcpp_action::create_client<Action>(node, name);
    return client->wait_for_action_server(timeout);
}

// ---------------------------------------------------------------------------
// spin_until
// ---------------------------------------------------------------------------
/// Spin `node` in a loop, checking `condition` each iteration. Returns when
/// the condition returns true or `duration` has elapsed.
inline void spin_until(rclcpp::Node* node, std::chrono::milliseconds duration,
                       std::function<bool()> condition) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node->get_node_base_interface());
        if (condition()) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

}  // namespace omr_controller::test
