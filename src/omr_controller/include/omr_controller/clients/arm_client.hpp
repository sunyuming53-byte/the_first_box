#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "omr_controller/types.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace omr_controller {

class ArmClient {
public:
    explicit ArmClient(rclcpp::Node::SharedPtr node);

    /// Send a joint goal via FollowJointTrajectory action (fire-and-forget).
    void moveJoints(const JointGoal& goal);

    /// Return the most recently cached joint positions.
    std::vector<double> currentJoints() const;

    /// True while a goal is being executed.
    bool isMoving() const;

    /// Not implemented in V1.
    void movePose(const geometry_msgs::msg::Pose& pose);

    /// Cancel the active goal.
    void stop();

    /// Check if action server is available (for pre-check before sending goal).
    /// Returns true if the action server responds within timeout_s seconds.
    bool actionServerReady(double timeout_s = 1.0);

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr actionClient_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
    rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
        activeGoal_;
    std::vector<double> jointPositions_;
    mutable std::mutex mutex_;
};

}  // namespace omr_controller
