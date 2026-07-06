#pragma once

#include <omr_controller/types.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace omr_controller {

class ArmClient {
public:
    explicit ArmClient(rclcpp::Node* node);

    /// Send a joint goal via FollowJointTrajectory action.
    /// Returns the goal handle (null if server unavailable).
    rclcpp_action::GoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    moveJoints(const JointGoal& goal);

    /// Return the most recently cached joint positions.
    std::vector<double> currentJoints() const;

    /// True while a goal is being executed.
    bool isMoving() const;

    /// Not implemented in V1.
    void movePose(const geometry_msgs::msg::Pose& pose);

    /// Cancel the active goal.
    void stop();

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Node* node_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr actionClient_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
    rclcpp_action::GoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr activeGoal_;
    std::vector<double> jointPositions_;
    mutable std::mutex mutex_;
};

}  // namespace omr_controller
