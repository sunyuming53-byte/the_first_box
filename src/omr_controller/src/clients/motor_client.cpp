#include "omr_controller/clients/motor_client.hpp"

#include <cmath>
#include <cstddef>

#include <chrono>
#include <string>
#include <utility>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace omr_controller {

MotorClientImpl::MotorClientImpl(rclcpp::Node::SharedPtr node, std::string action_name,
                                 std::string joint_state_topic, std::string joint_name)
    : node_(std::move(node)), jointName_(std::move(joint_name)) {
    actionClient_ = rclcpp_action::create_client<FollowJointTrajectory>(node_, action_name);
    jointStateSub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });
}

bool MotorClientImpl::enable() {
    std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = true;
    state_.servo_enabled = true;
    return true;
}

bool MotorClientImpl::disable() {
    std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);

    bool wasEnabled;
    bool wasServoEnabled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        wasEnabled = enabled_;
        wasServoEnabled = state_.servo_enabled;
        enabled_ = false;
        state_.servo_enabled = false;
    }

    if (stop()) {
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = wasEnabled;
        state_.servo_enabled = wasServoEnabled;
    }
    return false;
}

bool MotorClientImpl::isEnabled() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

bool MotorClientImpl::setVelocity(double rad_per_s) {
    stopRequested_.store(false);
    if (!std::isfinite(rad_per_s)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_) {
            return false;
        }
    }

    return sendVelocityGoal(rad_per_s, 1.0, true);
}

MotorState MotorClientImpl::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

bool MotorClientImpl::stop() {
    stopRequested_.store(true);
    GoalHandle::SharedPtr goalToCancel;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        goalToCancel = activeGoal_;
        activeGoal_.reset();
    }

    if (goalToCancel) {
        actionClient_->async_cancel_goal(goalToCancel);
    }

    return sendVelocityGoal(0.0, 0.5, false);
}

bool MotorClientImpl::sendVelocityGoal(double rad_per_s, double duration_sec,
                                       bool require_enabled) {
    if (!actionClient_->wait_for_action_server(std::chrono::seconds(0))) {
        return false;
    }

    std::string jointName;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (require_enabled && !enabled_) {
            return false;
        }
        jointName = jointName_;
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {jointName};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.velocities = {rad_per_s};
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    goal.trajectory.points.push_back(point);

    auto sendGoalOptions = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    sendGoalOptions.goal_response_callback = [this,
                                              require_enabled](GoalHandle::SharedPtr goalHandle) {
        if (!goalHandle) {
            return;
        }

        if (require_enabled && stopRequested_.load()) {
            actionClient_->async_cancel_goal(goalHandle);
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        activeGoal_ = std::move(goalHandle);
    };
    sendGoalOptions.result_callback = [this](const GoalHandle::WrappedResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (activeGoal_ && activeGoal_->get_goal_id() == result.goal_id) {
            activeGoal_.reset();
        }
    };

    actionClient_->async_send_goal(goal, sendGoalOptions);
    return true;
}

void MotorClientImpl::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] != jointName_) {
            continue;
        }

        if (i < msg->position.size()) {
            state_.position_rad = msg->position[i];
        }
        if (i < msg->velocity.size()) {
            state_.velocity_rad_s = msg->velocity[i];
        }
        state_.connected = true;
        return;
    }
}

}  // namespace omr_controller
