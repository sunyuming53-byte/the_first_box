#include "omr_controller/clients/arm_client.hpp"

#include <stdexcept>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace omr_controller {

ArmClient::ArmClient(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    const std::string actionName = node_->has_parameter("arm_jtc_action")
                                       ? node_->get_parameter("arm_jtc_action").as_string()
                                       : "/arm_cm/follow_joint_trajectory";
    const std::string topicName = node_->has_parameter("joint_state_topic")
                                      ? node_->get_parameter("joint_state_topic").as_string()
                                      : "/joint_states";

    actionClient_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        node_, actionName);

    jointStateSub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        topicName, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });
}

void ArmClient::moveJoints(const JointGoal& goal) {
    if (!actionClient_->wait_for_action_server(std::chrono::seconds(1))) {
        return;
    }

    auto actionGoal = control_msgs::action::FollowJointTrajectory::Goal();
    for (int i = 1; i <= 6; ++i) {
        actionGoal.trajectory.joint_names.push_back("joint" + std::to_string(i));
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = goal.positions;
    point.time_from_start = rclcpp::Duration::from_seconds(
        goal.time_from_start_sec > 0.0 ? goal.time_from_start_sec : 1.0);

    actionGoal.trajectory.points.push_back(point);

    auto sendGoalOptions =
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    sendGoalOptions.goal_response_callback =
        [this](
            rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
                goalHandle) {
            std::lock_guard<std::mutex> lock(mutex_);
            activeGoal_ = goalHandle;
        };
    sendGoalOptions.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<
               control_msgs::action::FollowJointTrajectory>::WrappedResult&) {
            std::lock_guard<std::mutex> lock(mutex_);
            activeGoal_.reset();
        };

    actionClient_->async_send_goal(actionGoal, sendGoalOptions);
}

std::vector<double> ArmClient::currentJoints() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return jointPositions_;
}

bool ArmClient::isMoving() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return activeGoal_ != nullptr;
}

void ArmClient::movePose(const geometry_msgs::msg::Pose&) {
    throw std::logic_error("not implemented");
}

void ArmClient::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (activeGoal_) {
        actionClient_->async_cancel_goal(activeGoal_);
        activeGoal_.reset();
    }
}

bool ArmClient::actionServerReady(double timeout_s) {
    return actionClient_->wait_for_action_server(std::chrono::duration<double>(timeout_s));
}

void ArmClient::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    jointPositions_ = msg->position;
}

}  // namespace omr_controller
