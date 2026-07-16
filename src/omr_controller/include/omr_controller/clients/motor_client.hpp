#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "omr_controller/types.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace omr_controller {

class MotorClient {
public:
    virtual ~MotorClient() = default;

    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool isEnabled() const = 0;
    virtual bool setVelocity(double rad_per_s) = 0;
    virtual MotorState getState() const = 0;
    virtual bool stop() = 0;
};

class MotorClientStub : public MotorClient {
public:
    bool enable() override { return false; }
    bool disable() override { return false; }
    bool isEnabled() const override { return false; }
    bool setVelocity(double /*rad_per_s*/) override { return false; }
    MotorState getState() const override { return {}; }
    bool stop() override { return false; }
};

class MotorClientImpl : public MotorClient {
public:
    explicit MotorClientImpl(
        rclcpp::Node::SharedPtr node,
        std::string action_name = "/dais_joint_trajectory_controller/follow_joint_trajectory",
        std::string joint_state_topic = "/joint_states", std::string joint_name = "joint_dais");

    bool enable() override;
    bool disable() override;
    bool isEnabled() const override;
    bool setVelocity(double rad_per_s) override;
    MotorState getState() const override;
    bool stop() override;

private:
    friend class MotorClientImplTestAccess;

    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    bool sendVelocityGoal(double rad_per_s, double duration_sec, bool require_enabled);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    std::mutex lifecycleMutex_;
    mutable std::mutex mutex_;
    MotorState state_;
    bool enabled_{false};
    std::string jointName_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr actionClient_;
    GoalHandle::SharedPtr activeGoal_;
    std::atomic<bool> stopRequested_{false};
};

}  // namespace omr_controller
