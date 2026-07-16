#pragma once

#include <atomic>
#include <memory>

#include "omr_controller/clients/arm_client.hpp"
#include "omr_controller/clients/gripper_client.hpp"
#include "omr_controller/srv/move_arm.hpp"
#include "omr_controller/srv/move_rail.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace omr_controller {

class RemoteControlNode : public rclcpp::Node {
public:
    explicit RemoteControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

private:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using MoveArmSrv = omr_controller::srv::MoveArm;
    using MoveRailSrv = omr_controller::srv::MoveRail;
    using SetBool = std_srvs::srv::SetBool;

    void moveArmCallback(const MoveArmSrv::Request::SharedPtr request,
                         MoveArmSrv::Response::SharedPtr response);
    void moveRailCallback(const MoveRailSrv::Request::SharedPtr request,
                          MoveRailSrv::Response::SharedPtr response);
    void setGripperCallback(const SetBool::Request::SharedPtr request,
                            SetBool::Response::SharedPtr response);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    std::unique_ptr<ArmClient> arm_;
    std::unique_ptr<GripperClient> gripper_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr dais_action_client_;

    rclcpp::Service<MoveArmSrv>::SharedPtr move_arm_svc_;
    rclcpp::Service<MoveRailSrv>::SharedPtr move_rail_svc_;
    rclcpp::Service<SetBool>::SharedPtr set_gripper_svc_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    std::atomic<bool> rail_busy_{false};
};

}  // namespace omr_controller
