#include "remote_control_node.hpp"

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace omr_controller {

RemoteControlNode::RemoteControlNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("remote_control", options) {
    declare_parameter("arm_jtc_action", "/arm_cm/follow_joint_trajectory");
    declare_parameter("dais_jtc_action",
                      "/dais_joint_trajectory_controller/follow_joint_trajectory");
    declare_parameter("gripper_action", "/gripper/follow_joint_trajectory");
    declare_parameter("cmd_vel_topic", "/m65_controller_manager/diff_drive_controller/cmd_vel");

    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {});
    arm_ = std::make_unique<ArmClient>(node_ptr);
    const auto gripper_action = get_parameter("gripper_action").as_string();
    gripper_ = std::make_unique<GripperClient>(node_ptr, gripper_action);

    const auto dais_action = get_parameter("dais_jtc_action").as_string();
    dais_action_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(node_ptr, dais_action);

    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    move_arm_svc_ = create_service<MoveArmSrv>(
        "~/move_arm", [this](const MoveArmSrv::Request::SharedPtr req,
                             MoveArmSrv::Response::SharedPtr res) { moveArmCallback(req, res); });

    move_rail_svc_ =
        create_service<MoveRailSrv>("~/move_rail", [this](const MoveRailSrv::Request::SharedPtr req,
                                                          MoveRailSrv::Response::SharedPtr res) {
            moveRailCallback(req, res);
        });

    set_gripper_svc_ =
        create_service<SetBool>("~/set_gripper", [this](const SetBool::Request::SharedPtr req,
                                                        SetBool::Response::SharedPtr res) {
            setGripperCallback(req, res);
        });

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(msg); });

    RCLCPP_INFO(get_logger(), "RemoteControl ready");
}

void RemoteControlNode::moveArmCallback(const MoveArmSrv::Request::SharedPtr request,
                                        MoveArmSrv::Response::SharedPtr response) {
    if (!arm_->actionServerReady(1.0)) {
        response->success = false;
        response->message = "action server unavailable";
        return;
    }

    if (arm_->isMoving()) {
        response->success = false;
        response->message = "busy";
        return;
    }

    JointGoal goal;
    goal.positions.assign(request->positions.begin(), request->positions.end());
    goal.time_from_start_sec = request->time_from_start > 0.0 ? request->time_from_start : 1.0;
    goal.speed_ratio = 50;

    arm_->moveJoints(goal);
    response->success = true;
    response->message = "accept";
}

void RemoteControlNode::moveRailCallback(const MoveRailSrv::Request::SharedPtr request,
                                         MoveRailSrv::Response::SharedPtr response) {
    if (!dais_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        response->success = false;
        response->message = "action server unavailable";
        return;
    }

    if (rail_busy_.exchange(true)) {
        response->success = false;
        response->message = "busy";
        return;
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {"joint_dais"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {request->position};
    point.time_from_start = rclcpp::Duration::from_seconds(3.0);
    goal.trajectory.points.push_back(point);

    response->success = true;
    response->message = "accept";

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr handle) {
            if (!handle) {
                rail_busy_ = false;
            }
        };
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult&) {
            rail_busy_ = false;
        };

    dais_action_client_->async_send_goal(goal, send_goal_options);
}

void RemoteControlNode::setGripperCallback(const SetBool::Request::SharedPtr request,
                                           SetBool::Response::SharedPtr response) {
    bool ok;
    if (request->data) {
        ok = gripper_->open(50);
    } else {
        ok = gripper_->close(50);
    }
    response->success = ok;
    response->message = ok ? "accepted" : "action server unavailable";
}

void RemoteControlNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_pub_->publish(*msg);
}

}  // namespace omr_controller
