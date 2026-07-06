#include "omr_controller/door_trajectory_node.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

namespace omr_controller {

// ── Lazy initialisation ───────────────────────────────────────────────────

void DoorTrajectoryNode::ensureMoveGroup() {
    if (move_group_) return;

    if (!has_parameter("robot_description")) {
        RCLCPP_ERROR(get_logger(),
                     "robot_description parameter not set — "
                     "cannot initialise MoveGroupInterface");
        return;
    }

    try {
        moveit::planning_interface::MoveGroupInterface::Options opts("arm_group");
        move_group_ = std::make_shared<
            moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), opts);
        move_group_->setPlanningTime(5.0);

        RCLCPP_INFO(get_logger(),
                    "MoveGroupInterface initialised for arm_group");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),
                     "Failed to initialise MoveGroupInterface: %s", e.what());
        move_group_.reset();
    }
}

// ── Joint state subscription ──────────────────────────────────────────────

void DoorTrajectoryNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    current_joints_ = msg->position;
}

// ── Completion polling ────────────────────────────────────────────────────

bool DoorTrajectoryNode::waitForCompletion(
    const std::vector<double>& target_joints, double timeout_sec) {
    constexpr double kPollRateHz = 50.0;
    const auto poll_period = std::chrono::duration<double>(1.0 / kPollRateHz);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_sec);

    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<double> current;
        {
            std::lock_guard<std::mutex> lock(joints_mutex_);
            current = current_joints_;
        }

        if (current.size() >= 6) {
            bool all_within_tolerance = true;
            for (size_t i = 0; i < 6; ++i) {
                if (std::abs(current[i] - target_joints[i]) >=
                    joint_state_tolerance_) {
                    all_within_tolerance = false;
                    break;
                }
            }
            if (all_within_tolerance) {
                RCLCPP_DEBUG(get_logger(),
                             "waitForCompletion: all joints within tolerance");
                return true;
            }
        }

        std::this_thread::sleep_for(poll_period);
    }

    RCLCPP_WARN(get_logger(),
                "waitForCompletion: timeout after %.1f s", timeout_sec);
    return false;
}

// ── Plan + execute + poll ─────────────────────────────────────────────────

bool DoorTrajectoryNode::planAndExecuteToPose(
    const geometry_msgs::msg::Pose& target) {
    ensureMoveGroup();

    if (!move_group_) {
        RCLCPP_ERROR(get_logger(),
                     "MoveGroupInterface not available — cannot plan");
        return false;
    }

    move_group_->setStartStateToCurrentState();

    if (!move_group_->setPoseTarget(target)) {
        RCLCPP_ERROR(get_logger(), "setPoseTarget failed");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);

    if (!static_cast<bool>(plan_result)) {
        RCLCPP_ERROR(get_logger(), "plan failed: %d",
                     static_cast<int>(plan_result.val));
        return false;
    }

    RCLCPP_INFO(get_logger(), "plan succeeded (%.3f s planning time)",
                plan.planning_time_);

    moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);

    if (!static_cast<bool>(exec_result)) {
        RCLCPP_ERROR(get_logger(), "execute failed: %d",
                     static_cast<int>(exec_result.val));
        return false;
    }

    // Extract target joint positions from the planned trajectory's final
    // point so we can poll /joint_states for open-loop completion.
    const auto& points =
        plan.trajectory_.joint_trajectory.points;
    if (points.empty()) {
        RCLCPP_ERROR(get_logger(),
                     "plan trajectory has no points — cannot poll completion");
        return false;
    }

    const auto& target_joints = points.back().positions;
    if (target_joints.size() < 6) {
        RCLCPP_ERROR(get_logger(),
                     "plan trajectory has fewer than 6 joints (%zu)",
                     target_joints.size());
        return false;
    }

    // poll /joint_states for completion (open-loop JTC workaround)
    constexpr double kCompletionTimeoutSec = 10.0;
    return waitForCompletion(target_joints, kCompletionTimeoutSec);
}

// ── Collision object builders ──────────────────────────────────────────────

moveit_msgs::msg::CollisionObject DoorTrajectoryNode::buildDoorPanelMsg(
    double theta_rad) const {
    moveit_msgs::msg::CollisionObject door_panel;
    door_panel.id = "door_panel";
    door_panel.header.frame_id = planning_frame_;
    door_panel.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] =
        door_panel_size_[0];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] =
        door_panel_size_[1];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] =
        door_panel_size_[2];

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_rad);

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    door_panel.primitives.push_back(primitive);
    door_panel.primitive_poses.push_back(pose);

    return door_panel;
}

moveit_msgs::msg::CollisionObject DoorTrajectoryNode::buildDoorFrameMsg() const {
    moveit_msgs::msg::CollisionObject door_frame;
    door_frame.id = "door_frame";
    door_frame.header.frame_id = planning_frame_;
    door_frame.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
        door_panel_size_[2];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
        door_frame_radius_;

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;

    door_frame.primitives.push_back(primitive);
    door_frame.primitive_poses.push_back(pose);

    return door_frame;
}

// ── Planning scene management ──────────────────────────────────────────────

void DoorTrajectoryNode::setupDoorCollisionObjects() {
    if (door_objects_added_) return;

    if (!planning_scene_) {
        planning_scene_ =
            std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    }

    auto door_panel = buildDoorPanelMsg(0.0);
    planning_scene_->applyCollisionObject(door_panel);

    auto door_frame = buildDoorFrameMsg();
    planning_scene_->applyCollisionObject(door_frame);

    door_objects_added_ = true;
    RCLCPP_INFO(get_logger(),
                "Door collision objects added to planning scene: "
                "panel box %.2fx%.2fx%.2f m, frame cylinder r=%.2f m h=%.2f m "
                "in frame '%s'",
                door_panel_size_[0], door_panel_size_[1], door_panel_size_[2],
                door_frame_radius_, door_panel_size_[2],
                planning_frame_.c_str());
}

void DoorTrajectoryNode::updateDoorPose(double theta_rad) {
    auto door_panel = buildDoorPanelMsg(theta_rad);
    door_panel.operation = moveit_msgs::msg::CollisionObject::MOVE;

    if (planning_scene_) {
        planning_scene_->applyCollisionObject(door_panel);
    }
}

// ── Joint-space home approach ──────────────────────────────────────────────

bool DoorTrajectoryNode::planAndExecuteJointHome() {
    ensureMoveGroup();

    if (!move_group_) {
        RCLCPP_ERROR(get_logger(),
                     "MoveGroupInterface not available — cannot plan home");
        return false;
    }

    move_group_->setStartStateToCurrentState();

    if (!move_group_->setJointValueTarget(home_joints_)) {
        RCLCPP_ERROR(get_logger(), "setJointValueTarget for home failed");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);

    if (!static_cast<bool>(plan_result)) {
        RCLCPP_ERROR(get_logger(), "home plan failed: %d",
                     static_cast<int>(plan_result.val));
        return false;
    }

    RCLCPP_INFO(get_logger(), "home plan succeeded (%.3f s planning time)",
                plan.planning_time_);

    moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);

    if (!static_cast<bool>(exec_result)) {
        RCLCPP_ERROR(get_logger(), "home execute failed: %d",
                     static_cast<int>(exec_result.val));
        return false;
    }

    constexpr double kCompletionTimeoutSec = 10.0;
    return waitForCompletion(home_joints_, kCompletionTimeoutSec);
}

}  // namespace omr_controller
