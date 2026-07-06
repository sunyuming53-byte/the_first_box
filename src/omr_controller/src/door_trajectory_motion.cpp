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

}  // namespace omr_controller
