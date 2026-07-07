#include "omr_controller/state_machine/door_trajectory_action.hpp"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <sstream>
#include <thread>

namespace omr_controller {

// =============================================================================
// Port definitions
// =============================================================================

BT::PortsList DoorTrajectoryAction::providedPorts() {
    return {
        // Geometry
        BT::InputPort<double>("r", 2.0, "Radial distance of door bottom endpoint from z-axis (m)"),
        BT::InputPort<double>("L", 1.5, "Half-length of the arm segment (m)"),
        BT::InputPort<double>("h", 0.0, "Height of the bottom endpoint above xy-plane (m)"),
        BT::InputPort<std::string>("hinge_transform", "0,0,0,0,0,0",
                                   "Arm-base → door-hinge tx,ty,tz,rx,ry,rz"),
        // Motion
        BT::InputPort<double>("theta_max_deg", 90.0, "Maximum door opening angle (deg)"),
        BT::InputPort<double>("theta_step_deg", 5.0, "Door opening angle step (deg)"),
        BT::InputPort<std::string>("phi_values", "0,15,30,45,60,75,90",
                                   "Comma-separated segment rotation angles (deg)"),
        BT::InputPort<std::string>("omega_values", "15,30,45,60,75,90",
                                   "Comma-separated gimbal rotation angles (deg)"),
        // Arm config
        BT::InputPort<std::string>("home_joints", "0,0,0,0,0,0",
                                   "Comma-separated home joint positions (rad)"),
        BT::InputPort<double>("joint_state_tolerance", 0.01,
                              "Joint position tolerance for completion check (rad)"),
        // Timing
        BT::InputPort<double>("idle_delay_sec", 1.0, "Initial idle delay before starting (s)"),
        // Collision geometry
        BT::InputPort<std::string>("door_panel_size", "2.0,0.05,0.8",
                                   "Door collision box dimensions x,y,z (m)"),
        BT::InputPort<double>("door_frame_radius", 0.05, "Door frame cylinder radius (m)"),
        // Planning
        BT::InputPort<std::string>("planning_frame", "base_link", "Planning scene frame id"),
    };
}

// =============================================================================
// Helpers
// =============================================================================

namespace {
/// Apply 4×4 homogeneous transform (CV_64F) to a geometry_msgs::Pose.
geometry_msgs::msg::Pose transformPose(const cv::Mat& T, const geometry_msgs::msg::Pose& src) {
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));
    tf2::Matrix3x3 rot_mat(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                           R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                           R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

    // Transform position: p_world = R * p_src + t
    geometry_msgs::msg::Pose dst;
    dst.position.x = R.at<double>(0, 0) * src.position.x + R.at<double>(0, 1) * src.position.y +
                     R.at<double>(0, 2) * src.position.z + T.at<double>(0, 3);
    dst.position.y = R.at<double>(1, 0) * src.position.x + R.at<double>(1, 1) * src.position.y +
                     R.at<double>(1, 2) * src.position.z + T.at<double>(1, 3);
    dst.position.z = R.at<double>(2, 0) * src.position.x + R.at<double>(2, 1) * src.position.y +
                     R.at<double>(2, 2) * src.position.z + T.at<double>(2, 3);

    // Transform orientation: q_world = q_R * q_src
    tf2::Quaternion q_src(src.orientation.x, src.orientation.y, src.orientation.z,
                          src.orientation.w);
    tf2::Quaternion q_R;
    rot_mat.getRotation(q_R);
    q_R.normalize();
    tf2::Quaternion q_result = q_R * q_src;

    dst.orientation.x = q_result.x();
    dst.orientation.y = q_result.y();
    dst.orientation.z = q_result.z();
    dst.orientation.w = q_result.w();
    return dst;
}
}  // namespace

std::vector<double> DoorTrajectoryAction::parseDoubles(const std::string& str) {
    std::vector<double> result;
    std::istringstream stream(str);
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (!token.empty()) {
            result.push_back(std::stod(token));
        }
    }
    return result;
}

const char* DoorTrajectoryAction::stateName(TrajectoryState s) {
    switch (s) {
        case TrajectoryState::IDLE: return "IDLE";
        case TrajectoryState::APPROACH_HOME: return "APPROACH_HOME";
        case TrajectoryState::PLAN_APPROACH: return "PLAN_APPROACH";
        case TrajectoryState::EXECUTE_APPROACH: return "EXECUTE_APPROACH";
        case TrajectoryState::PREPARE_WAYPOINTS: return "PREPARE_WAYPOINTS";
        case TrajectoryState::PLANNING_WAYPOINT: return "PLANNING_WAYPOINT";
        case TrajectoryState::EXECUTING_WAYPOINT: return "EXECUTING_WAYPOINT";
        case TrajectoryState::WAITING_COMPLETION: return "WAITING_COMPLETION";
        case TrajectoryState::DONE: return "DONE";
        case TrajectoryState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

// =============================================================================
// Constructor
// =============================================================================

DoorTrajectoryAction::DoorTrajectoryAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

// =============================================================================
// onStart — initialise parameters and subscribe to joint_states
// =============================================================================

BT::NodeStatus DoorTrajectoryAction::onStart() {
    // ── Get the ROS node from the blackboard ────────────────────────────────
    if (!config().blackboard->get("ros_node", ros_node_) || !ros_node_) {
        RCLCPP_FATAL(rclcpp::get_logger("door_trajectory"), "Blackboard key 'ros_node' not found");
        return BT::NodeStatus::FAILURE;
    }

    // ── Read ports ──────────────────────────────────────────────────────────
    r_ = 2.0;
    L_ = 1.5;
    h_ = 0.0;
    theta_max_deg_ = 90.0;
    theta_step_deg_ = 5.0;
    joint_state_tolerance_ = 0.01;
    idle_delay_sec_ = 1.0;
    door_frame_radius_ = 0.05;
    planning_frame_ = "base_link";
    T_armBase_doorHinge_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    phi_values_deg_ = {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0};
    home_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    door_panel_size_ = {2.0, 0.05, 0.8};

    getInput("r", r_);
    getInput("L", L_);
    getInput("h", h_);
    getInput("theta_max_deg", theta_max_deg_);
    getInput("theta_step_deg", theta_step_deg_);
    getInput("joint_state_tolerance", joint_state_tolerance_);
    getInput("idle_delay_sec", idle_delay_sec_);
    getInput("door_frame_radius", door_frame_radius_);
    getInput("planning_frame", planning_frame_);

    // Parse string ports
    std::string hinge_str;
    if (getInput("hinge_transform", hinge_str)) {
        T_armBase_doorHinge_ = parseDoubles(hinge_str);
    }
    if (T_armBase_doorHinge_.size() != 6) {
        T_armBase_doorHinge_.resize(6, 0.0);
    }

    std::string phi_str;
    if (getInput("phi_values", phi_str)) {
        phi_values_deg_ = parseDoubles(phi_str);
    }

    std::string omega_str;
    if (getInput("omega_values", omega_str)) {
        omega_values_deg_ = parseDoubles(omega_str);
    }
    if (omega_values_deg_.empty()) {
        omega_values_deg_ = {15.0, 30.0, 45.0, 60.0, 75.0, 90.0};
    }

    std::string home_str;
    if (getInput("home_joints", home_str)) {
        home_joints_ = parseDoubles(home_str);
    }
    if (home_joints_.size() != 6) {
        home_joints_.resize(6, 0.0);
    }

    std::string panel_str;
    if (getInput("door_panel_size", panel_str)) {
        door_panel_size_ = parseDoubles(panel_str);
    }
    if (door_panel_size_.size() != 3) {
        door_panel_size_ = {2.0, 0.05, 0.8};
    }

    // ── Compute and cache T_base_hinge from hinge_transform ──────────────────
    {
        double tx = T_armBase_doorHinge_[0];
        double ty = T_armBase_doorHinge_[1];
        double tz = T_armBase_doorHinge_[2];
        double rx = T_armBase_doorHinge_[3];
        double ry = T_armBase_doorHinge_[4];
        double rz = T_armBase_doorHinge_[5];

        double cr = std::cos(rx);
        double sr = std::sin(rx);
        double cp = std::cos(ry);
        double sp = std::sin(ry);
        double cy = std::cos(rz);
        double sy = std::sin(rz);

        cv::Mat R = cv::Mat_<double>(3, 3);
        R.at<double>(0, 0) = cy * cp;
        R.at<double>(0, 1) = cy * sp * sr - sy * cr;
        R.at<double>(0, 2) = cy * sp * cr + sy * sr;
        R.at<double>(1, 0) = sy * cp;
        R.at<double>(1, 1) = sy * sp * sr + cy * cr;
        R.at<double>(1, 2) = sy * sp * cr - cy * sr;
        R.at<double>(2, 0) = -sp;
        R.at<double>(2, 1) = cp * sr;
        R.at<double>(2, 2) = cp * cr;

        T_base_hinge_cache_ = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat roi = T_base_hinge_cache_(cv::Rect(0, 0, 3, 3));
        R.copyTo(roi);
        T_base_hinge_cache_.at<double>(0, 3) = tx;
        T_base_hinge_cache_.at<double>(1, 3) = ty;
        T_base_hinge_cache_.at<double>(2, 3) = tz;
    }

    // ── Subscribe to /joint_states ──────────────────────────────────────────
    joint_state_sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });

    // ── Reset state machine ─────────────────────────────────────────────────
    current_state_ = TrajectoryState::IDLE;
    current_waypoint_ = 0;
    waypoints_.clear();
    door_objects_added_ = false;
    move_group_.reset();
    planning_scene_.reset();

    idle_start_time_ = ros_node_->get_clock()->now();

    RCLCPP_INFO(ros_node_->get_logger(),
                "DoorTrajectoryAction started. "
                "r=%.2f L=%.2f h=%.2f theta_step=%.1fdeg theta_max=%.1fdeg "
                "%zu phi values %zu omega values",
                r_, L_, h_, theta_step_deg_, theta_max_deg_, phi_values_deg_.size(),
                omega_values_deg_.size());

    return BT::NodeStatus::RUNNING;
}

// =============================================================================
// onRunning — state machine (ported from DoorTrajectoryNode::tick())
// =============================================================================

BT::NodeStatus DoorTrajectoryAction::onRunning() {
    switch (current_state_) {
        // ── IDLE: wait for idle delay ──────────────────────────────────────
        case TrajectoryState::IDLE: {
            auto now = ros_node_->get_clock()->now();
            if ((now - idle_start_time_).seconds() >= idle_delay_sec_) {
                current_state_ = TrajectoryState::APPROACH_HOME;
                RCLCPP_INFO(ros_node_->get_logger(), "IDLE -> APPROACH_HOME");
            }
            return BT::NodeStatus::RUNNING;
        }

        // ── APPROACH_HOME: verify arm is at home; command home move if not ──
        case TrajectoryState::APPROACH_HOME: {
            if (isAtHome()) {
                approach_pose_ = computePoseForThetaPhiOmega(0.0, 0.0, 0.0);
                current_state_ = TrajectoryState::PLAN_APPROACH;
                RCLCPP_INFO(ros_node_->get_logger(),
                            "APPROACH_HOME -> PLAN_APPROACH (home confirmed)");
            } else {
                RCLCPP_WARN(ros_node_->get_logger(),
                            "Arm not at home position, commanding home move first");
                if (planAndExecuteJointHome()) {
                    approach_pose_ = computePoseForThetaPhiOmega(0.0, 0.0, 0.0);
                    current_state_ = TrajectoryState::PLAN_APPROACH;
                    RCLCPP_INFO(ros_node_->get_logger(),
                                "APPROACH_HOME -> PLAN_APPROACH (home commanded)");
                } else {
                    RCLCPP_ERROR(ros_node_->get_logger(), "Home approach failed");
                    current_state_ = TrajectoryState::ERROR;
                }
            }
            return (current_state_ == TrajectoryState::ERROR) ? BT::NodeStatus::FAILURE
                                                              : BT::NodeStatus::RUNNING;
        }

        // ── PLAN_APPROACH: collision-aware plan to first waypoint ──────────
        case TrajectoryState::PLAN_APPROACH: {
            setupDoorCollisionObjects();
            updateDoorPose(0.0);

            if (planAndExecuteToPose(approach_pose_)) {
                current_state_ = TrajectoryState::EXECUTE_APPROACH;
                RCLCPP_INFO(ros_node_->get_logger(),
                            "PLAN_APPROACH -> EXECUTE_APPROACH (pose reached)");
            } else {
                RCLCPP_ERROR(ros_node_->get_logger(), "Approach plan+execute failed");
                current_state_ = TrajectoryState::ERROR;
            }
            return (current_state_ == TrajectoryState::ERROR) ? BT::NodeStatus::FAILURE
                                                              : BT::NodeStatus::RUNNING;
        }

        // ── EXECUTE_APPROACH: proceed to waypoint preparation ──────────────
        case TrajectoryState::EXECUTE_APPROACH: {
            current_state_ = TrajectoryState::PREPARE_WAYPOINTS;
            RCLCPP_INFO(ros_node_->get_logger(), "EXECUTE_APPROACH -> PREPARE_WAYPOINTS");
            return BT::NodeStatus::RUNNING;
        }

        // ── PREPARE_WAYPOINTS: pre-compute all (θ, φ, ω) waypoint poses ──────
        case TrajectoryState::PREPARE_WAYPOINTS: {
            waypoints_.clear();
            current_waypoint_ = 0;
            waypoint_thetas_.clear();

            for (double theta_deg = 0.0; theta_deg <= theta_max_deg_ + 1e-9;
                 theta_deg += theta_step_deg_) {
                double theta_rad = theta_deg * M_PI / 180.0;
                for (double phi_deg : phi_values_deg_) {
                    double phi_rad = phi_deg * M_PI / 180.0;
                    for (double omega_deg : omega_values_deg_) {
                        double omega_rad = omega_deg * M_PI / 180.0;
                        waypoints_.push_back(
                            computePoseForThetaPhiOmega(theta_rad, phi_rad, omega_rad));
                        waypoint_thetas_.push_back(theta_rad);
                    }
                }
            }

            RCLCPP_INFO(ros_node_->get_logger(),
                        "PREPARE_WAYPOINTS: %zu waypoints computed "
                        "(%zu θ × %zu φ × %zu ω)",
                        waypoints_.size(),
                        static_cast<size_t>(std::ceil((theta_max_deg_ / theta_step_deg_) + 1)),
                        phi_values_deg_.size(), omega_values_deg_.size());

            if (waypoints_.empty()) {
                RCLCPP_ERROR(ros_node_->get_logger(), "No waypoints generated");
                current_state_ = TrajectoryState::ERROR;
                return BT::NodeStatus::FAILURE;
            }

            current_state_ = TrajectoryState::PLANNING_WAYPOINT;
            return BT::NodeStatus::RUNNING;
        }

        // ── PLANNING_WAYPOINT: plan+execute current waypoint pose ──────────
        case TrajectoryState::PLANNING_WAYPOINT: {
            if (current_waypoint_ >= waypoints_.size()) {
                current_state_ = TrajectoryState::DONE;
                RCLCPP_INFO(ros_node_->get_logger(),
                            "PLANNING_WAYPOINT -> DONE (no more waypoints)");
                return BT::NodeStatus::SUCCESS;
            }

            RCLCPP_INFO(ros_node_->get_logger(), "Planning waypoint %zu/%zu", current_waypoint_ + 1,
                        waypoints_.size());

            if (planAndExecuteToPose(waypoints_[current_waypoint_])) {
                current_state_ = TrajectoryState::EXECUTING_WAYPOINT;
            } else {
                RCLCPP_ERROR(ros_node_->get_logger(),
                             "Waypoint %zu plan+execute failed — aborting sequence",
                             current_waypoint_);
                current_state_ = TrajectoryState::ERROR;
            }
            return (current_state_ == TrajectoryState::ERROR) ? BT::NodeStatus::FAILURE
                                                              : BT::NodeStatus::RUNNING;
        }

        // ── EXECUTING_WAYPOINT: proceed to completion check ────────────────
        case TrajectoryState::EXECUTING_WAYPOINT: {
            current_state_ = TrajectoryState::WAITING_COMPLETION;
            return BT::NodeStatus::RUNNING;
        }

        // ── WAITING_COMPLETION: advance to next waypoint or finish ─────────
        case TrajectoryState::WAITING_COMPLETION: {
            current_waypoint_++;
            if (current_waypoint_ >= waypoints_.size()) {
                current_state_ = TrajectoryState::DONE;
                RCLCPP_INFO(ros_node_->get_logger(),
                            "WAITING_COMPLETION -> DONE (all %zu waypoints complete)",
                            waypoints_.size());
                return BT::NodeStatus::SUCCESS;
            }

            current_state_ = TrajectoryState::PLANNING_WAYPOINT;
            RCLCPP_INFO(ros_node_->get_logger(),
                        "WAITING_COMPLETION -> PLANNING_WAYPOINT (waypoint %zu)",
                        current_waypoint_ + 1);
            return BT::NodeStatus::RUNNING;
        }

        // ── DONE: terminal state ───────────────────────────────────────────
        case TrajectoryState::DONE: return BT::NodeStatus::SUCCESS;

        // ── ERROR: terminal state ──────────────────────────────────────────
        case TrajectoryState::ERROR: return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}

// =============================================================================
// onHalted — cleanup when BT interrupts this node
// =============================================================================

void DoorTrajectoryAction::onHalted() {
    RCLCPP_WARN(ros_node_->get_logger(), "DoorTrajectoryAction halted at state %s",
                stateName(current_state_));

    // Remove door collision objects if present
    if (planning_scene_ && door_objects_added_) {
        planning_scene_->removeCollisionObjects({"door_panel", "door_frame"});
        door_objects_added_ = false;
    }

    current_state_ = TrajectoryState::IDLE;
}

// =============================================================================
// MoveIt2 interface
// =============================================================================

void DoorTrajectoryAction::ensureMoveGroup() {
    if (move_group_) return;
    if (!ros_node_) return;

    if (!ros_node_->has_parameter("robot_description")) {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "robot_description parameter not set — "
                     "cannot initialise MoveGroupInterface");
        return;
    }

    try {
        moveit::planning_interface::MoveGroupInterface::Options opts("arm_group");
        move_group_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(ros_node_, opts);
        move_group_->setPlanningTime(5.0);
        RCLCPP_INFO(ros_node_->get_logger(), "MoveGroupInterface initialised for arm_group");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Failed to initialise MoveGroupInterface: %s",
                     e.what());
        move_group_.reset();
    }
}

// =============================================================================
// Joint state callback
// =============================================================================

void DoorTrajectoryAction::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joints_mutex_);
    current_joints_ = msg->position;
}

// =============================================================================
// Completion polling
// =============================================================================

bool DoorTrajectoryAction::waitForCompletion(const std::vector<double>& target_joints,
                                             double timeout_sec) {
    constexpr double kPollRateHz = 50.0;
    const auto poll_period = std::chrono::duration<double>(1.0 / kPollRateHz);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);

    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<double> current;
        {
            std::lock_guard<std::mutex> lock(joints_mutex_);
            current = current_joints_;
        }

        if (current.size() >= 6) {
            bool all_within_tolerance = true;
            for (size_t i = 0; i < 6; ++i) {
                if (std::abs(current[i] - target_joints[i]) >= joint_state_tolerance_) {
                    all_within_tolerance = false;
                    break;
                }
            }
            if (all_within_tolerance) {
                return true;
            }
        }

        std::this_thread::sleep_for(poll_period);
    }

    RCLCPP_WARN(ros_node_->get_logger(), "waitForCompletion: timeout after %.1f s", timeout_sec);
    return false;
}

// =============================================================================
// Plan + execute + poll
// =============================================================================

bool DoorTrajectoryAction::planAndExecuteToPose(const geometry_msgs::msg::Pose& target) {
    ensureMoveGroup();

    if (!move_group_) {
        RCLCPP_ERROR(ros_node_->get_logger(), "MoveGroupInterface not available — cannot plan");
        return false;
    }

    move_group_->setStartStateToCurrentState();

    if (!move_group_->setPoseTarget(target)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "setPoseTarget failed");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);

    if (!static_cast<bool>(plan_result)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "plan failed: %d", static_cast<int>(plan_result.val));
        return false;
    }

    RCLCPP_INFO(ros_node_->get_logger(), "plan succeeded (%.3f s planning time)",
                plan.planning_time_);

    moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);

    if (!static_cast<bool>(exec_result)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "execute failed: %d",
                     static_cast<int>(exec_result.val));
        return false;
    }

    const auto& points = plan.trajectory_.joint_trajectory.points;
    if (points.empty()) {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "plan trajectory has no points — cannot poll completion");
        return false;
    }

    const auto& target_joints = points.back().positions;
    if (target_joints.size() < 6) {
        RCLCPP_ERROR(ros_node_->get_logger(), "plan trajectory has fewer than 6 joints (%zu)",
                     target_joints.size());
        return false;
    }

    constexpr double kCompletionTimeoutSec = 10.0;
    return waitForCompletion(target_joints, kCompletionTimeoutSec);
}

// =============================================================================
// Joint-space home approach
// =============================================================================

bool DoorTrajectoryAction::planAndExecuteJointHome() {
    ensureMoveGroup();

    if (!move_group_) {
        RCLCPP_ERROR(ros_node_->get_logger(), "MoveGroupInterface not available");
        return false;
    }

    move_group_->setStartStateToCurrentState();

    if (!move_group_->setJointValueTarget(home_joints_)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "setJointValueTarget for home failed");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);

    if (!static_cast<bool>(plan_result)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "home plan failed: %d",
                     static_cast<int>(plan_result.val));
        return false;
    }

    RCLCPP_INFO(ros_node_->get_logger(), "home plan succeeded (%.3f s planning time)",
                plan.planning_time_);

    moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);

    if (!static_cast<bool>(exec_result)) {
        RCLCPP_ERROR(ros_node_->get_logger(), "home execute failed: %d",
                     static_cast<int>(exec_result.val));
        return false;
    }

    constexpr double kCompletionTimeoutSec = 10.0;
    return waitForCompletion(home_joints_, kCompletionTimeoutSec);
}

// =============================================================================
// Collision object builders
// =============================================================================

moveit_msgs::msg::CollisionObject
DoorTrajectoryAction::buildDoorPanelMsg(double theta_rad, const cv::Mat& T_base_hinge) const {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "door_panel";
    obj.header.frame_id = planning_frame_;
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {door_panel_size_[0], door_panel_size_[1], door_panel_size_[2]};

    // Pose in hinge frame: center at (width/2·cosθ, width/2·sinθ, height/2), rotated by θ
    // around z
    geometry_msgs::msg::Pose hinge_pose;
    hinge_pose.position.x = door_panel_size_[0] / 2.0 * std::cos(theta_rad);
    hinge_pose.position.y = door_panel_size_[0] / 2.0 * std::sin(theta_rad);
    hinge_pose.position.z = door_panel_size_[2] / 2.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_rad);
    hinge_pose.orientation.x = q.x();
    hinge_pose.orientation.y = q.y();
    hinge_pose.orientation.z = q.z();
    hinge_pose.orientation.w = q.w();

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(transformPose(T_base_hinge, hinge_pose));
    return obj;
}

moveit_msgs::msg::CollisionObject
DoorTrajectoryAction::buildDoorFrameMsg(const cv::Mat& T_base_hinge) const {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = "door_frame";
    obj.header.frame_id = planning_frame_;
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = door_panel_size_[2];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = door_frame_radius_;

    // Pose in hinge frame: at origin, orient cylinder along z
    geometry_msgs::msg::Pose hinge_pose;
    hinge_pose.position.x = 0.0;
    hinge_pose.position.y = 0.0;
    hinge_pose.position.z = door_panel_size_[2] / 2.0;
    hinge_pose.orientation.w = 1.0;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(transformPose(T_base_hinge, hinge_pose));
    return obj;
}

// =============================================================================
// Planning scene management
// =============================================================================

void DoorTrajectoryAction::setupDoorCollisionObjects() {
    if (door_objects_added_) return;

    if (!planning_scene_) {
        planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    }

    auto door_panel = buildDoorPanelMsg(0.0, T_base_hinge_cache_);
    planning_scene_->applyCollisionObject(door_panel);

    auto door_frame = buildDoorFrameMsg(T_base_hinge_cache_);
    planning_scene_->applyCollisionObject(door_frame);

    door_objects_added_ = true;
    RCLCPP_INFO(ros_node_->get_logger(),
                "Door collision objects added to planning scene: "
                "panel box %.2fx%.2fx%.2f m, frame cylinder r=%.2f m h=%.2f m "
                "in frame '%s'",
                door_panel_size_[0], door_panel_size_[1], door_panel_size_[2], door_frame_radius_,
                door_panel_size_[2], planning_frame_.c_str());
}

void DoorTrajectoryAction::updateDoorPose(double theta_rad) {
    auto door_panel = buildDoorPanelMsg(theta_rad, T_base_hinge_cache_);
    door_panel.operation = moveit_msgs::msg::CollisionObject::MOVE;

    if (planning_scene_) {
        planning_scene_->applyCollisionObject(door_panel);
    }
}

// =============================================================================
// Pose computation (math model)
// =============================================================================

geometry_msgs::msg::Pose DoorTrajectoryAction::computePoseForThetaPhiOmega(double theta_rad,
                                                                           double phi_rad,
                                                                           double omega_rad) const {
    cv::Mat T_hinge_target = computeWorldTTarget(theta_rad, phi_rad, omega_rad, r_, L_, h_);
    cv::Mat T_base_target = T_base_hinge_cache_ * T_hinge_target;

    return homogeneous_to_pose(T_base_target);
}

// =============================================================================
// Home check
// =============================================================================

bool DoorTrajectoryAction::isAtHome() const {
    std::vector<double> current;
    {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        current = current_joints_;
    }
    if (current.size() < home_joints_.size()) return false;
    for (size_t i = 0; i < home_joints_.size(); ++i) {
        if (std::abs(current[i] - home_joints_[i]) >= joint_state_tolerance_) {
            return false;
        }
    }
    return true;
}

}  // namespace omr_controller
