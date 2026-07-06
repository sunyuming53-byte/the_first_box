#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "omr_controller/door_math.hpp"
#include "omr_controller/geometry_utils.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>

namespace omr_controller {

/// @brief States of the trajectory sequencing state machine.
enum class TrajectoryState {
    IDLE,                ///< Waiting for start signal.
    APPROACH_HOME,       ///< Verifying arm is at home position.
    PLAN_APPROACH,       ///< Planning + executing approach to first waypoint.
    EXECUTE_APPROACH,    ///< Verifying approach completion.
    PREPARE_WAYPOINTS,   ///< Pre-computing all (θ, φ) waypoint poses.
    PLANNING_WAYPOINT,   ///< Planning + executing a single waypoint.
    EXECUTING_WAYPOINT,  ///< Verifying waypoint completion.
    WAITING_COMPLETION,  ///< Checking if waypoint reached; advancing index.
    DONE,                ///< Trajectory sequence complete.
    ERROR                ///< Fatal error; sequence aborted.
};

/// @brief Standalone ROS node that generates and publishes a trajectory feed
///        for the door-opening task (6-DOF waypoints derived from the analytic
///        θ-φ parameterization).
///
/// Drives the (θ, φ) schedule, calls planAndExecuteToPose() for each waypoint,
/// and stops on failure.
class DoorTrajectoryNode : public rclcpp::Node {
public:
    explicit DoorTrajectoryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("door_trajectory_node", options) {
        // ── Geometry parameters ──────────────────────────────────────────
        r_ = declare_parameter("r", 2.0);
        L_ = declare_parameter("L", 1.5);
        h_ = declare_parameter("h", 0.0);

        // Arm-base → door-hinge transform: tx, ty, tz (m), rx, ry, rz (rad)
        T_armBase_doorHinge_ = declare_parameter(
            "T_armBase_doorHinge", std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

        // ── Motion parameters ────────────────────────────────────────────
        theta_step_deg_ = declare_parameter("theta_step_deg", 5.0);
        theta_max_deg_ = declare_parameter("theta_max_deg", 90.0);
        phi_values_deg_ = declare_parameter(
            "phi_values_deg", std::vector<double>({0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0}));

        // ── Arm configuration ────────────────────────────────────────────
        home_joints_ =
            declare_parameter("home_joints", std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
        joint_state_tolerance_ = declare_parameter("joint_state_tolerance", 0.01);

        // ── Timing ───────────────────────────────────────────────────────
        idle_delay_sec_ = declare_parameter("idle_delay_sec", 1.0);

        // ── Collision geometry (for MoveIt2 planning scene) ──────────────
        door_panel_size_ =
            declare_parameter("door_panel_size", std::vector<double>({2.0, 0.05, 0.8}));
        door_frame_radius_ = declare_parameter("door_frame_radius", 0.05);

        // ── Planning scene configuration ─────────────────────────────────
        planning_frame_ = declare_parameter("planning_frame", std::string("base_link"));

        // ── ROS interfaces ───────────────────────────────────────────────
        state_pub_ = create_publisher<std_msgs::msg::String>("~/state", 10);

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCallback(msg); });

        double tick_rate = declare_parameter("tick_rate", 10.0);
        auto period = std::chrono::duration<double>(1.0 / tick_rate);
        tick_timer_ = create_wall_timer(period, [this]() { tick(); });

        idle_start_time_ = get_clock()->now();

        publishState("IDLE");

        RCLCPP_INFO(get_logger(),
                    "DoorTrajectoryNode started. "
                    "r=%.2f L=%.2f h=%.2f theta_step=%.1fdeg theta_max=%.1fdeg "
                    "tick_rate=%.1f Hz idle_delay=%.1fs",
                    r_, L_, h_, theta_step_deg_, theta_max_deg_, tick_rate, idle_delay_sec_);
    }

    /// @brief Periodic tick — drives the trajectory sequencing state machine.
    void tick();

    // ── MoveIt2 interface ────────────────────────────────────────────────

    /// @brief Plan and execute a Cartesian pose target using MoveGroupInterface.
    ///
    /// Calls setPoseTarget() → plan() → execute() on the underlying
    /// MoveGroupInterface, then polls /joint_states until the arm reaches the
    /// final trajectory point or a timeout expires.  This polling is required
    /// because the JTC is configured for open-loop control (execute() returns
    /// immediately).
    ///
    /// @param target  Target end-effector pose in the planning frame.
    /// @return true if the trajectory was planned, executed, and completed
    ///              within tolerance, false otherwise.
    virtual bool planAndExecuteToPose(const geometry_msgs::msg::Pose& target);

    // ── Joint-space motion (for test mocking) ─────────────────────────────

    /// @brief Plan and execute a joint-space move to the home position.
    ///        Uses setJointValueTarget(home_joints_) instead of setPoseTarget.
    ///        Virtual for test mocking.
    virtual bool planAndExecuteJointHome();

    // ── Collision object management ───────────────────────────────────────

    /// @brief Set up door collision objects (panel + frame) in the planning
    ///        scene.  Idempotent — only adds objects once per node lifetime.
    ///        Virtual for test mocking.
    virtual void setupDoorCollisionObjects();

    /// @brief Update the door panel pose as a function of opening angle θ.
    ///        Rotation is about the Z-axis (hinge axis per the math model
    ///        in docs/line-segment-rotation.tex §1).
    ///
    ///        The panel is updated in-place via operation=MOVE; the door frame
    ///        cylinder is NOT modified (it is stationary at the hinge axis).
    ///
    /// @param theta_rad  Door opening angle in radians (positive = open).
    void updateDoorPose(double theta_rad);

    // ── Public accessors (for testing) ───────────────────────────────────

    double r() const { return r_; }
    double L() const { return L_; }
    double h() const { return h_; }
    const std::vector<double>& T_armBase_doorHinge() const { return T_armBase_doorHinge_; }
    double theta_step_deg() const { return theta_step_deg_; }
    double theta_max_deg() const { return theta_max_deg_; }
    const std::vector<double>& phi_values_deg() const { return phi_values_deg_; }
    const std::vector<double>& home_joints() const { return home_joints_; }
    double joint_state_tolerance() const { return joint_state_tolerance_; }
    double idle_delay_sec() const { return idle_delay_sec_; }
    const std::vector<double>& door_panel_size() const { return door_panel_size_; }
    double door_frame_radius() const { return door_frame_radius_; }

protected:
    // ── Exposed for test injection ───────────────────────────────────────

    /// @brief Replace the internal MoveGroupInterface (for test mocking).
    void
    setMoveGroupForTesting(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg) {
        move_group_ = std::move(mg);
    }

    /// @brief Directly set cached joint positions (for test injection).
    void setJointPositionsForTesting(const std::vector<double>& positions) {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        current_joints_ = positions;
    }

    /// @brief Read cached joint positions (for test assertions).
    std::vector<double> getJointPositionsForTesting() const {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        return current_joints_;
    }

    /// @brief Get current state machine state (for test assertions).
    TrajectoryState getCurrentState() const { return current_state_; }

    /// @brief Get pre-computed waypoints (for test assertions).
    const std::vector<geometry_msgs::msg::Pose>& getWaypoints() const { return waypoints_; }

    /// @brief Get current waypoint index (for test assertions).
    size_t getCurrentWaypointIndex() const { return current_waypoint_; }

    /// @brief Inject idle start time (for test acceleration).
    void setIdleStartTimeForTesting(const rclcpp::Time& t) { idle_start_time_ = t; }

    // ── Members exposed for test access ──────────────────────────────────

    /// @brief MoveGroupInterface — lazy-initialised on first planAndExecuteToPose().
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    /// @brief PlanningSceneInterface — lazy-initialised, for managing door
    ///        collision objects.  Stored as a pointer so the blocking default
    ///        constructor (which waits for /planning_scene services) is not
    ///        called during node construction.
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

    /// @brief Whether door collision objects have been added to the scene.
    bool door_objects_added_{false};

    std::vector<double> current_joints_;
    mutable std::mutex joints_mutex_;

    // ── State machine members ────────────────────────────────────────────

    TrajectoryState current_state_ = TrajectoryState::IDLE;
    rclcpp::Time idle_start_time_;
    std::vector<geometry_msgs::msg::Pose> waypoints_;
    std::vector<double> waypoint_thetas_;
    size_t current_waypoint_ = 0;
    geometry_msgs::msg::Pose approach_pose_;

    // ── MoveIt2 helpers (protected for test access) ──────────────────────

    /// @brief Ensure move_group_ is initialised (called on first use).
    void ensureMoveGroup();

    /// @brief Handler for /joint_states subscription.
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /// @brief Poll current_joints_ until all 6 joints are within tolerance of
    ///        the target or timeout_sec elapses.
    ///
    /// @return true if all joints are within joint_state_tolerance_,
    ///         false on timeout.
    bool waitForCompletion(const std::vector<double>& target_joints, double timeout_sec);

    // ── Collision object builders (protected for test access) ──────────────

    /// @brief Build the door_panel CollisionObject message.
    ///        The panel is a thin box rotated about Z by theta_rad.
    ///        Pose is at origin (hinge axis).
    moveit_msgs::msg::CollisionObject buildDoorPanelMsg(double theta_rad) const;

    /// @brief Build the door_frame CollisionObject message.
    ///        The frame is a stationary cylinder at the hinge axis (Z-axis).
    moveit_msgs::msg::CollisionObject buildDoorFrameMsg() const;

private:
    /// @brief Publish current state as a string on ~/state.
    void publishState(const std::string& state) {
        auto msg = std_msgs::msg::String();
        msg.data = state;
        state_pub_->publish(msg);
    }

    /// @brief Compute a single waypoint pose for given (θ, φ) in radians.
    ///
    /// Builds the world-frame target pose from the analytic model, then
    /// transforms it into the arm-base frame via T_armBase_doorHinge_.
    geometry_msgs::msg::Pose computePoseForThetaPhi(double theta_rad, double phi_rad) const {
        // Step 1: Build 4x4 homogeneous T_base_hinge from {tx,ty,tz,rx,ry,rz}
        //         following the arm_pose_to_homogeneous pattern in
        //         realman_calibration/src/transform.cpp.
        double tx = T_armBase_doorHinge_[0];
        double ty = T_armBase_doorHinge_[1];
        double tz = T_armBase_doorHinge_[2];
        double rx = T_armBase_doorHinge_[3];  // roll
        double ry = T_armBase_doorHinge_[4];  // pitch
        double rz = T_armBase_doorHinge_[5];  // yaw

        double cr = std::cos(rx);
        double sr = std::sin(rx);
        double cp = std::cos(ry);
        double sp = std::sin(ry);
        double cy = std::cos(rz);
        double sy = std::sin(rz);

        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
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

        cv::Mat T_base_hinge = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat roi = T_base_hinge(cv::Rect(0, 0, 3, 3));
        R.copyTo(roi);
        T_base_hinge.at<double>(0, 3) = tx;
        T_base_hinge.at<double>(1, 3) = ty;
        T_base_hinge.at<double>(2, 3) = tz;

        // Step 2: Compute hinge-frame → target-frame transform.
        cv::Mat T_hinge_target = computeWorldTTarget(theta_rad, phi_rad, r_, L_, h_);

        // Step 3: Chain transforms: base_T_target = base_T_hinge * hinge_T_target
        cv::Mat T_base_target = T_base_hinge * T_hinge_target;

        // Step 4: Convert to geometry_msgs::Pose.
        return homogeneous_to_pose(T_base_target);
    }

    /// @brief Check whether all 6 cached joint positions are within
    ///        joint_state_tolerance_ of home_joints_.
    bool isAtHome() const {
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

    // ── Parameters ───────────────────────────────────────────────────────
    double r_;
    double L_;
    double h_;
    std::vector<double> T_armBase_doorHinge_;
    double theta_step_deg_;
    double theta_max_deg_;
    std::vector<double> phi_values_deg_;
    std::vector<double> home_joints_;
    double joint_state_tolerance_;
    double idle_delay_sec_;
    std::vector<double> door_panel_size_;
    double door_frame_radius_;
    std::string planning_frame_;

    // ── ROS interfaces ───────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr tick_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

// ── tick() — trajectory sequencing state machine ──────────────────────────

inline void DoorTrajectoryNode::tick() {
    switch (current_state_) {
        // ── IDLE: wait for start signal (timer-based delay) ──────────────────
        case TrajectoryState::IDLE: {
            auto now = get_clock()->now();
            if ((now - idle_start_time_).seconds() >= idle_delay_sec_) {
                current_state_ = TrajectoryState::APPROACH_HOME;
                publishState("APPROACH_HOME");
                RCLCPP_INFO(get_logger(), "IDLE → APPROACH_HOME");
            }
            break;
        }

        // ── APPROACH_HOME: verify arm is at home; command home move if not ──
        case TrajectoryState::APPROACH_HOME: {
            if (isAtHome()) {
                approach_pose_ = computePoseForThetaPhi(0.0, 0.0);
                current_state_ = TrajectoryState::PLAN_APPROACH;
                publishState("PLAN_APPROACH");
                RCLCPP_INFO(get_logger(), "APPROACH_HOME → PLAN_APPROACH (home confirmed)");
            } else {
                RCLCPP_WARN(get_logger(), "Arm not at home position, commanding home move first");
                if (planAndExecuteJointHome()) {
                    approach_pose_ = computePoseForThetaPhi(0.0, 0.0);
                    current_state_ = TrajectoryState::PLAN_APPROACH;
                    publishState("PLAN_APPROACH");
                    RCLCPP_INFO(get_logger(), "APPROACH_HOME → PLAN_APPROACH (home commanded)");
                } else {
                    RCLCPP_ERROR(get_logger(), "Home approach failed");
                    current_state_ = TrajectoryState::ERROR;
                    publishState("ERROR");
                }
            }
            break;
        }

        // ── PLAN_APPROACH: collision-aware plan to first waypoint ────────────
        case TrajectoryState::PLAN_APPROACH: {
            setupDoorCollisionObjects();
            updateDoorPose(0.0);

            if (planAndExecuteToPose(approach_pose_)) {
                current_state_ = TrajectoryState::EXECUTE_APPROACH;
                publishState("EXECUTE_APPROACH");
                RCLCPP_INFO(get_logger(), "PLAN_APPROACH → EXECUTE_APPROACH (pose reached)");
            } else {
                RCLCPP_ERROR(get_logger(), "Approach plan+execute failed");
                current_state_ = TrajectoryState::ERROR;
                publishState("ERROR");
            }
            break;
        }

        // ── EXECUTE_APPROACH: completion verified internally; proceed ────────
        case TrajectoryState::EXECUTE_APPROACH: {
            current_state_ = TrajectoryState::PREPARE_WAYPOINTS;
            publishState("PREPARE_WAYPOINTS");
            RCLCPP_INFO(get_logger(), "EXECUTE_APPROACH → PREPARE_WAYPOINTS");
            break;
        }

        // ── PREPARE_WAYPOINTS: pre-compute all (θ, φ) waypoint poses ────────
        case TrajectoryState::PREPARE_WAYPOINTS: {
            waypoints_.clear();
            current_waypoint_ = 0;

            for (double theta_deg = 0.0; theta_deg <= theta_max_deg_ + 1e-9;
                 theta_deg += theta_step_deg_) {
                double theta_rad = theta_deg * M_PI / 180.0;
                for (double phi_deg : phi_values_deg_) {
                    double phi_rad = phi_deg * M_PI / 180.0;
                    waypoints_.push_back(computePoseForThetaPhi(theta_rad, phi_rad));
                }
            }

            RCLCPP_INFO(get_logger(), "PREPARE_WAYPOINTS: %zu waypoints computed",
                        waypoints_.size());

            if (waypoints_.empty()) {
                RCLCPP_ERROR(get_logger(), "No waypoints generated");
                current_state_ = TrajectoryState::ERROR;
                publishState("ERROR");
            } else {
                current_state_ = TrajectoryState::PLANNING_WAYPOINT;
                publishState("PLANNING_WAYPOINT");
            }
            break;
        }

        // ── PLANNING_WAYPOINT: plan+execute current waypoint pose ────────────
        case TrajectoryState::PLANNING_WAYPOINT: {
            if (current_waypoint_ >= waypoints_.size()) {
                current_state_ = TrajectoryState::DONE;
                publishState("DONE");
                RCLCPP_INFO(get_logger(), "PLANNING_WAYPOINT → DONE (no more waypoints)");
                break;
            }

            RCLCPP_INFO(get_logger(), "Planning waypoint %zu/%zu", current_waypoint_ + 1,
                        waypoints_.size());

            if (planAndExecuteToPose(waypoints_[current_waypoint_])) {
                current_state_ = TrajectoryState::EXECUTING_WAYPOINT;
                publishState("EXECUTING_WAYPOINT");
            } else {
                RCLCPP_ERROR(get_logger(), "Waypoint %zu plan+execute failed — aborting sequence",
                             current_waypoint_);
                current_state_ = TrajectoryState::ERROR;
                publishState("ERROR");
            }
            break;
        }

        // ── EXECUTING_WAYPOINT: completion verified internally; proceed ──────
        case TrajectoryState::EXECUTING_WAYPOINT: {
            current_state_ = TrajectoryState::WAITING_COMPLETION;
            publishState("WAITING_COMPLETION");
            break;
        }

        // ── WAITING_COMPLETION: advance to next waypoint or finish ───────────
        case TrajectoryState::WAITING_COMPLETION: {
            current_waypoint_++;
            if (current_waypoint_ >= waypoints_.size()) {
                current_state_ = TrajectoryState::DONE;
                publishState("DONE");
                RCLCPP_INFO(get_logger(), "WAITING_COMPLETION → DONE (all %zu waypoints complete)",
                            waypoints_.size());
            } else {
                current_state_ = TrajectoryState::PLANNING_WAYPOINT;
                publishState("PLANNING_WAYPOINT");
                RCLCPP_INFO(get_logger(), "WAITING_COMPLETION → PLANNING_WAYPOINT (waypoint %zu)",
                            current_waypoint_ + 1);
            }
            break;
        }

        // ── DONE: trajectory sequence complete ───────────────────────────────
        case TrajectoryState::DONE: {
            RCLCPP_INFO(get_logger(), "Trajectory sequence DONE (%zu waypoints completed)",
                        waypoints_.size());
            tick_timer_->cancel();
            break;
        }

        // ── ERROR: terminal state (no recovery) ──────────────────────────────
        case TrajectoryState::ERROR: {
            RCLCPP_ERROR(get_logger(), "Trajectory sequence ERROR at waypoint %zu/%zu",
                         current_waypoint_, waypoints_.size());
            tick_timer_->cancel();
            break;
        }
    }
}

}  // namespace omr_controller
