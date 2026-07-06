#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace omr_controller {

/// @brief Standalone ROS node that generates and publishes a trajectory feed
///        for the door-opening task (6-DOF waypoints derived from the analytic
///        θ-φ parameterization).
///
/// This is T6 of the moveit2-door-trajectory plan — skeleton only.  The
/// state machine, MoveIt2 interfaces, and BT integration are not implemented
/// yet.  The node loads all geometry/motion parameters as ROS parameters and
/// runs a periodic tick() stub.
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
            "T_armBase_doorHinge",
            std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

        // ── Motion parameters ────────────────────────────────────────────
        theta_step_deg_ = declare_parameter("theta_step_deg", 5.0);
        phi_values_deg_ = declare_parameter(
            "phi_values_deg",
            std::vector<double>({0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0}));

        // ── Arm configuration ────────────────────────────────────────────
        home_joints_ = declare_parameter(
            "home_joints",
            std::vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
        joint_state_tolerance_ = declare_parameter("joint_state_tolerance", 0.01);

        // ── Collision geometry (for MoveIt2 planning scene) ──────────────
        door_panel_size_ = declare_parameter(
            "door_panel_size",
            std::vector<double>({2.0, 0.05, 0.8}));
        door_frame_radius_ = declare_parameter("door_frame_radius", 0.05);

        // ── ROS interfaces ───────────────────────────────────────────────
        state_pub_ = create_publisher<std_msgs::msg::String>("~/state", 10);

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                jointStateCallback(msg);
            });

        double tick_rate = declare_parameter("tick_rate", 20.0);
        auto period = std::chrono::duration<double>(1.0 / tick_rate);
        tick_timer_ = create_wall_timer(period, [this]() { tick(); });

        RCLCPP_INFO(get_logger(),
                    "DoorTrajectoryNode started (skeleton). "
                    "r=%.2f L=%.2f h=%.2f theta_step=%.1fdeg tick_rate=%.1f Hz",
                    r_, L_, h_, theta_step_deg_, tick_rate);
    }

    /// @brief Periodic tick — stub.  Will contain the trajectory-generation
    ///        state machine in future waves.
    void tick() {
        auto msg = std_msgs::msg::String();
        msg.data = "IDLE";
        state_pub_->publish(msg);
    }

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
    bool planAndExecuteToPose(const geometry_msgs::msg::Pose& target);

    // ── Public accessors (for testing) ───────────────────────────────────

    double r() const { return r_; }
    double L() const { return L_; }
    double h() const { return h_; }
    const std::vector<double>& T_armBase_doorHinge() const {
        return T_armBase_doorHinge_;
    }
    double theta_step_deg() const { return theta_step_deg_; }
    const std::vector<double>& phi_values_deg() const {
        return phi_values_deg_;
    }
    const std::vector<double>& home_joints() const { return home_joints_; }
    double joint_state_tolerance() const { return joint_state_tolerance_; }
    const std::vector<double>& door_panel_size() const {
        return door_panel_size_;
    }
    double door_frame_radius() const { return door_frame_radius_; }

protected:
    // ── Exposed for test injection ───────────────────────────────────────

    /// @brief Replace the internal MoveGroupInterface (for test mocking).
    void setMoveGroupForTesting(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg) {
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

    // ── Members exposed for test access ──────────────────────────────────

    /// @brief MoveGroupInterface — lazy-initialised on first planAndExecuteToPose().
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::vector<double> current_joints_;
    mutable std::mutex joints_mutex_;

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
    bool waitForCompletion(const std::vector<double>& target_joints,
                           double timeout_sec);

private:
    // ── Parameters ───────────────────────────────────────────────────────
    double r_;
    double L_;
    double h_;
    std::vector<double> T_armBase_doorHinge_;
    double theta_step_deg_;
    std::vector<double> phi_values_deg_;
    std::vector<double> home_joints_;
    double joint_state_tolerance_;
    std::vector<double> door_panel_size_;
    double door_frame_radius_;

    // ── ROS interfaces ───────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr tick_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;
};

}  // namespace omr_controller
