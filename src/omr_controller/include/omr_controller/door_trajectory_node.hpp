#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
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

        double tick_rate = declare_parameter("tick_rate", 20.0);
        auto period = std::chrono::duration<double>(1.0 / tick_rate);
        tick_timer_ = create_wall_timer(period, [this]() { tick(); });

        RCLCPP_INFO(get_logger(),
                    "DoorTrajectoryNode started (skeleton). "
                    "r=%.2f L=%.2f h=%.2f θ_step=%.1f° tick_rate=%.1f Hz",
                    r_, L_, h_, theta_step_deg_, tick_rate);
    }

    /// @brief Periodic tick — stub.  Will contain the trajectory-generation
    ///        state machine in future waves.
    void tick() {
        auto msg = std_msgs::msg::String();
        msg.data = "IDLE";
        state_pub_->publish(msg);
    }

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
};

}  // namespace omr_controller
