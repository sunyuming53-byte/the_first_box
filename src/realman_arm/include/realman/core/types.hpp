#pragma once
#include <cstdint>
#include <string>

namespace rm {

enum class ArmModel {
    RM_65,
    RM_75,
    ECO65,
    ECO63,
    RML_63,
    RML_63_III,
    GEN_72,
    GEN_72_II,
};

struct ArmConfig {
    std::string ip{"192.168.1.18"};   // arm controller IP
    int         tcp_port{8080};        // arm API port (default 8080)
    ArmModel    model{ArmModel::RM_65}; // arm model (affects DOF and kinematics)
    int         dof{6};                // degrees of freedom (6 for 6-axis, 7 for 7-axis)

    // ── UDP realtime push (arm → host) ──
    std::string udp_ip{"192.168.1.10"}; // host IP to receive UDP push data
    int         udp_port{8089};          // host UDP port
    int         udp_cycle{5};            // push interval in milliseconds
    int         udp_force_coordinate{0}; // force-data coordinate frame: 0=base, 1=tool
    bool        udp_joint_speed{true};   // push joint speed
    bool        udp_arm_current_status{false}; // push arm current/voltage status
    bool        udp_lift_state{false};   // push lift (rail/linear axis) state
    bool        udp_expand_state{false}; // push expansion module state
    bool        udp_hand{false};         // push end-effector gripper state
    bool        udp_aloha{false};        // push ALOHA (bimanual) state

    // ── Trajectory following ──
    int         trajectory_mode{0}; // 0=passthrough, 1=fit, 2=filter
    int         radio{0};           // blending radius percentage (0–100)
};

} // namespace rm
