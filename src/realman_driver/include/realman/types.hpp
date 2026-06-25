#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <array>

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

struct JointPosition {
    std::vector<double> radians;
    explicit JointPosition(std::vector<double> r) : radians(std::move(r)) {}
    JointPosition() = default;
};

struct CartesianPose {
    double x{0}, y{0}, z{0};           // meters
    double roll{0}, pitch{0}, yaw{0};  // radians (Euler RPY)
};

using SpeedRatio = uint8_t;  // 0–100

struct ArmState {
    JointPosition                 joint_position;
    CartesianPose                 tool_pose;
    std::array<double, 6>         joint_current{};
    std::array<double, 6>         joint_temperature{};
    bool                          is_moving{false};
    int                           error_code{0};
    std::string                   error_message;
};

struct GripperState {
    int enable_state{0};   // 0=disabled, 1=enabled
    int status{0};         // 0=offline, 1=online
    int error{0};          // error bitfield (bit0: stall, bit1: over-temp, bit2: over-current, bit3: driver, bit4: internal)
    int mode{0};           // 1=open-idle, 2=closed-idle, 3=stopped-idle, 4=closing, 5=opening, 6=force-stop
    int current_force{0};  // current gripping force, grams
    int temperature{0};    // degrees Celsius
    int actpos{0};         // current opening position (0–1000, dimensionless)
};

struct ArmConfig {
    std::string ip{"192.168.1.18"};
    int         tcp_port{8080};
    ArmModel    model{ArmModel::RM_65};
    int         dof{6};

    std::string udp_ip{"192.168.1.10"};
    int         udp_port{8089};
    int         udp_cycle{5};
    int         udp_force_coordinate{0};
    bool        udp_joint_speed{true};
    bool        udp_arm_current_status{false};
    bool        udp_lift_state{false};
    bool        udp_expand_state{false};
    bool        udp_hand{false};
    bool        udp_aloha{false};

    int         trajectory_mode{0};
    int         radio{0};
};

} // namespace rm
