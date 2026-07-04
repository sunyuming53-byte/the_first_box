#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include <string>

namespace rm {

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

} // namespace rm
