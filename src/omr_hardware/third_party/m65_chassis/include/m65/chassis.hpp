// SPDX-License-Identifier: MIT
// m65::Chassis — M65 chassis serial driver (zero ROS deps)
// Follows dais::Motor PIMPL facade pattern.
#pragma once

#include <memory>

#include "m65/types.hpp"

namespace m65 {

class Chassis {
public:
    explicit Chassis(const ChassisConfig& config);
    ~Chassis();

    // Non-copyable, non-movable (owns serial resources)
    Chassis(const Chassis&)            = delete;
    Chassis& operator=(const Chassis&) = delete;
    Chassis(Chassis&&)                 = delete;
    Chassis& operator=(Chassis&&)      = delete;

    // ── Connection ──
    bool connect();
    void disconnect();
    [[nodiscard]] bool is_connected() const;

    // ── Motion control ──
    void set_velocity(double linear_x, double angular_z);

    // ── State ──
    ChassisState read_state();
    [[nodiscard]] ChassisState last_state() const;

    // ── Status ──
    [[nodiscard]] bool is_emergency() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace m65
