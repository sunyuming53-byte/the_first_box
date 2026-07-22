// SPDX-License-Identifier: MIT
// PhotogateHardware — ros2_control SystemInterface for photogate sensors via serial
// Follows M65BaseHardware lifecycle pattern exactly.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "photogate/photogate.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace omr_hardware {

class PhotogateHardware : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time& time,
                                         const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time,
                                          const rclcpp::Duration& period) override;

private:
    std::unique_ptr<photogate::Photogate> pg_;
    photogate::PhotogateConfig pg_config_;
    int gate_count_ = 0;

    // Per-gate state interfaces (mapped via joint names)
    std::vector<std::string> joint_names_;
    std::vector<double> hw_blocked_;
    std::vector<double> hw_pulse_count_;
};

}  // namespace omr_hardware
