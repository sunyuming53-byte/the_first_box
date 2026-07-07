#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "m65/chassis.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace omr_hardware {

class M65BaseHardware : public hardware_interface::SystemInterface {
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
    std::unique_ptr<m65::Chassis> chassis_;
    m65::ChassisConfig chassis_config_;
    std::string left_wheel_name_;
    std::string right_wheel_name_;

    double hw_left_position_state_  = 0.0;
    double hw_left_velocity_state_  = 0.0;
    double hw_right_position_state_ = 0.0;
    double hw_right_velocity_state_ = 0.0;

    double hw_left_velocity_cmd_  = 0.0;
    double hw_right_velocity_cmd_ = 0.0;

    int32_t prev_left_encoder_  = 0;
    int32_t prev_right_encoder_ = 0;
    bool first_read_            = true;
};

}  // namespace omr_hardware
