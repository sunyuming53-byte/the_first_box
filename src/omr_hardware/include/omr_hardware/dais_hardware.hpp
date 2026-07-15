#pragma once
#include <memory>
#include <string>
#include <vector>

#include "dais/motor.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace omr_hardware {

class DaisHardware : public hardware_interface::SystemInterface {
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
    std::unique_ptr<dais::Motor> motor_;
    dais::MotorConfig motor_config_;
    std::string joint_name_;
    double hw_position_state_ = 0.0;
    double hw_velocity_state_ = 0.0;
    double hw_velocity_cmd_ = 0.0;
    double screw_lead_m_ = 0.01;  // meters per revolution (screw lead)
};

}  // namespace omr_hardware
