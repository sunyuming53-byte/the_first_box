#pragma once
#include "realman/core/arm.hpp"

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace realman_hardware {

class ArmSystem : public hardware_interface::SystemInterface {
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
    std::unique_ptr<rm::Arm> arm_;
    rm::ArmConfig arm_config_;
    int dof_{6};

    // Parsed from HardwareInfo (joint names, command/state interfaces)
    std::vector<std::string> joint_names_;

    // Actual data storage — read() writes here, write() reads from here
    std::vector<double> hw_position_;
    std::vector<double> hw_velocity_;
    std::vector<double> hw_effort_;
    std::vector<double> hw_position_cmd_;
    std::vector<double> hw_position_cmd_prev_;
};

}  // namespace realman_hardware
