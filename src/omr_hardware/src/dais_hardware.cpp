#include "omr_hardware/dais_hardware.hpp"

#include <cmath>

#include <string>

#include <pluginlib/class_list_macros.hpp>

using hardware_interface::CallbackReturn;

namespace omr_hardware {

CallbackReturn DaisHardware::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Parse single joint name from URDF/ros2_control tag
    if (info.joints.size() != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("DaisHardware"), "Expected exactly 1 joint, got %zu",
                     info.joints.size());
        return CallbackReturn::ERROR;
    }
    joint_name_ = info.joints[0].name;

    // Parse motor config from ros2_control <param> tags in URDF
    motor_config_.serial_port = info.hardware_parameters.at("serial_port");
    motor_config_.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
    motor_config_.slave_id = std::stoi(info.hardware_parameters.at("slave_id"));
    motor_config_.gear_ratio_denom = std::stoi(info.hardware_parameters.at("gear_ratio"));
    screw_lead_m_ = std::stod(info.hardware_parameters.at("screw_lead"));

    if (screw_lead_m_ <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("DaisHardware"), "Invalid screw_lead: %.4f (must be > 0)",
                     screw_lead_m_);
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DaisHardware"),
                "DaisHardware on_init: port=%s baud=%d slave=%d gear=%d lead=%.4f joint=%s",
                motor_config_.serial_port.c_str(), motor_config_.baud_rate, motor_config_.slave_id,
                motor_config_.gear_ratio_denom, screw_lead_m_, joint_name_.c_str());

    return CallbackReturn::SUCCESS;
}

CallbackReturn DaisHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    motor_ = std::make_unique<dais::Motor>(motor_config_);
    if (!motor_->connect()) {
        RCLCPP_ERROR(rclcpp::get_logger("DaisHardware"), "Failed to connect to motor at %s",
                     motor_config_.serial_port.c_str());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("DaisHardware"), "DaisHardware configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DaisHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!motor_ || !motor_->is_connected()) {
        RCLCPP_ERROR(rclcpp::get_logger("DaisHardware"), "Not connected — cannot activate");
        return CallbackReturn::ERROR;
    }

    if (!motor_->configure_device()) {
        RCLCPP_ERROR(rclcpp::get_logger("DaisHardware"), "Failed to configure motor device");
        return CallbackReturn::ERROR;
    }

    if (!motor_->enable()) {
        RCLCPP_ERROR(rclcpp::get_logger("DaisHardware"), "Failed to enable motor");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DaisHardware"), "DaisHardware activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn DaisHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (motor_) {
        motor_->disable();
        motor_->disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("DaisHardware"), "DaisHardware deactivated");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DaisHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_state_);
    interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_state_);
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> DaisHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_cmd_);
    return interfaces;
}

hardware_interface::return_type DaisHardware::read(const rclcpp::Time& /*time*/,
                                                   const rclcpp::Duration& /*period*/) {
    if (!motor_ || !motor_->is_connected()) {
        return hardware_interface::return_type::OK;
    }

    dais::MotorState s = motor_->read_state();
    hw_position_state_ = s.position_rad * screw_lead_m_ / (2.0 * M_PI);
    hw_velocity_state_ = s.velocity_rpm * screw_lead_m_ / 60.0;  // rpm → m/s (via screw lead)

    if (s.comm_error || s.fault_code != 0) {
        static rclcpp::Clock clock(RCL_ROS_TIME);
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("DaisHardware"), clock, 2000,
                              "Dais motor error: fault=0x%04X comm_error=%d", s.fault_code,
                              static_cast<int>(s.comm_error));
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DaisHardware::write(const rclcpp::Time& /*time*/,
                                                    const rclcpp::Duration& /*period*/) {
    if (!motor_ || !motor_->is_connected()) {
        return hardware_interface::return_type::OK;
    }

    // Drive fault latches inside dais::Motor (zeros cmd + disables servo).
    if (motor_->has_error()) {
        return hardware_interface::return_type::OK;
    }

    motor_->set_velocity_command(hw_velocity_cmd_ * (2.0 * M_PI) /
                                 screw_lead_m_);  // m/s → rad/s (via screw lead)

    return hardware_interface::return_type::OK;
}

}  // namespace omr_hardware

PLUGINLIB_EXPORT_CLASS(omr_hardware::DaisHardware, hardware_interface::SystemInterface)
