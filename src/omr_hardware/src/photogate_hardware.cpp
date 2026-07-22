// SPDX-License-Identifier: MIT
// PhotogateHardware — ros2_control SystemInterface for photogate sensors via serial
// Follows M65BaseHardware lifecycle pattern exactly.
#include "omr_hardware/photogate_hardware.hpp"

#include <pluginlib/class_list_macros.hpp>

using hardware_interface::CallbackReturn;

namespace omr_hardware {

CallbackReturn PhotogateHardware::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Gate count is determined by how many joints are declared in URDF.
    // Each joint: photogate_gate{N}_joint with state interfaces "blocked" and "pulse_count".
    gate_count_ = static_cast<int>(info.joints.size());
    if (gate_count_ == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("PhotogateHardware"),
                     "No joints declared — at least one photogate joint is required");
        return CallbackReturn::ERROR;
    }

    joint_names_.reserve(gate_count_);
    for (int i = 0; i < gate_count_; ++i) {
        joint_names_.push_back(info.joints[i].name);
    }

    // Parse hardware params from URDF <param> tags
    pg_config_.serial_port = info.hardware_parameters.at("serial_port");
    pg_config_.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
    pg_config_.gate_count = gate_count_;

    RCLCPP_INFO(rclcpp::get_logger("PhotogateHardware"),
                "PhotogateHardware on_init: port=%s baud=%d gates=%d",
                pg_config_.serial_port.c_str(), pg_config_.baud_rate, gate_count_);

    hw_blocked_.resize(gate_count_, 0.0);
    hw_pulse_count_.resize(gate_count_, 0.0);

    return CallbackReturn::SUCCESS;
}

CallbackReturn PhotogateHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    pg_ = std::make_unique<photogate::Photogate>(pg_config_);
    if (!pg_->connect()) {
        RCLCPP_ERROR(rclcpp::get_logger("PhotogateHardware"),
                     "Failed to connect to photogate at %s", pg_config_.serial_port.c_str());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("PhotogateHardware"), "PhotogateHardware configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhotogateHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!pg_ || !pg_->is_connected()) {
        RCLCPP_ERROR(rclcpp::get_logger("PhotogateHardware"),
                     "Not connected — cannot activate");
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("PhotogateHardware"), "PhotogateHardware activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn PhotogateHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (pg_) {
        pg_->disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("PhotogateHardware"), "PhotogateHardware deactivated");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PhotogateHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (int i = 0; i < gate_count_; ++i) {
        interfaces.emplace_back(joint_names_[i], "blocked", &hw_blocked_[i]);
        interfaces.emplace_back(joint_names_[i], "pulse_count", &hw_pulse_count_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> PhotogateHardware::export_command_interfaces() {
    return {};  // Sensors are read-only — no command interfaces
}

hardware_interface::return_type PhotogateHardware::read(const rclcpp::Time& /*time*/,
                                                        const rclcpp::Duration& /*period*/) {
    if (!pg_ || !pg_->is_connected()) {
        return hardware_interface::return_type::OK;
    }

    pg_->read_frames();

    for (int i = 0; i < gate_count_; ++i) {
        auto s = pg_->gate_state(i);
        hw_blocked_[i] = s.blocked ? 1.0 : 0.0;
        hw_pulse_count_[i] = static_cast<double>(s.pulse_count);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PhotogateHardware::write(const rclcpp::Time& /*time*/,
                                                         const rclcpp::Duration& /*period*/) {
    return hardware_interface::return_type::OK;  // Read-only sensor
}

}  // namespace omr_hardware

PLUGINLIB_EXPORT_CLASS(omr_hardware::PhotogateHardware, hardware_interface::SystemInterface)
