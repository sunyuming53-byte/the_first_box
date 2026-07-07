#include "omr_hardware/m65_hardware.hpp"

#include <cmath>

#include <string>

#include <pluginlib/class_list_macros.hpp>

using hardware_interface::CallbackReturn;

namespace omr_hardware {

CallbackReturn M65BaseHardware::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Validate exactly 2 joints (differential drive: left + right wheel)
    if (info.joints.size() != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("M65BaseHardware"), "Expected exactly 2 joints, got %zu",
                     info.joints.size());
        return CallbackReturn::ERROR;
    }
    left_wheel_name_ = info.joints[0].name;
    right_wheel_name_ = info.joints[1].name;

    // Parse chassis config from ros2_control <param> tags in URDF
    chassis_config_.serial_port = info.hardware_parameters.at("serial_port");
    chassis_config_.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
    chassis_config_.wheel_separation = std::stod(info.hardware_parameters.at("wheel_separation"));
    chassis_config_.wheel_radius = std::stod(info.hardware_parameters.at("wheel_radius"));
    chassis_config_.encoder_cpr = std::stoi(info.hardware_parameters.at("encoder_cpr"));

    RCLCPP_INFO(rclcpp::get_logger("M65BaseHardware"),
                "M65BaseHardware on_init: port=%s baud=%d separation=%.4f radius=%.4f cpr=%d "
                "left=%s right=%s",
                chassis_config_.serial_port.c_str(), chassis_config_.baud_rate,
                chassis_config_.wheel_separation, chassis_config_.wheel_radius,
                chassis_config_.encoder_cpr, left_wheel_name_.c_str(), right_wheel_name_.c_str());

    return CallbackReturn::SUCCESS;
}

CallbackReturn M65BaseHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    chassis_ = std::make_unique<m65::Chassis>(chassis_config_);
    if (!chassis_->connect()) {
        RCLCPP_ERROR(rclcpp::get_logger("M65BaseHardware"), "Failed to connect to chassis at %s",
                     chassis_config_.serial_port.c_str());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("M65BaseHardware"), "M65BaseHardware configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn M65BaseHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!chassis_ || !chassis_->is_connected()) {
        RCLCPP_ERROR(rclcpp::get_logger("M65BaseHardware"), "Not connected — cannot activate");
        return CallbackReturn::ERROR;
    }

    if (chassis_config_.encoder_cpr <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("M65BaseHardware"),
                     "encoder_cpr must be > 0, got %d — cannot activate",
                     chassis_config_.encoder_cpr);
        return CallbackReturn::ERROR;
    }

    first_read_ = true;
    RCLCPP_INFO(rclcpp::get_logger("M65BaseHardware"), "M65BaseHardware activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn M65BaseHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (chassis_) {
        chassis_->set_velocity(0.0, 0.0);
        chassis_->disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("M65BaseHardware"), "M65BaseHardware deactivated");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> M65BaseHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.emplace_back(left_wheel_name_, hardware_interface::HW_IF_POSITION,
                            &hw_left_position_state_);
    interfaces.emplace_back(left_wheel_name_, hardware_interface::HW_IF_VELOCITY,
                            &hw_left_velocity_state_);
    interfaces.emplace_back(right_wheel_name_, hardware_interface::HW_IF_POSITION,
                            &hw_right_position_state_);
    interfaces.emplace_back(right_wheel_name_, hardware_interface::HW_IF_VELOCITY,
                            &hw_right_velocity_state_);
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> M65BaseHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.emplace_back(left_wheel_name_, hardware_interface::HW_IF_VELOCITY,
                            &hw_left_velocity_cmd_);
    interfaces.emplace_back(right_wheel_name_, hardware_interface::HW_IF_VELOCITY,
                            &hw_right_velocity_cmd_);
    return interfaces;
}

hardware_interface::return_type M65BaseHardware::read(const rclcpp::Time& /*time*/,
                                                      const rclcpp::Duration& period) {
    if (!chassis_ || !chassis_->is_connected()) {
        return hardware_interface::return_type::OK;
    }

    m65::ChassisState s = chassis_->read_state();

    // First read: seed previous encoder values, output zero velocity
    if (first_read_) {
        prev_left_encoder_ = s.left_encoder;
        prev_right_encoder_ = s.right_encoder;
        hw_left_velocity_state_ = 0.0;
        hw_right_velocity_state_ = 0.0;
        first_read_ = false;
        return hardware_interface::return_type::OK;
    }

    // Compute encoder delta → position increment (radians)
    double left_delta =
        (s.left_encoder - prev_left_encoder_) * 2.0 * M_PI / chassis_config_.encoder_cpr;
    double right_delta =
        (s.right_encoder - prev_right_encoder_) * 2.0 * M_PI / chassis_config_.encoder_cpr;

    hw_left_position_state_ += left_delta;
    hw_right_position_state_ += right_delta;

    prev_left_encoder_ = s.left_encoder;
    prev_right_encoder_ = s.right_encoder;

    // Velocity from encoder delta / period
    double dt = period.seconds();
    if (dt > 0.0) {
        hw_left_velocity_state_ = left_delta / dt;
        hw_right_velocity_state_ = right_delta / dt;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type M65BaseHardware::write(const rclcpp::Time& /*time*/,
                                                       const rclcpp::Duration& /*period*/) {
    if (!chassis_ || !chassis_->is_connected()) {
        return hardware_interface::return_type::OK;
    }

    // diff_drive_controller outputs per-wheel velocity in rad/s.
    // Convert to unicycle model: linear + angular.
    double vl = hw_left_velocity_cmd_;
    double vr = hw_right_velocity_cmd_;

    double linear = (vl + vr) * chassis_config_.wheel_radius / 2.0;
    double angular = 0.0;
    if (chassis_config_.wheel_separation > 0.0) {
        angular = (vr - vl) * chassis_config_.wheel_radius / chassis_config_.wheel_separation;
    }

    chassis_->set_velocity(linear, angular);

    return hardware_interface::return_type::OK;
}

}  // namespace omr_hardware

PLUGINLIB_EXPORT_CLASS(omr_hardware::M65BaseHardware, hardware_interface::SystemInterface)
