#include "omr_hardware/arm_system.hpp"

#include <cmath>

#include <pluginlib/class_list_macros.hpp>

using hardware_interface::CallbackReturn;

namespace omr_hardware {

CallbackReturn ArmSystem::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // Parse joint names from URDF/ros2_control tag
    joint_names_.clear();
    for (const auto& joint : info.joints) {
        joint_names_.push_back(joint.name);
    }
    dof_ = static_cast<int>(joint_names_.size());
    if (dof_ < 1 || dof_ > 7) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Invalid DOF: %d (expected 1-7)", dof_);
        return CallbackReturn::ERROR;
    }

    // Reorder joint_names_ to match the SDK internal joint order.
    // The SDK (libapi_c) returns joint positions in a fixed hardware-defined
    // order that differs from the URDF.  For RM65 the SDK order is:
    //   joint2, joint3, joint1, joint4, joint5, joint6
    // We reorder here so that read()/write() index i maps directly to
    // SDK index i, and controllers match by joint name.
    if (dof_ == 6) {
        static const std::array<const char*, 6> SDK_ORDER_RM65 = {"joint2", "joint3", "joint1",
                                                                  "joint4", "joint5", "joint6"};
        // Validate that the URDF provides all joints expected by this arm model
        for (const auto& sdk_name : SDK_ORDER_RM65) {
            bool found = false;
            for (const auto& j : info.joints) {
                if (j.name == sdk_name) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"),
                             "URDF missing joint '%s' required by RM65 SDK order. "
                             "URDF joint names must match joint1..joint6",
                             sdk_name);
                return CallbackReturn::ERROR;
            }
        }
        std::vector<std::string> reordered(dof_);
        for (int i = 0; i < dof_; ++i) {
            reordered[i] = SDK_ORDER_RM65[i];
        }
        joint_names_ = reordered;
    }

    // Parse arm config from ros2_control <param> tags in URDF
    arm_config_.ip = info.hardware_parameters.at("arm_ip");
    arm_config_.tcp_port = std::stoi(info.hardware_parameters.at("tcp_port"));
    arm_config_.dof = dof_;

    // Resize state/command vectors
    hw_position_.resize(dof_, 0.0);
    hw_velocity_.resize(dof_, 0.0);
    hw_effort_.resize(dof_, 0.0);
    hw_position_cmd_.resize(dof_, 0.0);
    hw_position_cmd_prev_.resize(dof_, NAN);  // NAN = "no previous command"

    // State polling throttle — SDK TCP commands are expensive.
    // We cache joint_pos_ between reads and only re-poll every POLL_INTERVAL_MS.
    last_poll_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "ArmSystem on_init: ip=%s port=%d dof=%d",
                arm_config_.ip.c_str(), arm_config_.tcp_port, dof_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    try {
        arm_ = std::make_unique<rm::Arm>(arm_config_);
    } catch (const rm::ArmError& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Failed to create rm::Arm: %s", e.what());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "ArmSystem configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    if (!arm_) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Arm not initialized — cannot activate");
        return CallbackReturn::ERROR;
    }
    // Trigger lazy TCP connection to the arm (ensureConnected is called internally
    // on the first command; jointPosition() is the cheapest command available)
    try {
        auto pos = arm_->jointPosition();
        for (int i = 0; i < dof_ && i < static_cast<int>(pos.radians.size()); ++i) {
            hw_position_[i] = pos.radians[i];
            hw_position_cmd_[i] = pos.radians[i];
            hw_position_cmd_prev_[i] = NAN;
        }
    } catch (const rm::ArmError& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmSystem"), "Not connected — cannot activate: %s",
                     e.what());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "ArmSystem activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("ArmSystem"), "ArmSystem deactivated");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(static_cast<std::size_t>(dof_) * 3);
    for (int i = 0; i < dof_; ++i) {
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION,
                                &hw_position_[i]);
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY,
                                &hw_velocity_[i]);
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_effort_[i]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(static_cast<std::size_t>(dof_));
    for (int i = 0; i < dof_; ++i) {
        interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION,
                                &hw_position_cmd_[i]);
    }
    return interfaces;
}

hardware_interface::return_type ArmSystem::read(const rclcpp::Time& /*time*/,
                                                const rclcpp::Duration& /*period*/) {
    if (!arm_) {
        return hardware_interface::return_type::OK;
    }
    if (!arm_->isConnected() && !arm_->ensureConnected()) {
        return hardware_interface::return_type::OK;
    }

    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    if ((now - last_poll_time_).seconds() * 1000.0 < kPollIntervalMs) {
        return hardware_interface::return_type::OK;
    }
    last_poll_time_ = now;

    try {
        auto pos = arm_->jointPosition();
        for (int i = 0; i < dof_ && i < static_cast<int>(pos.radians.size()); ++i) {
            hw_position_[i] = pos.radians[i];
        }
        static int tick = 0;
        if (++tick % 200 == 0) {
            RCLCPP_DEBUG(rclcpp::get_logger("ArmSystem"),
                         "joints: [%.3f %.3f %.3f %.3f %.3f %.3f] rad", hw_position_[0],
                         hw_position_[1], hw_position_[2], hw_position_[3], hw_position_[4],
                         hw_position_[5]);
        }
        // Velocity and effort are not provided by the current SDK — leave as 0
    } catch (const rm::ArmError& e) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ArmSystem"), steady_clock, 5000,
                              "read() failed: %s", e.what());
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystem::write(const rclcpp::Time& /*time*/,
                                                 const rclcpp::Duration& /*period*/) {
    if (!arm_ || !arm_->isConnected()) {
        return hardware_interface::return_type::OK;
    }

    // Check if command changed since last cycle (skip redundant sends)
    bool changed = false;
    for (int i = 0; i < dof_; ++i) {
        if (std::isnan(hw_position_cmd_prev_[i]) ||
            std::abs(hw_position_cmd_[i] - hw_position_cmd_prev_[i]) > 1e-6) {
            changed = true;
            break;
        }
    }
    if (!changed) {
        return hardware_interface::return_type::OK;
    }

    try {
        auto target =
            std::vector<double>(hw_position_cmd_.begin(), hw_position_cmd_.begin() + dof_);
        arm_->moveJ(rm::JointPosition(std::move(target)), 100, false);
        hw_position_cmd_prev_ = hw_position_cmd_;
    } catch (const rm::ArmError& e) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ArmSystem"), steady_clock, 5000,
                              "write() failed: %s", e.what());
    }

    return hardware_interface::return_type::OK;
}

}  // namespace omr_hardware

PLUGINLIB_EXPORT_CLASS(omr_hardware::ArmSystem, hardware_interface::SystemInterface)
