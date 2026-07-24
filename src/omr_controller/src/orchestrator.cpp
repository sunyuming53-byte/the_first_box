#include "omr_controller/orchestrator.hpp"

#include <exception>

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "omr_controller/clients/topic_camera_adapter.hpp"
#include "omr_controller/state_machine/bt_factory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace omr_controller {

TaskOrchestrator::TaskOrchestrator(const rclcpp::NodeOptions& options)
    : Node("task_orchestrator", options), diagnostics_(this) {
    node_handle_ =
        std::shared_ptr<rclcpp::Node>(static_cast<rclcpp::Node*>(this), [](rclcpp::Node*) {});
    arm_ = std::make_unique<ArmClient>(node_handle_);
    gripper_ = std::make_unique<GripperClient>(node_handle_);
    vision_ = std::make_unique<VisionClient>(std::make_unique<TopicCameraAdapter>(node_handle_));

    const auto motorJtcAction = declare_parameter<std::string>(
        "motor_jtc_action", "/dais_joint_trajectory_controller/follow_joint_trajectory");
    const auto motorJointStateTopic =
        declare_parameter<std::string>("motor_joint_state_topic", "/joint_states");
    const auto motorJointName = declare_parameter<std::string>("motor_joint_name", "joint_dais");
    motor_ = std::make_unique<MotorClientImpl>(node_handle_, motorJtcAction, motorJointStateTopic,
                                               motorJointName);
    base_ = std::make_unique<BaseClientImpl>(node_handle_);

    // Guideway (rail) client: direct-serial, lazy connection — no port is
    // opened until the first guideway command. Must not run alongside the
    // DaisHardware/PhotogateHardware ros2_control plugins (same serial ports).
    GuidewayConfig gwCfg;
    gwCfg.photogate_port =
        declare_parameter<std::string>("guideway_photogate_port", gwCfg.photogate_port);
    gwCfg.photogate_baud = declare_parameter<int>("guideway_photogate_baud", gwCfg.photogate_baud);
    gwCfg.gate_count = declare_parameter<int>("guideway_gate_count", gwCfg.gate_count);
    gwCfg.lower_gate = declare_parameter<int>("guideway_lower_gate", gwCfg.lower_gate);
    gwCfg.home_gate = declare_parameter<int>("guideway_home_gate", gwCfg.home_gate);
    gwCfg.upper_gate = declare_parameter<int>("guideway_upper_gate", gwCfg.upper_gate);
    gwCfg.motor_port = declare_parameter<std::string>("guideway_motor_port", gwCfg.motor_port);
    gwCfg.motor_baud = declare_parameter<int>("guideway_motor_baud", gwCfg.motor_baud);
    gwCfg.slave_id = declare_parameter<int>("guideway_slave_id", gwCfg.slave_id);
    gwCfg.up_sign = declare_parameter<int>("guideway_up_sign", gwCfg.up_sign);
    gwCfg.screw_lead_m = declare_parameter<double>("guideway_screw_lead_m", gwCfg.screw_lead_m);
    gwCfg.watchdog_ms = declare_parameter<double>("guideway_watchdog_ms", gwCfg.watchdog_ms);
    gwCfg.coarse_rpm = declare_parameter<double>("guideway_coarse_rpm", gwCfg.coarse_rpm);
    gwCfg.fine_rpm = declare_parameter<double>("guideway_fine_rpm", gwCfg.fine_rpm);
    gwCfg.backoff_rpm = declare_parameter<double>("guideway_backoff_rpm", gwCfg.backoff_rpm);
    gwCfg.seek_timeout_s =
        declare_parameter<double>("guideway_seek_timeout_s", gwCfg.seek_timeout_s);
    gwCfg.home_tol_rad = declare_parameter<double>("guideway_home_tol_rad", gwCfg.home_tol_rad);
    gwCfg.return_timeout_s =
        declare_parameter<double>("guideway_return_timeout_s", gwCfg.return_timeout_s);
    gwCfg.position_max_rpm = static_cast<uint16_t>(
        declare_parameter<int>("guideway_position_max_rpm", gwCfg.position_max_rpm));
    gwCfg.position_accel_ms = static_cast<uint16_t>(
        declare_parameter<int>("guideway_position_accel_ms", gwCfg.position_accel_ms));
    gwCfg.soft_limit_margin_m =
        declare_parameter<double>("guideway_soft_limit_margin_m", gwCfg.soft_limit_margin_m);
    guideway_ = std::make_unique<GuidewayClientImpl>(gwCfg, get_logger());

    // State publisher
    state_pub_ = create_publisher<std_msgs::msg::String>("/task_state", 10);

    bt_tick_rate_ = declare_parameter("bt_tick_rate", 20.0);
    bt_xml_path_ = declare_parameter("bt_xml_path", "pick_and_place.xml");
    setup_parameter_callback();
    setup_diagnostics();

    // Build BT tree from XML
    load_bt_xml();

    // Tick timer
    configure_tick_timer(bt_tick_rate_);

    RCLCPP_INFO(get_logger(), "TaskOrchestrator started, tick rate: %.1f Hz", bt_tick_rate_);
}

void TaskOrchestrator::tick() {
    if (bt_tree_.rootNode() == nullptr) {
        auto msg = std_msgs::msg::String();
        msg.data = "ERROR";
        last_tree_status_ = "BT_XML_NOT_LOADED";
        state_pub_->publish(msg);
        return;
    }

    auto start = std::chrono::steady_clock::now();
    const auto status = bt_tree_.tickOnce();
    auto end = std::chrono::steady_clock::now();

    last_tick_duration_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
    last_tick_time_ = end;
    tick_count_++;

    auto msg = std_msgs::msg::String();
    msg.data = BT::toStr(status);
    last_tree_status_ = msg.data;
    state_pub_->publish(msg);
}

void TaskOrchestrator::load_bt_xml() {
    std::string error;
    if (!load_bt_xml(bt_xml_path_, &error)) {
        RCLCPP_ERROR(get_logger(), "Failed to load initial BT XML: %s", error.c_str());
    }
}

bool TaskOrchestrator::load_bt_xml(const std::string& xml_path, std::string* error) {
    std::string share_dir = ament_index_cpp::get_package_share_directory("omr_controller");
    std::string full_path = share_dir + "/bt_xml/" + xml_path;

    std::ifstream file(full_path);
    if (!file.is_open()) {
        const std::string message = "failed to open BT XML: " + full_path;
        if (error != nullptr) {
            *error = message;
        }
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string xml_content = buffer.str();

    try {
        auto new_tree = build_tree(xml_content, *arm_, *gripper_, *vision_, node_handle_);
        bt_tree_ = std::move(new_tree);
    } catch (const std::exception& ex) {
        const std::string message = "failed to build BT XML " + full_path + ": " + ex.what();
        if (error != nullptr) {
            *error = message;
        }
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(), "Loaded BT XML: %s", full_path.c_str());
    return true;
}

void TaskOrchestrator::configure_tick_timer(double tick_rate) {
    if (tick_rate <= 0.0) {
        RCLCPP_WARN(get_logger(), "Invalid bt_tick_rate %.3f, keeping %.3f", tick_rate,
                    bt_tick_rate_);
        return;
    }

    bt_tick_rate_ = tick_rate;
    auto period = std::chrono::duration<double>(1.0 / bt_tick_rate_);
    tick_timer_ = create_wall_timer(period, [this]() { tick(); });
}

void TaskOrchestrator::setup_parameter_callback() {
    parameter_callback_handle_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            double next_tick_rate = bt_tick_rate_;
            std::string next_bt_xml_path = bt_xml_path_;
            bool update_tick_rate = false;
            bool update_bt_xml = false;

            for (const auto& param : params) {
                const auto& name = param.get_name();
                if (name == "bt_tick_rate") {
                    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                        param.as_double() <= 0.0) {
                        result.successful = false;
                        result.reason = "bt_tick_rate must be a positive double";
                        return result;
                    }
                    next_tick_rate = param.as_double();
                    update_tick_rate = true;
                } else if (name == "bt_xml_path") {
                    if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
                        param.as_string().empty()) {
                        result.successful = false;
                        result.reason = "bt_xml_path must be a non-empty string";
                        return result;
                    }
                    next_bt_xml_path = param.as_string();
                    update_bt_xml = true;
                }
            }

            if (update_bt_xml) {
                std::string error;
                if (!load_bt_xml(next_bt_xml_path, &error)) {
                    result.successful = false;
                    result.reason = error;
                    return result;
                }
                bt_xml_path_ = next_bt_xml_path;
            }

            if (update_tick_rate) {
                configure_tick_timer(next_tick_rate);
                RCLCPP_INFO(get_logger(), "Updated bt_tick_rate to %.3f", bt_tick_rate_);
            }

            return result;
        });
}

void TaskOrchestrator::setup_diagnostics() {
    diagnostics_.setHardwareID("task_orchestrator");
    diagnostics_.add("behavior_tree", this, &TaskOrchestrator::produce_diagnostics);
    diagnostics_timer_ =
        create_wall_timer(std::chrono::seconds(1), [this]() { diagnostics_.force_update(); });
}

void TaskOrchestrator::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const bool tick_seen = tick_count_ > 0;
    const double last_tick_age_sec =
        tick_seen
            ? std::chrono::duration<double>(std::chrono::steady_clock::now() - last_tick_time_)
                  .count()
            : -1.0;

    if (bt_tree_.rootNode() == nullptr) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "BT XML not loaded");
    } else if (!tick_seen) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "waiting for first BT tick");
    } else if (last_tick_duration_ms_ > 1000.0 / bt_tick_rate_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "BT tick overran period");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "BT ticking");
    }

    stat.add("bt_xml_path", bt_xml_path_);
    stat.add("bt_tick_rate_hz", bt_tick_rate_);
    stat.add("last_tree_status", last_tree_status_);
    stat.add("tick_count", tick_count_);
    stat.add("last_tick_duration_ms", last_tick_duration_ms_);
    stat.add("last_tick_age_sec", last_tick_age_sec);
}

}  // namespace omr_controller
