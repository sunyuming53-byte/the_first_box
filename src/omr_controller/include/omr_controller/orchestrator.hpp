#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <cstdint>

#include <chrono>
#include <memory>
#include <string>

#include "omr_controller/clients/arm_client.hpp"
#include "omr_controller/clients/base_client.hpp"
#include "omr_controller/clients/gripper_client.hpp"
#include "omr_controller/clients/motor_client.hpp"
#include "omr_controller/clients/vision_client.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace omr_controller {

class TaskOrchestrator : public rclcpp::Node {
public:
    explicit TaskOrchestrator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    std::unique_ptr<ArmClient> arm_;
    std::unique_ptr<GripperClient> gripper_;
    std::unique_ptr<VisionClient> vision_;
    std::unique_ptr<MotorClient> motor_;
    std::unique_ptr<BaseClientImpl> base_;

private:
    void tick();
    void configure_tick_timer(double tick_rate);
    void load_bt_xml();
    bool load_bt_xml(const std::string& xml_path, std::string* error);
    void setup_parameter_callback();
    void setup_diagnostics();
    void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

    rclcpp::TimerBase::SharedPtr tick_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    diagnostic_updater::Updater diagnostics_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::Node::SharedPtr node_handle_;
    BT::Tree bt_tree_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    double bt_tick_rate_{20.0};
    std::string bt_xml_path_{"pick_and_place.xml"};
    std::string last_tree_status_{"IDLE"};
    std::chrono::steady_clock::time_point last_tick_time_{};
    double last_tick_duration_ms_{0.0};
    uint64_t tick_count_{0};
};

}  // namespace omr_controller
