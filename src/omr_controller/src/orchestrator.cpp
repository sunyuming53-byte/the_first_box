#include "omr_controller/orchestrator.hpp"
#include "omr_controller/state_machine/bt_factory.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <omr_vision/camera/types.hpp>

#include <fstream>
#include <sstream>
#include <string>

namespace omr_controller {

TaskOrchestrator::TaskOrchestrator(const rclcpp::NodeOptions& options)
    : Node("task_orchestrator", options) {
    auto self = std::shared_ptr<rclcpp::Node>(
        static_cast<rclcpp::Node*>(this), [](rclcpp::Node*) {});
    arm_ = std::make_unique<ArmClient>(self);
    gripper_ = std::make_unique<GripperClient>(self);
    vision_ = std::make_unique<VisionClient>(
        std::make_unique<CameraStreamAdapter>(omr_vision::camera::CameraConfig{}));
    motor_ = std::make_unique<MotorClientStub>();
    base_ = std::make_unique<BaseClientStub>();

    // State publisher
    state_pub_ = create_publisher<std_msgs::msg::String>("/task_state", 10);

    // Build BT tree from XML
    load_bt_xml();

    // Tick timer
    double tick_rate = declare_parameter("bt_tick_rate", 20.0);
    auto period = std::chrono::duration<double>(1.0 / tick_rate);
    tick_timer_ = create_wall_timer(period, [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "TaskOrchestrator started, tick rate: %.1f Hz", tick_rate);
}

void TaskOrchestrator::tick() {
    bt_tree_.tickOnce();

    auto msg = std_msgs::msg::String();
    msg.data = "RUNNING";
    state_pub_->publish(msg);
}

void TaskOrchestrator::load_bt_xml() {
    std::string xml_path = declare_parameter("bt_xml_path", "pick_and_place.xml");
    std::string share_dir = ament_index_cpp::get_package_share_directory("omr_controller");
    std::string full_path = share_dir + "/bt_xml/" + xml_path;

    std::ifstream file(full_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(),
                     "Failed to open BT XML: %s — tree will be empty",
                     full_path.c_str());
        return;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string xml_content = buffer.str();

    bt_tree_ = build_tree(xml_content, *arm_, *gripper_, *vision_);
    RCLCPP_INFO(get_logger(), "Loaded BT XML: %s", full_path.c_str());
}

}  // namespace omr_controller
