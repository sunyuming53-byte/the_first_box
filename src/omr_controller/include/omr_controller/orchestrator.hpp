#pragma once

#include "omr_controller/clients/arm_client.hpp"
#include "omr_controller/clients/base_client.hpp"
#include "omr_controller/clients/gripper_client.hpp"
#include "omr_controller/clients/motor_client.hpp"
#include "omr_controller/clients/vision_client.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

namespace omr_controller {

class TaskOrchestrator : public rclcpp::Node {
public:
    explicit TaskOrchestrator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void tick();
    void load_bt_xml();

    friend class ::OrchestratorIntegrationTest;

    std::unique_ptr<ArmClient> arm_;
    std::unique_ptr<GripperClient> gripper_;
    std::unique_ptr<VisionClient> vision_;
    std::unique_ptr<MotorClientStub> motor_;
    std::unique_ptr<BaseClientStub> base_;

    rclcpp::TimerBase::SharedPtr tick_timer_;
    BT::Tree bt_tree_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

}  // namespace omr_controller
