#include "realman/arm_node.hpp"
#include <std_srvs/srv/trigger.hpp>

namespace rm {

ArmNode::ArmNode(const rclcpp::NodeOptions& options, const ArmConfig& config)
    : rclcpp::Node("arm_node", options), arm_(config)
{
    setupServices();
}

void ArmNode::setupServices() {
    using Trigger = std_srvs::srv::Trigger;

    stop_svc_ = this->create_service<Trigger>(
        "~/stop",
        [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
            try {
                arm_.stop();
                res->success = true;
                res->message = "stopped";
            } catch (const ArmError& e) {
                res->success = false;
                res->message = e.what();
            }
        });
}

} // namespace rm
