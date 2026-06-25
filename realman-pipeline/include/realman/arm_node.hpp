#pragma once
#include <rclcpp/rclcpp.hpp>
#include "realman/arm.hpp"

namespace rm {

class ArmNode : public rclcpp::Node {
public:
    explicit ArmNode(const rclcpp::NodeOptions& options, const ArmConfig& config);

    Arm& arm() { return arm_; }

private:
    void setupServices();

    Arm arm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_svc_;
};

} // namespace rm
