#include "realman/arm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

class ExternalTriggerNode : public rclcpp::Node {
public:
    ExternalTriggerNode()
        : Node("external_trigger"), arm_(rm::ArmConfig{})
    {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/arm_trigger", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    if (msg->data == "home") {
                        rm::JointPosition home(
                            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
                        arm_.moveJ(home, 30);
                        RCLCPP_INFO(this->get_logger(), "Homed");
                    } else if (msg->data == "grip") {
                        arm_.gripper(100, 50);
                        RCLCPP_INFO(this->get_logger(), "Gripped");
                    }
                } catch (const rm::ArmError& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
                }
            });
    }

private:
    rm::Arm arm_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExternalTriggerNode>());
    rclcpp::shutdown();
    return 0;
}
