#include "realman/arm_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rm::ArmConfig cfg;
    auto node_opts = rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rm::ArmNode>(node_opts, cfg);
    RCLCPP_INFO(node->get_logger(), "ArmNode ready");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
