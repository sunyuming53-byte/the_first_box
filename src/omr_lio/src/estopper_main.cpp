#include "omr_lio/estopper_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<omr_lio::EstopperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
