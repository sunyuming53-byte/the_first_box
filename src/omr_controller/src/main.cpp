#include "omr_controller/orchestrator.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<omr_controller::TaskOrchestrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
