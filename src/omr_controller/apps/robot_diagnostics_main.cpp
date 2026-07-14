#include <memory>

#include "omr_controller/robot_diagnostics.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<omr_controller::RobotDiagnostics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
