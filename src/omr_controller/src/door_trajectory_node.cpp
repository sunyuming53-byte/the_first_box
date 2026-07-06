#include "omr_controller/door_trajectory_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<omr_controller::DoorTrajectoryNode>();
    RCLCPP_INFO(node->get_logger(), "Door trajectory node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
