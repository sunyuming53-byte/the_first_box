#include "omr_lio/lio_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<omr_lio::LioNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
