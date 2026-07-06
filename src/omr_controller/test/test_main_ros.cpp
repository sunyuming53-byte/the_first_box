#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    // DO NOT call rclcpp::shutdown
    return result;
}
