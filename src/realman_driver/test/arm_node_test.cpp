#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <thread>

#include "realman/node/arm_node.hpp"

class ArmNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};

TEST_F(ArmNodeTest, ConstructorSmoke) {
    rm::ArmConfig config;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rm::ArmNode>(options, config);

    EXPECT_NE(node, nullptr);
    EXPECT_STREQ(node->get_name(), "arm_node");

    // Verify node is alive via its parameters
    EXPECT_EQ(node->get_parameter("calibration_file").as_string(), "");
    EXPECT_EQ(node->get_parameter("base_frame").as_string(), "base_link");
    EXPECT_EQ(node->get_parameter("camera_frame").as_string(), "camera_link");

    // Verify stop service exists (can be called)
    auto client = node->create_client<std_srvs::srv::Trigger>("arm_node/stop");
    EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(1)))
        << "Stop service should be available";
}

TEST_F(ArmNodeTest, JointStatePublisher) {
    rm::ArmConfig config;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rm::ArmNode>(options, config);

    // Subscribe to the joint state topic to verify the publish path
    bool received = false;
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "arm_node/joint_states", 10,
        [&received](sensor_msgs::msg::JointState::SharedPtr msg) {
            (void)msg;
            received = true;
        });

    // Spin with executor for up to 1s to catch a timer fire (100ms interval)
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);

    auto start = node->now();
    while (rclcpp::ok() && !received &&
           (node->now() - start).seconds() < 1.0) {
        exec->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(received)
        << "JointState message should be published within 1 second";
}

TEST_F(ArmNodeTest, CalibrationFileEmpty) {
    rm::ArmConfig config;
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("calibration_file", "")
    });

    // Should not throw or crash with empty calibration file
    EXPECT_NO_THROW({
        auto node = std::make_shared<rm::ArmNode>(options, config);
        EXPECT_NE(node, nullptr);
    });

    // Node should be alive and operational
    EXPECT_TRUE(rclcpp::ok());
}
