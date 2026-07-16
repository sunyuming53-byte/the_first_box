#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "apps/remote_control_node.hpp"
#include "omr_controller/srv/move_arm.hpp"
#include "omr_controller/srv/move_rail.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace {

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// RemoteControlTest — spins a test node with service servers and a topic
// subscriber to validate the remote_control communication API.
// ---------------------------------------------------------------------------
class RemoteControlTest : public ::testing::Test {
protected:
    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("remote_control_test");

        // Service server: ~/move_arm
        move_arm_srv_ = node_->create_service<omr_controller::srv::MoveArm>(
            "~/move_arm", [this](const std::shared_ptr<omr_controller::srv::MoveArm::Request> req,
                                 std::shared_ptr<omr_controller::srv::MoveArm::Response> resp) {
                last_move_arm_positions_ = req->positions;
                last_move_arm_time_ = req->time_from_start;
                move_arm_called_.store(true);
                resp->success = true;
                resp->message = "arm moved";
            });

        // Service server: ~/move_rail
        move_rail_srv_ = node_->create_service<omr_controller::srv::MoveRail>(
            "~/move_rail", [this](const std::shared_ptr<omr_controller::srv::MoveRail::Request> req,
                                  std::shared_ptr<omr_controller::srv::MoveRail::Response> resp) {
                last_move_rail_position_ = req->position;
                move_rail_called_.store(true);
                resp->success = true;
                resp->message = "rail moved";
            });

        // Service server: ~/set_gripper (uses std_srvs/SetBool, NOT a custom srv)
        set_gripper_srv_ = node_->create_service<std_srvs::srv::SetBool>(
            "~/set_gripper", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
                last_gripper_data_ = req->data;
                set_gripper_called_.store(true);
                resp->success = true;
                resp->message = "gripper set";
            });

        // Topic subscriber: cmd_vel
        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                last_twist_ = *msg;
                cmd_vel_received_.store(true);
            });

        // Spin once to allow services/subscriptions to register in the ROS graph
        rclcpp::spin_some(node_);
    }

    void TearDown() override {
        cmd_vel_sub_.reset();
        set_gripper_srv_.reset();
        move_rail_srv_.reset();
        move_arm_srv_.reset();
        node_.reset();
    }

    // Helper: spin and wait for an atomic flag with timeout
    bool waitFor(const std::atomic<bool>& flag, std::chrono::milliseconds timeout = 3s) {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            rclcpp::spin_some(node_);
            if (flag.load()) return true;
            std::this_thread::sleep_for(10ms);
        }
        return flag.load();
    }

    rclcpp::Node::SharedPtr node_;

    rclcpp::Service<omr_controller::srv::MoveArm>::SharedPtr move_arm_srv_;
    rclcpp::Service<omr_controller::srv::MoveRail>::SharedPtr move_rail_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_gripper_srv_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    std::atomic<bool> move_arm_called_{false};
    std::atomic<bool> move_rail_called_{false};
    std::atomic<bool> set_gripper_called_{false};
    std::atomic<bool> cmd_vel_received_{false};

    // Captured request data
    std::array<double, 6> last_move_arm_positions_{};
    double last_move_arm_time_{0.0};
    double last_move_rail_position_{0.0};
    bool last_gripper_data_{false};
    geometry_msgs::msg::Twist last_twist_;
};

// ---------------------------------------------------------------------------
// Test case 1: Call MoveArm service with 6 joint positions and time.
// ---------------------------------------------------------------------------
TEST_F(RemoteControlTest, move_arm_service_call) {
    auto client = node_->create_client<omr_controller::srv::MoveArm>("~/move_arm");
    ASSERT_TRUE(client->wait_for_service(3s));

    auto request = std::make_shared<omr_controller::srv::MoveArm::Request>();
    request->positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    request->time_from_start = 2.0;

    auto future = client->async_send_request(request);
    EXPECT_TRUE(rclcpp::spin_until_future_complete(node_, future, 3s) ==
                rclcpp::FutureReturnCode::SUCCESS);

    EXPECT_TRUE(move_arm_called_.load());
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_EQ(response->message, "arm moved");

    // Verify request data was received correctly
    ASSERT_EQ(last_move_arm_positions_.size(), 6u);
    EXPECT_DOUBLE_EQ(last_move_arm_positions_[0], 0.1);
    EXPECT_DOUBLE_EQ(last_move_arm_positions_[1], 0.2);
    EXPECT_DOUBLE_EQ(last_move_arm_positions_[5], 0.6);
    EXPECT_DOUBLE_EQ(last_move_arm_time_, 2.0);
}

// ---------------------------------------------------------------------------
// Test case 2: Call MoveRail service with a target position.
// ---------------------------------------------------------------------------
TEST_F(RemoteControlTest, move_rail_service_call) {
    auto client = node_->create_client<omr_controller::srv::MoveRail>("~/move_rail");
    ASSERT_TRUE(client->wait_for_service(3s));

    auto request = std::make_shared<omr_controller::srv::MoveRail::Request>();
    request->position = 0.5;

    auto future = client->async_send_request(request);
    EXPECT_TRUE(rclcpp::spin_until_future_complete(node_, future, 3s) ==
                rclcpp::FutureReturnCode::SUCCESS);

    EXPECT_TRUE(move_rail_called_.load());
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_EQ(response->message, "rail moved");

    EXPECT_DOUBLE_EQ(last_move_rail_position_, 0.5);
}

// ---------------------------------------------------------------------------
// Test case 3: Call ~/set_gripper using std_srvs/SetBool (not a custom srv).
// ---------------------------------------------------------------------------
TEST_F(RemoteControlTest, set_gripper_service_call) {
    auto client = node_->create_client<std_srvs::srv::SetBool>("~/set_gripper");
    ASSERT_TRUE(client->wait_for_service(3s));

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    auto future = client->async_send_request(request);
    EXPECT_TRUE(rclcpp::spin_until_future_complete(node_, future, 3s) ==
                rclcpp::FutureReturnCode::SUCCESS);

    EXPECT_TRUE(set_gripper_called_.load());
    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_TRUE(last_gripper_data_);
}

// ---------------------------------------------------------------------------
// Test case 4: Publish a Twist on cmd_vel, verify subscriber captures it.
// ---------------------------------------------------------------------------
TEST_F(RemoteControlTest, cmd_vel_topic) {
    auto publisher = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Wait for subscriber to connect
    auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node_);
        if (publisher->get_subscription_count() > 0) break;
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(publisher->get_subscription_count(), 0u);

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.5;
    twist.angular.z = 0.3;

    publisher->publish(twist);
    EXPECT_TRUE(waitFor(cmd_vel_received_));

    EXPECT_DOUBLE_EQ(last_twist_.linear.x, 0.5);
    EXPECT_DOUBLE_EQ(last_twist_.angular.z, 0.3);
}

// ---------------------------------------------------------------------------
// Test case 5: Instantiate RemoteControlNode and verify service endpoints exist.
// ---------------------------------------------------------------------------
TEST_F(RemoteControlTest, remote_control_node_services_exist) {
    auto remote_node = std::make_shared<omr_controller::RemoteControlNode>();
    rclcpp::spin_some(remote_node);
    rclcpp::spin_some(node_);

    auto move_arm_cli =
        node_->create_client<omr_controller::srv::MoveArm>("/remote_control/move_arm");
    auto move_rail_cli =
        node_->create_client<omr_controller::srv::MoveRail>("/remote_control/move_rail");
    auto set_gripper_cli =
        node_->create_client<std_srvs::srv::SetBool>("/remote_control/set_gripper");

    EXPECT_TRUE(move_arm_cli->wait_for_service(3s));
    EXPECT_TRUE(move_rail_cli->wait_for_service(3s));
    EXPECT_TRUE(set_gripper_cli->wait_for_service(3s));
}

}  // namespace
