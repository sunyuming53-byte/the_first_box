#include <omr_controller/orchestrator.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

class OrchestratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        orchestrator_ = std::make_shared<omr_controller::TaskOrchestrator>();
    }

    void TearDown() override {
        orchestrator_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<omr_controller::TaskOrchestrator> orchestrator_;
};

TEST_F(OrchestratorTest, ConstructsWithoutThrow) {
    EXPECT_NE(orchestrator_, nullptr);
    EXPECT_EQ(orchestrator_->get_name(), std::string("task_orchestrator"));
}

TEST_F(OrchestratorTest, TaskStatePublisherExists) {
    std::atomic<bool> received{false};

    auto sub = orchestrator_->create_subscription<std_msgs::msg::String>(
        "/task_state", 10,
        [&received](const std_msgs::msg::String& msg) {
            received = true;
        });

    // Spin until we receive a message or timeout
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!received && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(orchestrator_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(received) << "/task_state did not publish within 2 seconds";
}

TEST_F(OrchestratorTest, NodeAliveAfter500msSpin) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(orchestrator_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Node survived 500ms of spinning without crashing
    SUCCEED();
}
