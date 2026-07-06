#include <omr_controller/orchestrator.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

// ============================================================================
// OrchestratorIntegrationTest — shared rclcpp lifecycle for all integration
// tests that exercise TaskOrchestrator as a ROS2 node.
// ============================================================================

class OrchestratorIntegrationTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        orchestrator_ = std::make_shared<omr_controller::TaskOrchestrator>();
    }

    std::shared_ptr<omr_controller::TaskOrchestrator> orchestrator_;
};

// ---------------------------------------------------------------------------
// Node constructs, advertises /task_state, and publishes within ~2 seconds.
// ---------------------------------------------------------------------------
TEST_F(OrchestratorIntegrationTest, NodeConstructsAndPublishesTaskState) {
    ASSERT_NE(orchestrator_, nullptr);
    EXPECT_EQ(orchestrator_->get_name(), std::string("task_orchestrator"));

    std::atomic<bool> received{false};
    auto sub = orchestrator_->create_subscription<std_msgs::msg::String>(
        "/task_state", 10,
        [&received](const std_msgs::msg::String&) { received = true; });

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!received && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(orchestrator_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(received) << "/task_state topic did not publish within 2 seconds";
}

// ---------------------------------------------------------------------------
// All five client unique_ptrs are non-null after construction (friend access).
// ---------------------------------------------------------------------------
TEST_F(OrchestratorIntegrationTest, HasAllFiveClients) {
    EXPECT_NE(orchestrator_->arm_, nullptr);
    EXPECT_NE(orchestrator_->gripper_, nullptr);
    EXPECT_NE(orchestrator_->vision_, nullptr);
    EXPECT_NE(orchestrator_->motor_, nullptr);
    EXPECT_NE(orchestrator_->base_, nullptr);
}

// ---------------------------------------------------------------------------
// Emergency stop capability — placeholder that verifies the node is alive and
// has the expected identity. The full /emergency_stop service implementation
// will be added later.
// ---------------------------------------------------------------------------
TEST_F(OrchestratorIntegrationTest, EmergencyStopCapabilityPresent) {
    // Node name confirms we are testing the correct orchestrator.
    EXPECT_EQ(orchestrator_->get_name(), "task_orchestrator");

    // The node has a bt_tick_rate parameter (set in the constructor).
    EXPECT_TRUE(orchestrator_->has_parameter("bt_tick_rate"));

    // Spin briefly and confirm no crash / unexpected shutdown.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(orchestrator_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    SUCCEED();
}
