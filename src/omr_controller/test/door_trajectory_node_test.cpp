#include "omr_controller/door_trajectory_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class DoorTrajectoryNodeTest : public ::testing::Test {
protected:
    void SetUp() override { node_ = std::make_shared<omr_controller::DoorTrajectoryNode>(); }

    std::shared_ptr<omr_controller::DoorTrajectoryNode> node_;
};

// ──── Construction and naming ────────────────────────────────────────────

TEST_F(DoorTrajectoryNodeTest, ConstructsWithoutThrow) {
    EXPECT_NE(node_, nullptr);
    EXPECT_EQ(node_->get_name(), std::string("door_trajectory_node"));
}

// ──── Scalar parameter defaults ──────────────────────────────────────────

TEST_F(DoorTrajectoryNodeTest, ScalarParametersHaveDefaults) {
    EXPECT_DOUBLE_EQ(node_->r(), 2.0);
    EXPECT_DOUBLE_EQ(node_->L(), 1.5);
    EXPECT_DOUBLE_EQ(node_->h(), 0.0);
    EXPECT_DOUBLE_EQ(node_->theta_step_deg(), 5.0);
    EXPECT_DOUBLE_EQ(node_->joint_state_tolerance(), 0.01);
    EXPECT_DOUBLE_EQ(node_->door_frame_radius(), 0.05);
}

// ──── Vector parameter defaults ──────────────────────────────────────────

TEST_F(DoorTrajectoryNodeTest, T_armBase_doorHingeHasDefault) {
    const auto& v = node_->T_armBase_doorHinge();
    ASSERT_EQ(v.size(), 6u);
    for (double val : v) {
        EXPECT_DOUBLE_EQ(val, 0.0);
    }
}

TEST_F(DoorTrajectoryNodeTest, PhiValuesDegHasDefault) {
    const auto& v = node_->phi_values_deg();
    ASSERT_EQ(v.size(), 7u);
    const double expected[] = {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0};
    for (size_t i = 0; i < v.size(); ++i) {
        EXPECT_DOUBLE_EQ(v[i], expected[i]) << "i=" << i;
    }
}

TEST_F(DoorTrajectoryNodeTest, HomeJointsHasDefault) {
    const auto& v = node_->home_joints();
    ASSERT_EQ(v.size(), 6u);
    for (double val : v) {
        EXPECT_DOUBLE_EQ(val, 0.0);
    }
}

TEST_F(DoorTrajectoryNodeTest, DoorPanelSizeHasDefault) {
    const auto& v = node_->door_panel_size();
    ASSERT_EQ(v.size(), 3u);
    EXPECT_DOUBLE_EQ(v[0], 2.0);
    EXPECT_DOUBLE_EQ(v[1], 0.05);
    EXPECT_DOUBLE_EQ(v[2], 0.8);
}

// ──── Custom parameter override (via NodeOptions) ────────────────────────

TEST(DoorTrajectoryNodeCustomParamTest, CustomRValueIsUsed) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("r", 3.5);

    auto node = std::make_shared<omr_controller::DoorTrajectoryNode>(opts);
    EXPECT_DOUBLE_EQ(node->r(), 3.5);
    // Other params should still have defaults
    EXPECT_DOUBLE_EQ(node->L(), 1.5);
}

TEST(DoorTrajectoryNodeCustomParamTest, CustomVectorParamIsUsed) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("phi_values_deg", std::vector<double>({10.0, 20.0, 30.0}));

    auto node = std::make_shared<omr_controller::DoorTrajectoryNode>(opts);
    const auto& v = node->phi_values_deg();
    ASSERT_EQ(v.size(), 3u);
    EXPECT_DOUBLE_EQ(v[0], 10.0);
    EXPECT_DOUBLE_EQ(v[1], 20.0);
    EXPECT_DOUBLE_EQ(v[2], 30.0);
}

// ──── Node alive after spin ──────────────────────────────────────────────

TEST_F(DoorTrajectoryNodeTest, NodeAliveAfterSpinSome) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    SUCCEED();
}

// ──── State publisher exists and publishes ───────────────────────────────

TEST_F(DoorTrajectoryNodeTest, StatePublisherPublishes) {
    // Subscribe first on a helper node so we receive IDLE published
    // during DoorTrajectoryNode construction.
    auto helper = std::make_shared<rclcpp::Node>("helper");
    std::atomic<bool> received{false};

    auto sub = helper->create_subscription<std_msgs::msg::String>(
        "/door_trajectory_node/state", 10, [&received](const std_msgs::msg::String& msg) {
            EXPECT_EQ(msg.data, "IDLE");
            received = true;
        });

    // Now construct the DoorTrajectoryNode — IDLE is published in constructor.
    auto dt_node = std::make_shared<omr_controller::DoorTrajectoryNode>();

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!received && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(helper->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(received) << "/door_trajectory_node/state did not publish 'IDLE' within 1 second";
}
