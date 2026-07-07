#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "omr_controller/state_machine/door_trajectory_action.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {

BT::NodeConfig make_config(rclcpp::Node::SharedPtr ros_node) {
    BT::NodeConfig cfg;
    cfg.blackboard = BT::Blackboard::create();
    cfg.blackboard->set("ros_node", ros_node);
    return cfg;
}

class DoorTrajectoryNodeTest : public ::testing::Test {
protected:
    void SetUp() override {
        ros_node_ = std::make_shared<rclcpp::Node>("param_test");
        auto cfg = make_config(ros_node_);
        node_ = std::make_shared<omr_controller::DoorTrajectoryAction>("door_traj", cfg);
        node_->executeTick();  // onStart() reads ports
    }

    rclcpp::Node::SharedPtr ros_node_;
    std::shared_ptr<omr_controller::DoorTrajectoryAction> node_;
};

// ──── Construction and parameter defaults ────────────────────────────

TEST_F(DoorTrajectoryNodeTest, RegistersCorrectly) { EXPECT_NE(node_, nullptr); }

TEST_F(DoorTrajectoryNodeTest, ScalarParametersHaveDefaults) {
    EXPECT_DOUBLE_EQ(node_->r(), 2.0);
    EXPECT_DOUBLE_EQ(node_->L(), 1.5);
    EXPECT_DOUBLE_EQ(node_->h(), 0.0);
    EXPECT_DOUBLE_EQ(node_->theta_step_deg(), 5.0);
    EXPECT_DOUBLE_EQ(node_->joint_state_tolerance(), 0.01);
    EXPECT_DOUBLE_EQ(node_->door_frame_radius(), 0.05);
}

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

// ──── Custom parameter override (via BT ports) ────────────────────────────

TEST(DoorTrajectoryNodeCustomParamTest, CustomRValueIsUsed) {
    auto ros_node = std::make_shared<rclcpp::Node>("custom_param");
    auto cfg = make_config(ros_node);
    cfg.input_ports["r"] = "3.5";

    auto node = std::make_shared<omr_controller::DoorTrajectoryAction>("door_traj", cfg);
    node->executeTick();

    EXPECT_DOUBLE_EQ(node->r(), 3.5);
    EXPECT_DOUBLE_EQ(node->L(), 1.5);  // Other params still default
}

TEST(DoorTrajectoryNodeCustomParamTest, CustomVectorParamIsUsed) {
    auto ros_node = std::make_shared<rclcpp::Node>("custom_param");
    auto cfg = make_config(ros_node);
    cfg.input_ports["phi_values"] = "10,20,30";

    auto node = std::make_shared<omr_controller::DoorTrajectoryAction>("door_traj", cfg);
    node->executeTick();

    const auto& v = node->phi_values_deg();
    ASSERT_EQ(v.size(), 3u);
    EXPECT_DOUBLE_EQ(v[0], 10.0);
    EXPECT_DOUBLE_EQ(v[1], 20.0);
    EXPECT_DOUBLE_EQ(v[2], 30.0);
}

// ──── ros_node provided on blackboard ─────────────────────────────────────

TEST_F(DoorTrajectoryNodeTest, RosNodeAvailableAfterOnStart) {
    EXPECT_NE(node_->rosNode(), nullptr);
}

// ──── Provided ports list is non-empty ─────────────────────────────────────

TEST(DoorTrajectoryActionPortTest, ProvidedPortsAreNonEmpty) {
    auto ports = omr_controller::DoorTrajectoryAction::providedPorts();
    EXPECT_GT(ports.size(), 0u);
}

}  // namespace
