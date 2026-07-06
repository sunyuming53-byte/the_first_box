#include "omr_controller/clients/gripper_client.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace omr_controller;

// ---------------------------------------------------------------------------
// Helper: create a mock GripperCommand action server that auto-accepts and
// immediately succeeds every goal.
// ---------------------------------------------------------------------------
namespace {

auto create_mock_gripper_server(
    rclcpp::Node* node,
    const std::string& name)
    -> std::shared_ptr<rclcpp_action::Server<control_msgs::action::GripperCommand>>
{
  using Action = control_msgs::action::GripperCommand;

  auto handle_goal =
      [](const rclcpp_action::GoalUUID&,
         std::shared_ptr<const Action::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };

  auto handle_cancel =
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>&) {
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  auto handle_accepted =
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>& gh) {
    auto result = std::make_shared<Action::Result>();
    gh->succeed(result);
  };

  return rclcpp_action::create_server<Action>(
      node, name, handle_goal, handle_cancel, handle_accepted);
}

}  // namespace

// ---------------------------------------------------------------------------
// Fixture with rclcpp lifecycle for tests that need the ROS 2 graph.
// ---------------------------------------------------------------------------
class GripperClientTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_gripper");
  }

  void TearDown() override {
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(GripperClientTest, ConstructDefaultActionName) {
  GripperClient client(node_.get());
  (void)client;
}

TEST_F(GripperClientTest, ConstructCustomActionName) {
  GripperClient client(node_.get(), "/custom/gripper");
  (void)client;
}

TEST_F(GripperClientTest, OpenOnMockServerReturnsTrue) {
  auto server = create_mock_gripper_server(
      node_.get(), "/gripper/follow_joint_trajectory");
  rclcpp::spin_some(node_);

  GripperClient client(node_.get());
  EXPECT_TRUE(client.open());
}

TEST_F(GripperClientTest, CloseOnMockServerReturnsTrue) {
  auto server = create_mock_gripper_server(
      node_.get(), "/gripper/follow_joint_trajectory");
  rclcpp::spin_some(node_);

  GripperClient client(node_.get());
  EXPECT_TRUE(client.close());
}

TEST_F(GripperClientTest, NoActionServerOpenReturnsFalse) {
  GripperClient client(node_.get());
  EXPECT_FALSE(client.open());
}

TEST_F(GripperClientTest, NoActionServerCloseReturnsFalse) {
  GripperClient client(node_.get());
  EXPECT_FALSE(client.close());
}
