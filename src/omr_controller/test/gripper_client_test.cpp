#include "omr_controller/clients/gripper_client.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

using namespace omr_controller;

class GripperClientTest : public ::testing::Test {
 protected:
  void SetUp() override { node_ = std::make_shared<rclcpp::Node>("test_gripper"); }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(GripperClientTest, ConstructDefaultActionName) {
  EXPECT_NO_THROW({
    GripperClient client(node_.get());
  });
}

TEST_F(GripperClientTest, ConstructCustomActionName) {
  EXPECT_NO_THROW({
    GripperClient client(node_.get(), "/custom/gripper");
  });
}
