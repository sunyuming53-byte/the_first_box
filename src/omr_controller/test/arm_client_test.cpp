#include "omr_controller/clients/arm_client.hpp"

#include <stdexcept>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

class ArmClientTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("test_arm_client");
        client_ = std::make_unique<omr_controller::ArmClient>(node_.get());
    }

    void TearDown() override {
        client_.reset();
        node_.reset();
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<omr_controller::ArmClient> client_;
};

TEST_F(ArmClientTest, ConstructionSucceeds) {
    EXPECT_EQ(client_->currentJoints().size(), 0u);
}

TEST_F(ArmClientTest, CurrentJointsEmptyBeforeMessages) {
    auto joints = client_->currentJoints();
    EXPECT_TRUE(joints.empty());
}

TEST_F(ArmClientTest, IsMovingFalseInitially) {
    EXPECT_FALSE(client_->isMoving());
}

TEST_F(ArmClientTest, MovePoseThrowsLogicError) {
    geometry_msgs::msg::Pose pose;
    EXPECT_THROW(client_->movePose(pose), std::logic_error);
}

TEST_F(ArmClientTest, MoveJointsNoServerDoesNotThrow) {
    omr_controller::JointGoal goal;
    goal.positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    EXPECT_NO_THROW(client_->moveJoints(goal));
}
