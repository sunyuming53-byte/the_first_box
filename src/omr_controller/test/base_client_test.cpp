#include "omr_controller/clients/base_client.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

using namespace omr_controller;

class BaseClientImplTest : public ::testing::Test {
protected:
    void SetUp() override { node_ = std::make_shared<rclcpp::Node>("base_client_test"); }

    rclcpp::Node::SharedPtr node_;
};

TEST_F(BaseClientImplTest, MoveReturnsTrue) {
    BaseClientImpl impl(node_);
    EXPECT_TRUE(impl.move(0.5, 0.0));
}

TEST_F(BaseClientImplTest, StopReturnsTrue) {
    BaseClientImpl impl(node_);
    EXPECT_TRUE(impl.stop());
}

TEST_F(BaseClientImplTest, GetPoseDefaultsToZeros) {
    BaseClientImpl impl(node_);
    auto pose = impl.getPose();
    EXPECT_DOUBLE_EQ(pose[0], 0.0);
    EXPECT_DOUBLE_EQ(pose[1], 0.0);
    EXPECT_DOUBLE_EQ(pose[2], 0.0);
}

TEST_F(BaseClientImplTest, Polymorphic) {
    BaseClient* client = new BaseClientImpl(node_);
    EXPECT_TRUE(client->move(0.2, 0.1));
    auto pose = client->getPose();
    EXPECT_DOUBLE_EQ(pose[0], 0.0);
    EXPECT_DOUBLE_EQ(pose[1], 0.0);
    EXPECT_DOUBLE_EQ(pose[2], 0.0);
    delete client;
}
