#include "omr_controller/clients/base_client.hpp"

#include <gtest/gtest.h>

using namespace omr_controller;

TEST(BaseClientStubTest, MoveReturnsFalse) {
  BaseClientStub stub;
  EXPECT_FALSE(stub.move(0.5, 0.0));
}

TEST(BaseClientStubTest, StopReturnsFalse) {
  BaseClientStub stub;
  EXPECT_FALSE(stub.stop());
}

TEST(BaseClientStubTest, GetPoseReturnsZeros) {
  BaseClientStub stub;
  auto pose = stub.getPose();
  EXPECT_DOUBLE_EQ(pose[0], 0.0);
  EXPECT_DOUBLE_EQ(pose[1], 0.0);
  EXPECT_DOUBLE_EQ(pose[2], 0.0);
}

TEST(BaseClientStubTest, Polymorphic) {
  BaseClient* client = new BaseClientStub();
  EXPECT_FALSE(client->move(0.2, 0.1));
  auto pose = client->getPose();
  EXPECT_DOUBLE_EQ(pose[0], 0.0);
  EXPECT_DOUBLE_EQ(pose[1], 0.0);
  EXPECT_DOUBLE_EQ(pose[2], 0.0);
  delete client;
}
