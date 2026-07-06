#include "omr_controller/clients/motor_client.hpp"

#include <gtest/gtest.h>

using namespace omr_controller;

TEST(MotorClientStubTest, EnableReturnsFalse) {
    MotorClientStub stub;
    EXPECT_FALSE(stub.enable());
}

TEST(MotorClientStubTest, DisableReturnsFalse) {
    MotorClientStub stub;
    EXPECT_FALSE(stub.disable());
}

TEST(MotorClientStubTest, IsEnabledReturnsFalse) {
    MotorClientStub stub;
    EXPECT_FALSE(stub.isEnabled());
}

TEST(MotorClientStubTest, SetVelocityReturnsFalse) {
    MotorClientStub stub;
    EXPECT_FALSE(stub.setVelocity(1.0));
}

TEST(MotorClientStubTest, StopReturnsFalse) {
    MotorClientStub stub;
    EXPECT_FALSE(stub.stop());
}

TEST(MotorClientStubTest, GetStateDefault) {
    MotorClientStub stub;
    MotorState state = stub.getState();
    EXPECT_DOUBLE_EQ(state.position_rad, 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_rad_s, 0.0);
    EXPECT_FALSE(state.connected);
    EXPECT_FALSE(state.servo_enabled);
}

TEST(MotorClientStubTest, Polymorphic) {
    MotorClient* client = new MotorClientStub();
    EXPECT_FALSE(client->enable());
    EXPECT_FALSE(client->isEnabled());
    EXPECT_DOUBLE_EQ(client->getState().position_rad, 0.0);
    delete client;
}
