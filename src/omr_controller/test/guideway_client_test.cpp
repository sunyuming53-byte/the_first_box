#include "omr_controller/clients/guideway_client.hpp"

#include <gtest/gtest.h>

using namespace omr_controller;

// ── Stub behavior (mirrors motor_client_test.cpp depth) ──

TEST(GuidewayClientStubTest, ConnectReturnsFalse) {
    GuidewayClientStub stub;
    EXPECT_FALSE(stub.connect());
    EXPECT_FALSE(stub.isConnected());
}

TEST(GuidewayClientStubTest, CalibrateReturnsFalse) {
    GuidewayClientStub stub;
    EXPECT_FALSE(stub.calibrate());
    EXPECT_FALSE(stub.isCalibrated());
    EXPECT_FALSE(stub.isHomed());
}

TEST(GuidewayClientStubTest, VelocityCommandsReturnFalse) {
    GuidewayClientStub stub;
    EXPECT_FALSE(stub.setVelocity(0.01));
    EXPECT_FALSE(stub.jog(GuidewayDirection::UP, 10.0));
    EXPECT_FALSE(stub.stop());
}

TEST(GuidewayClientStubTest, PositionCommandsReturnFalse) {
    GuidewayClientStub stub;
    EXPECT_FALSE(stub.moveToRail(0.05, 30.0));
    EXPECT_FALSE(stub.moveToRailAsync(0.05));
    EXPECT_FALSE(stub.waitMotionDone(1.0));
    EXPECT_FALSE(stub.moveToHome(30.0));
}

TEST(GuidewayClientStubTest, MotionDoneDefaultsTrue) {
    GuidewayClientStub stub;
    EXPECT_TRUE(stub.motionDone());
}

TEST(GuidewayClientStubTest, StatusDefaults) {
    GuidewayClientStub stub;
    const GuidewayStatus st = stub.status();
    EXPECT_FALSE(st.connected);
    EXPECT_FALSE(st.calibrated);
    EXPECT_FALSE(st.homed);
    EXPECT_TRUE(st.motion_done);
    EXPECT_EQ(st.safety, GuidewaySafetyState::NORMAL);
    EXPECT_DOUBLE_EQ(st.rail_position_m, 0.0);
    EXPECT_DOUBLE_EQ(stub.railPositionM(), 0.0);
    EXPECT_DOUBLE_EQ(stub.railVelocityMps(), 0.0);
}

TEST(GuidewayClientStubTest, CalibrationDefaults) {
    GuidewayClientStub stub;
    const GuidewayCalibration c = stub.calibration();
    EXPECT_FALSE(c.calibrated);
    EXPECT_FALSE(c.homed);
    EXPECT_DOUBLE_EQ(c.total_travel_m, 0.0);
}

TEST(GuidewayClientStubTest, ClearFaultReturnsFalse) {
    GuidewayClientStub stub;
    EXPECT_FALSE(stub.clearFault());
}

TEST(GuidewayClientStubTest, Polymorphic) {
    GuidewayClient* client = new GuidewayClientStub();
    EXPECT_FALSE(client->connect());
    EXPECT_FALSE(client->isCalibrated());
    EXPECT_TRUE(client->motionDone());
    delete client;
}

// ── Impl: lazy connection with unavailable hardware (no serial in CI) ──

namespace {

GuidewayConfig unavailableHardwareConfig() {
    GuidewayConfig cfg;
    cfg.photogate_port = "/dev/nonexistent_photogate";
    cfg.motor_port = "/dev/nonexistent_motor";
    return cfg;
}

}  // namespace

TEST(GuidewayClientImplTest, ConstructionDoesNotOpenPorts) {
    GuidewayClientImpl impl(unavailableHardwareConfig(), rclcpp::get_logger("guideway_test"));
    // Lazy connection: constructing with bogus ports must succeed and stay
    // disconnected until a command is issued.
    EXPECT_FALSE(impl.isConnected());
    EXPECT_FALSE(impl.status().connected);
}

TEST(GuidewayClientImplTest, CommandsFailCleanlyWithoutHardware) {
    GuidewayClientImpl impl(unavailableHardwareConfig(), rclcpp::get_logger("guideway_test"));
    EXPECT_FALSE(impl.connect());
    EXPECT_FALSE(impl.calibrate());
    EXPECT_FALSE(impl.setVelocity(0.01));
    EXPECT_FALSE(impl.jog(GuidewayDirection::DOWN, 5.0));
    EXPECT_FALSE(impl.moveToRail(0.01, 1.0));
    EXPECT_FALSE(impl.moveToHome(1.0));
    EXPECT_FALSE(impl.clearFault());
    EXPECT_FALSE(impl.isConnected());
}

TEST(GuidewayClientImplTest, StopWithoutHardwareIsSafeNoOp) {
    GuidewayClientImpl impl(unavailableHardwareConfig(), rclcpp::get_logger("guideway_test"));
    EXPECT_TRUE(impl.stop());  // nothing moving — safe no-op
    EXPECT_TRUE(impl.motionDone());
}

TEST(GuidewayClientImplTest, StatusQueriesSafeWithoutHardware) {
    GuidewayClientImpl impl(unavailableHardwareConfig(), rclcpp::get_logger("guideway_test"));
    EXPECT_FALSE(impl.isCalibrated());
    EXPECT_FALSE(impl.isHomed());
    EXPECT_EQ(impl.safetyState(), GuidewaySafetyState::NORMAL);
    EXPECT_DOUBLE_EQ(impl.railPositionM(), 0.0);
    EXPECT_DOUBLE_EQ(impl.railVelocityMps(), 0.0);
}
