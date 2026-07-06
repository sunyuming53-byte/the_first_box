#include "omr_controller/types.hpp"

#include <gtest/gtest.h>

using namespace omr_controller;

TEST(TypesTest, Defaults) {
    JointGoal jg;
    EXPECT_EQ(jg.speed_ratio, 50);
    EXPECT_DOUBLE_EQ(jg.time_from_start_sec, 0.0);
    EXPECT_TRUE(jg.positions.empty());

    TaskResult tr;
    EXPECT_FALSE(tr.success);
    EXPECT_TRUE(tr.message.empty());

    ArmState as;
    EXPECT_FALSE(as.connected);
    EXPECT_TRUE(as.joint_positions.empty());

    MotorState ms;
    EXPECT_DOUBLE_EQ(ms.position_rad, 0.0);
    EXPECT_DOUBLE_EQ(ms.velocity_rad_s, 0.0);
    EXPECT_FALSE(ms.connected);
    EXPECT_FALSE(ms.servo_enabled);

    DetectionResult dr;
    EXPECT_TRUE(dr.label.empty());
    EXPECT_DOUBLE_EQ(dr.center.x, 0.0);
    EXPECT_DOUBLE_EQ(dr.center.y, 0.0);
    EXPECT_DOUBLE_EQ(dr.confidence, 0.0);
}

TEST(TypesTest, Assignment) {
    JointGoal jg;
    jg.positions = {1.0, 2.0, 3.0};
    jg.speed_ratio = 75;
    jg.time_from_start_sec = 2.5;
    ASSERT_EQ(jg.positions.size(), 3);
    EXPECT_DOUBLE_EQ(jg.positions[0], 1.0);
    EXPECT_DOUBLE_EQ(jg.positions[1], 2.0);
    EXPECT_DOUBLE_EQ(jg.positions[2], 3.0);
    EXPECT_EQ(jg.speed_ratio, 75);
    EXPECT_DOUBLE_EQ(jg.time_from_start_sec, 2.5);

    TaskResult tr;
    tr.success = true;
    tr.message = "done";
    EXPECT_TRUE(tr.success);
    EXPECT_EQ(tr.message, "done");

    ArmState as;
    as.joint_positions = {0.1, 0.2};
    as.connected = true;
    ASSERT_EQ(as.joint_positions.size(), 2);
    EXPECT_DOUBLE_EQ(as.joint_positions[0], 0.1);
    EXPECT_TRUE(as.connected);

    MotorState ms;
    ms.position_rad = 1.57;
    ms.velocity_rad_s = 0.5;
    ms.connected = true;
    ms.servo_enabled = true;
    EXPECT_DOUBLE_EQ(ms.position_rad, 1.57);
    EXPECT_DOUBLE_EQ(ms.velocity_rad_s, 0.5);
    EXPECT_TRUE(ms.connected);
    EXPECT_TRUE(ms.servo_enabled);

    DetectionResult dr;
    dr.label = "bolt";
    dr.center = {320, 240};
    dr.confidence = 0.95;
    EXPECT_EQ(dr.label, "bolt");
    EXPECT_DOUBLE_EQ(dr.center.x, 320);
    EXPECT_DOUBLE_EQ(dr.center.y, 240);
    EXPECT_DOUBLE_EQ(dr.confidence, 0.95);
}
