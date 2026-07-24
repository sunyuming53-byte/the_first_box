// SPDX-License-Identifier: Apache-2.0
// Unit tests for omr_hardware::loadRailConfig (flat YAML parser).

#include "omr_hardware/rail_controller.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

class RailConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = (std::filesystem::temp_directory_path() /
                 ("rail_config_test_" + std::to_string(::getpid()) + ".yaml"))
                    .string();
    }

    void TearDown() override { std::remove(path_.c_str()); }

    void write(const std::string& contents) {
        std::ofstream out(path_);
        out << contents;
    }

    std::string path_;
};

TEST_F(RailConfigTest, MissingFileFails) {
    omr_hardware::RailControllerConfig cfg;
    std::string error;
    EXPECT_FALSE(omr_hardware::loadRailConfig("/nonexistent/rail.yaml", cfg, &error));
    EXPECT_FALSE(error.empty());
}

TEST_F(RailConfigTest, ParsesAllKeys) {
    write(R"(# comment
photogate_port: /dev/ttyACM7
photogate_baud: 230400
gate_count: 3
lower_gate: 2
home_gate: 0
upper_gate: 1
motor_port: "/dev/ttyUSB3"
motor_baud: 115200
slave_id: 4
up_sign: 1
screw_lead_m: 0.02
watchdog_ms: 350
coarse_rpm: 99
fine_rpm: 4.5
backoff_rpm: 12
seek_timeout_s: 60
home_tol_rad: 0.01
return_timeout_s: 30
position_max_rpm: 120
position_accel_ms: 200
soft_limit_margin_m: 0.005
)");
    omr_hardware::RailControllerConfig cfg;
    std::string error;
    ASSERT_TRUE(omr_hardware::loadRailConfig(path_, cfg, &error)) << error;
    EXPECT_EQ(cfg.photogate_port, "/dev/ttyACM7");
    EXPECT_EQ(cfg.photogate_baud, 230400);
    EXPECT_EQ(cfg.gate_count, 3);
    EXPECT_EQ(cfg.lower_gate, 2);
    EXPECT_EQ(cfg.home_gate, 0);
    EXPECT_EQ(cfg.upper_gate, 1);
    EXPECT_EQ(cfg.motor_port, "/dev/ttyUSB3");
    EXPECT_EQ(cfg.motor_baud, 115200);
    EXPECT_EQ(cfg.slave_id, 4);
    EXPECT_EQ(cfg.up_sign, 1);
    EXPECT_DOUBLE_EQ(cfg.screw_lead_m, 0.02);
    EXPECT_DOUBLE_EQ(cfg.watchdog_ms, 350.0);
    EXPECT_DOUBLE_EQ(cfg.coarse_rpm, 99.0);
    EXPECT_DOUBLE_EQ(cfg.fine_rpm, 4.5);
    EXPECT_DOUBLE_EQ(cfg.backoff_rpm, 12.0);
    EXPECT_DOUBLE_EQ(cfg.seek_timeout_s, 60.0);
    EXPECT_DOUBLE_EQ(cfg.home_tol_rad, 0.01);
    EXPECT_DOUBLE_EQ(cfg.return_timeout_s, 30.0);
    EXPECT_EQ(cfg.position_max_rpm, 120);
    EXPECT_EQ(cfg.position_accel_ms, 200);
    EXPECT_DOUBLE_EQ(cfg.soft_limit_margin_m, 0.005);
}

TEST_F(RailConfigTest, KeepsDefaultsForMissingKeys) {
    write("motor_port: /dev/ttyUSB9\n");
    omr_hardware::RailControllerConfig cfg;
    ASSERT_TRUE(omr_hardware::loadRailConfig(path_, cfg, nullptr));
    EXPECT_EQ(cfg.motor_port, "/dev/ttyUSB9");
    // untouched defaults
    EXPECT_EQ(cfg.photogate_port, "/dev/ttyACM0");
    EXPECT_EQ(cfg.up_sign, -1);
    EXPECT_DOUBLE_EQ(cfg.screw_lead_m, 0.01);
}

TEST_F(RailConfigTest, InvalidUpSignFails) {
    write("up_sign: 2\n");
    omr_hardware::RailControllerConfig cfg;
    std::string error;
    EXPECT_FALSE(omr_hardware::loadRailConfig(path_, cfg, &error));
    EXPECT_NE(error.find("up_sign"), std::string::npos);
}

TEST_F(RailConfigTest, UnparsableValueFails) {
    write("motor_baud: not_a_number\n");
    omr_hardware::RailControllerConfig cfg;
    std::string error;
    EXPECT_FALSE(omr_hardware::loadRailConfig(path_, cfg, &error));
    EXPECT_FALSE(error.empty());
}

TEST_F(RailConfigTest, IgnoresUnknownKeysAndComments) {
    write(R"(
# header comment
some_future_key: 42
coarse_rpm: 33
)");
    omr_hardware::RailControllerConfig cfg;
    ASSERT_TRUE(omr_hardware::loadRailConfig(path_, cfg, nullptr));
    EXPECT_DOUBLE_EQ(cfg.coarse_rpm, 33.0);
}

}  // namespace
