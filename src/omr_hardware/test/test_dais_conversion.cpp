#include <cmath>
#include <gtest/gtest.h>

constexpr double kPi = 3.14159265358979323846;

// ── Conversion functions (mirrors DaisHardware implementation) ──

/// position_rad → position_m (prismatic joint)
inline double pos_rad_to_m(double rad, double lead_m) { return rad * lead_m / (2.0 * kPi); }

/// velocity_rpm → velocity_m_s (prismatic joint)
inline double vel_rpm_to_mps(double rpm, double lead_m) { return rpm * lead_m / 60.0; }

/// velocity_m_s → motor_rad_s (inverse for write path)
inline double vel_mps_to_radps(double mps, double lead_m) { return mps * (2.0 * kPi) / lead_m; }

// ═══════════════════════════════════════════════════════════════
// Position conversion tests
// ═══════════════════════════════════════════════════════════════

TEST(PositionConversion, ZeroRadToZeroM) { EXPECT_DOUBLE_EQ(pos_rad_to_m(0.0, 0.01), 0.0); }

TEST(PositionConversion, TwoPiRadToLead) {
    // One full revolution → one screw lead of linear motion
    EXPECT_NEAR(pos_rad_to_m(2.0 * kPi, 0.01), 0.01, 1e-9);
}

TEST(PositionConversion, NegativePreservesSign) {
    EXPECT_NEAR(pos_rad_to_m(-2.0 * kPi, 0.01), -0.01, 1e-9);
}

TEST(PositionConversion, HalfRevolution) { EXPECT_NEAR(pos_rad_to_m(kPi, 0.01), 0.005, 1e-9); }

TEST(PositionConversion, DifferentLeads) {
    EXPECT_NEAR(pos_rad_to_m(2.0 * kPi, 0.005), 0.005, 1e-9);  // 5mm lead
    EXPECT_NEAR(pos_rad_to_m(2.0 * kPi, 0.010), 0.010, 1e-9);  // 10mm lead
    EXPECT_NEAR(pos_rad_to_m(2.0 * kPi, 0.020), 0.020, 1e-9);  // 20mm lead
}

// ═══════════════════════════════════════════════════════════════
// Velocity read conversion tests (rpm → m/s)
// ═══════════════════════════════════════════════════════════════

TEST(VelocityReadConversion, ZeroRpmToZeroMps) { EXPECT_DOUBLE_EQ(vel_rpm_to_mps(0.0, 0.01), 0.0); }

TEST(VelocityReadConversion, SixHundredRpmToPointOneMps) {
    // 600 RPM with 10mm lead → 0.1 m/s
    EXPECT_NEAR(vel_rpm_to_mps(600.0, 0.01), 0.1, 1e-9);
}

TEST(VelocityReadConversion, NegativeReversesDirection) {
    EXPECT_NEAR(vel_rpm_to_mps(-600.0, 0.01), -0.1, 1e-9);
}

TEST(VelocityReadConversion, DifferentLeads) {
    EXPECT_NEAR(vel_rpm_to_mps(600.0, 0.005), 0.05, 1e-9);  // 5mm lead
    EXPECT_NEAR(vel_rpm_to_mps(600.0, 0.020), 0.20, 1e-9);  // 20mm lead
}

// ═══════════════════════════════════════════════════════════════
// Velocity write conversion tests (m/s → rad/s)
// ═══════════════════════════════════════════════════════════════

TEST(VelocityWriteConversion, ZeroMpsToZeroRadps) {
    EXPECT_DOUBLE_EQ(vel_mps_to_radps(0.0, 0.01), 0.0);
}

TEST(VelocityWriteConversion, PointOneMpsToRadps) {
    // 0.1 m/s with 10mm lead → 62.832 rad/s
    EXPECT_NEAR(vel_mps_to_radps(0.1, 0.01), 62.83185307179586, 1e-6);
}

TEST(VelocityWriteConversion, NegativeCommand) {
    EXPECT_NEAR(vel_mps_to_radps(-0.1, 0.01), -62.83185307179586, 1e-6);
}

TEST(VelocityWriteConversion, DifferentLeads) {
    EXPECT_NEAR(vel_mps_to_radps(0.1, 0.005), 125.66370614359172, 1e-6);  // 5mm
    EXPECT_NEAR(vel_mps_to_radps(0.1, 0.020), 31.41592653589793, 1e-6);   // 20mm
}

// ═══════════════════════════════════════════════════════════════
// Round-trip tests (write → read should recover original value)
// ═══════════════════════════════════════════════════════════════

TEST(RoundTrip, WriteThenReadVelocity) {
    const double lead = 0.01;
    const double cmd_mps = 0.1;
    double radps = vel_mps_to_radps(cmd_mps, lead);
    // Convert back: rad/s → m/s via vel_rpm_to_mps but first rad/s → rpm
    double rpm = radps * 60.0 / (2.0 * kPi);
    double recovered_mps = vel_rpm_to_mps(rpm, lead);
    EXPECT_NEAR(cmd_mps, recovered_mps, 1e-9);
}

TEST(RoundTrip, NegativeVelocity) {
    const double lead = 0.005;
    const double cmd_mps = -0.05;
    double radps = vel_mps_to_radps(cmd_mps, lead);
    double rpm = radps * 60.0 / (2.0 * kPi);
    double recovered_mps = vel_rpm_to_mps(rpm, lead);
    EXPECT_NEAR(cmd_mps, recovered_mps, 1e-9);
}
