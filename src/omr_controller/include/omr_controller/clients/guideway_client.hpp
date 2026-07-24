#pragma once

// GuidewayClient — full guideway (rail) motion control client.
//
// Wraps omr_hardware::RailController (photogate limit interlock, full-travel
// calibration, velocity loop, position loop in rail coordinates). Unlike the
// action/topic clients, this client talks to the serial hardware directly
// (same precedent as VisionClient); it therefore MUST NOT run at the same
// time as the DaisHardware/PhotogateHardware ros2_control plugins, which own
// the same serial ports. Connection is lazy: ports are only opened on the
// first command, so constructing the client is always safe.
//
// Rail coordinate convention: meters, 0 at the calibrated home edge,
// positive = up. All methods are thread-safe.

#include <memory>

#include "omr_hardware/rail_controller.hpp"
#include <rclcpp/rclcpp.hpp>

namespace omr_controller {

using GuidewayConfig = omr_hardware::RailControllerConfig;
using GuidewayStatus = omr_hardware::RailStatus;
using GuidewayCalibration = omr_hardware::RailCalibration;
using GuidewaySafetyState = omr_hardware::RailSafetyState;
using GuidewayDirection = omr_hardware::RailDirection;

class GuidewayClient {
public:
    virtual ~GuidewayClient() = default;

    // ── Connection ──
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;

    // ── Status (non-blocking; suitable for BT tick polling) ──
    virtual GuidewayStatus status() const = 0;
    virtual GuidewayCalibration calibration() const = 0;
    virtual bool isCalibrated() const = 0;
    virtual bool isHomed() const = 0;
    virtual GuidewaySafetyState safetyState() const = 0;
    virtual double railPositionM() const = 0;
    virtual double railVelocityMps() const = 0;

    // ── Calibration (blocking; can take minutes) ──
    virtual bool calibrate() = 0;

    // ── Velocity loop (interlock protected; works without calibration) ──
    /// Rail velocity in m/s, up positive; 0 stops.
    virtual bool setVelocity(double rail_mps) = 0;
    /// Low-level jog at motor rpm (debug/manual use).
    virtual bool jog(GuidewayDirection dir, double rpm) = 0;
    /// Stop now; also aborts a blocking/async operation in progress.
    virtual bool stop() = 0;

    // ── Position loop (requires calibration; soft-limit checked) ──
    virtual bool moveToRail(double rail_m, double timeout_s) = 0;
    virtual bool moveToRailAsync(double rail_m) = 0;
    virtual bool motionDone() const = 0;
    virtual bool waitMotionDone(double timeout_s) = 0;
    virtual bool moveToHome(double timeout_s) = 0;

    // ── Recovery ──
    virtual bool clearFault() = 0;
};

class GuidewayClientStub : public GuidewayClient {
public:
    bool connect() override { return false; }
    void disconnect() override {}
    bool isConnected() const override { return false; }
    GuidewayStatus status() const override { return {}; }
    GuidewayCalibration calibration() const override { return {}; }
    bool isCalibrated() const override { return false; }
    bool isHomed() const override { return false; }
    GuidewaySafetyState safetyState() const override { return GuidewaySafetyState::NORMAL; }
    double railPositionM() const override { return 0.0; }
    double railVelocityMps() const override { return 0.0; }
    bool calibrate() override { return false; }
    bool setVelocity(double /*rail_mps*/) override { return false; }
    bool jog(GuidewayDirection /*dir*/, double /*rpm*/) override { return false; }
    bool stop() override { return false; }
    bool moveToRail(double /*rail_m*/, double /*timeout_s*/) override { return false; }
    bool moveToRailAsync(double /*rail_m*/) override { return false; }
    bool motionDone() const override { return true; }
    bool waitMotionDone(double /*timeout_s*/) override { return false; }
    bool moveToHome(double /*timeout_s*/) override { return false; }
    bool clearFault() override { return false; }
};

class GuidewayClientImpl : public GuidewayClient {
public:
    explicit GuidewayClientImpl(GuidewayConfig cfg, rclcpp::Logger logger);

    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;
    GuidewayStatus status() const override;
    GuidewayCalibration calibration() const override;
    bool isCalibrated() const override;
    bool isHomed() const override;
    GuidewaySafetyState safetyState() const override;
    double railPositionM() const override;
    double railVelocityMps() const override;
    bool calibrate() override;
    bool setVelocity(double rail_mps) override;
    bool jog(GuidewayDirection dir, double rpm) override;
    bool stop() override;
    bool moveToRail(double rail_m, double timeout_s) override;
    bool moveToRailAsync(double rail_m) override;
    bool motionDone() const override;
    bool waitMotionDone(double timeout_s) override;
    bool moveToHome(double timeout_s) override;
    bool clearFault() override;

private:
    /// Lazy connection: open serial ports on first use.
    bool ensureConnected();

    rclcpp::Logger logger_;
    std::unique_ptr<omr_hardware::RailController> rail_;
};

}  // namespace omr_controller
