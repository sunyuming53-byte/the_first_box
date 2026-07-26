// SPDX-License-Identifier: Apache-2.0
// omr_hardware::RailController — full guideway (rail) motion control library.
//
// Wraps photogate::Photogate (3-gate limit/home sensing) + dais::Motor
// (Modbus RTU servo) into one thread-safe controller:
//   - limit interlock with photogate watchdog (background safety thread)
//   - full-travel calibration (lower blade pass-through + enter-edge latching)
//   - velocity-loop motion in rail coordinates (interlock protected)
//   - position-loop motion to absolute rail coordinates (soft-limit checked)
//
// Zero ROS dependencies. All commands are serialized through one worker
// thread; the same thread continuously polls the photogate and enforces the
// limit interlock while idle, so safety is never gated on the caller.
//
// Rail coordinate convention: meters, 0 at the calibrated home gate edge,
// positive = up. Conversion uses screw_lead_m and up_sign from config.
#pragma once

#include <cstdint>

#include <functional>
#include <memory>
#include <string>

namespace omr_hardware {

enum class RailSafetyState {
    NORMAL,        ///< free to move both directions
    UP_LIMITED,    ///< upper photogate tripped — upward motion rejected
    DOWN_LIMITED,  ///< lower photogate tripped — downward motion rejected
    SAFETY_FAULT,  ///< watchdog/motor/geometry fault — all motion rejected
};

enum class RailDirection { UP, DOWN };

enum class RailLogLevel { INFO, WARN, ERROR };

/// Optional log sink (e.g. bridge to rclcpp logger). Default: stderr/stdout.
using RailLogFn = std::function<void(RailLogLevel, const std::string&)>;

/// Fields map 1:1 to config/rail_photogate.yaml (flat keys).
struct RailControllerConfig {
    // photogate
    std::string photogate_port = "/dev/ttyACM0";
    int photogate_baud = 115200;
    int gate_count = 3;
    int lower_gate = 0;
    int home_gate = 1;
    int upper_gate = 2;
    // motor
    std::string motor_port = "/dev/ttyUSB0";
    int motor_baud = 57600;
    int slave_id = 1;
    // kinematics
    int up_sign = -1;  ///< +1/-1: motor direction that moves the rail up
    double screw_lead_m = 0.01;
    // safety & motion profile
    double watchdog_ms = 200.0;
    double coarse_rpm = 150.0;
    double fine_rpm = 3.0;
    double backoff_rpm = 10.0;
    double seek_timeout_s = 120.0;
    double home_tol_rad = 0.05;
    double return_timeout_s = 60.0;
    uint16_t position_max_rpm = 60;
    uint16_t position_accel_ms = 100;
    /// Extra margin kept away from calibrated travel ends for position moves.
    double soft_limit_margin_m = 0.0;
};

struct RailGates {
    bool lower = false;
    bool home = false;
    bool upper = false;
    bool valid = false;  ///< all three gates have reported at least one frame
};

struct RailCalibration {
    bool calibrated = false;
    bool homed = false;
    double theta_lower = 0.0;  ///< motor angle (rad) at lower gate enter edge
    double theta_home = 0.0;   ///< motor angle (rad) at home gate enter edge
    double theta_upper = 0.0;  ///< motor angle (rad) at upper gate enter edge
    double lower_travel_rad = 0.0;
    double upper_travel_rad = 0.0;
    double total_travel_rad = 0.0;
    double lower_travel_m = 0.0;
    double upper_travel_m = 0.0;
    double total_travel_m = 0.0;
};

/// Consistent snapshot for polling from an external framework (e.g. BT tick).
struct RailStatus {
    bool connected = false;
    RailGates gates{};
    RailSafetyState safety = RailSafetyState::NORMAL;
    bool calibrated = false;
    bool homed = false;
    bool motion_done = true;  ///< no async position move in flight
    double motor_position_rad = 0.0;
    double motor_velocity_rpm = 0.0;
    double rail_position_m = 0.0;    ///< valid only when calibrated
    double rail_velocity_mps = 0.0;  ///< up positive
};

class RailController {
public:
    explicit RailController(RailControllerConfig cfg, RailLogFn log = {});
    ~RailController();

    // Non-copyable, non-movable (owns worker thread + serial resources)
    RailController(const RailController&) = delete;
    RailController& operator=(const RailController&) = delete;
    RailController(RailController&&) = delete;
    RailController& operator=(RailController&&) = delete;

    // ── Connection ──
    /// Open photogate + motor serial ports and start the safety thread poll.
    bool connect();
    /// Stop motion, disable servo, close both ports.
    void disconnect();
    [[nodiscard]] bool isConnected() const;

    // ── Status (thread-safe, non-blocking) ──
    [[nodiscard]] RailStatus status() const;
    [[nodiscard]] RailCalibration calibration() const;

    // ── Calibration ──
    /// Full-travel calibration: latch lower/home/upper enter edges, validate
    /// geometry, then closed-loop return to home. Blocking (can take minutes).
    bool calibrate();

    // ── Velocity loop (interlock protected; works without calibration) ──
    /// Rail velocity in m/s, up positive; 0 stops. Rejected toward an active
    /// limit or in SAFETY_FAULT.
    bool setVelocityMps(double rail_mps);
    /// Low-level jog at motor rpm in a rail direction (debug/manual use).
    bool jog(RailDirection dir, double rpm);
    /// Stop motion now; also aborts a blocking/async operation in progress.
    bool stop();

    // ── Position loop (requires calibration; soft-limit checked) ──
    /// Move to absolute rail coordinate (m, 0 = home, up positive). Blocking.
    bool moveToRail(double rail_m, double timeout_s);
    /// Non-blocking variant; poll status().motion_done or waitMotionDone().
    bool moveToRailAsync(double rail_m);
    /// Wait for the async move to finish. Returns move success.
    bool waitMotionDone(double timeout_s);
    bool moveToHome(double timeout_s);

    // ── Recovery ──
    /// Leave SAFETY_FAULT if gate data is valid again; re-seeds limit state.
    bool clearFault();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/// Parse the flat-YAML rail config (config/rail_photogate.yaml). Unknown keys
/// are ignored; on failure returns false and fills *error.
bool loadRailConfig(const std::string& path, RailControllerConfig& cfg, std::string* error);

}  // namespace omr_hardware
