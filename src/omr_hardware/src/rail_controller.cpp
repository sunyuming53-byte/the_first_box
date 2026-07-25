// SPDX-License-Identifier: Apache-2.0
// omr_hardware::RailController implementation.
//
// Threading model (mirrors rm::Arm): one worker thread owns the photogate and
// motor exclusively. Public calls enqueue tasks and (optionally) wait on a
// future. When the queue is empty the worker keeps polling the photogate and
// enforcing the limit interlock, so safety does not depend on the caller.
//
// The interlock / calibration state machine is ported unchanged from the
// validated bring-up tool (test/rail_limit_homing_hw_test.cpp).

#include "omr_hardware/rail_controller.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <future>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

#include "dais/motor.hpp"
#include "photogate/photogate.hpp"

namespace omr_hardware {

namespace {

constexpr double PI = 3.14159265358979323846;

double rpmToRadS(double rpm) { return rpm * (2.0 * PI) / 60.0; }

const char* safetyName(RailSafetyState s) {
    switch (s) {
        case RailSafetyState::NORMAL:
            return "NORMAL";
        case RailSafetyState::UP_LIMITED:
            return "UP_LIMITED";
        case RailSafetyState::DOWN_LIMITED:
            return "DOWN_LIMITED";
        case RailSafetyState::SAFETY_FAULT:
            return "SAFETY_FAULT";
    }
    return "?";
}

void trimInplace(std::string& s) {
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) {
        s.pop_back();
    }
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) {
        ++i;
    }
    s.erase(0, i);
}

}  // namespace

class RailController::Impl {
public:
    Impl(RailControllerConfig cfg, RailLogFn log) : cfg_(std::move(cfg)), log_(std::move(log)) {
        worker_ = std::thread([this] { workerLoop(); });
    }

    ~Impl() {
        {
            std::lock_guard<std::mutex> lk(queueMutex_);
            running_ = false;
        }
        abort_.store(true);
        queueCv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
        // Worker is gone: safe to touch hardware from this thread.
        shutdownHardware();
    }

    // ── Public API (thread-safe) ─────────────────────────────────────────

    bool connect() {
        return runOnWorker([this] { return doConnect(); });
    }

    void disconnect() {
        abort_.store(true);
        runOnWorker([this] {
            abort_.store(false);
            doDisconnect();
            return true;
        });
    }

    bool isConnected() const { return connected_.load(); }

    RailStatus status() const {
        RailStatus st;
        {
            std::lock_guard<std::mutex> lk(stateMutex_);
            st = snapshot_;
        }
        {
            std::lock_guard<std::mutex> lk(motionMutex_);
            st.motion_done = motionDone_;
        }
        return st;
    }

    RailCalibration calibration() const {
        std::lock_guard<std::mutex> lk(stateMutex_);
        return calibPub_;
    }

    bool calibrate() {
        return runOnWorker([this] { return doCalibrate(); });
    }

    bool setVelocityMps(double railMps) {
        return runOnWorker([this, railMps] { return doSetVelocityMps(railMps); });
    }

    bool jog(RailDirection dir, double rpm) {
        return runOnWorker([this, dir, rpm] { return doJog(dir, rpm); });
    }

    bool stop() {
        // Abort any long-running operation first (loops poll the flag), then
        // let the worker actually stop the motor.
        abort_.store(true);
        return runOnWorker([this] {
            abort_.store(false);
            commandStop();
            if (motor_ && motor_->is_connected() && mode_ == dais::ControlMode::Position) {
                motor_->disable();
                enabled_ = false;
            }
            publishSnapshot();
            return true;
        });
    }

    bool moveToRail(double railM, double timeoutS) {
        if (!startMove(railM, timeoutS)) {
            return false;
        }
        return waitMotionDone(timeoutS + 10.0);
    }

    bool moveToRailAsync(double railM) { return startMove(railM, cfg_.return_timeout_s); }

    bool waitMotionDone(double timeoutS) {
        std::unique_lock<std::mutex> lk(motionMutex_);
        const bool done = motionCv_.wait_for(lk, std::chrono::duration<double>(timeoutS),
                                             [this] { return motionDone_; });
        return done && motionOk_;
    }

    bool moveToHome(double timeoutS) { return moveToRail(0.0, timeoutS); }

    bool clearFault() {
        return runOnWorker([this] { return doClearFault(); });
    }

private:
    // ── Logging ──────────────────────────────────────────────────────────

    void logf(RailLogLevel lvl, const char* fmt, ...) {
        char buf[512];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        if (log_) {
            log_(lvl, buf);
            return;
        }
        std::FILE* out = (lvl == RailLogLevel::INFO) ? stdout : stderr;
        fprintf(out, "[rail] %s\n", buf);
        fflush(out);
    }

    // ── Worker thread & task queue ───────────────────────────────────────

    void workerLoop() {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lk(queueMutex_);
                queueCv_.wait_for(lk, std::chrono::milliseconds(5),
                                  [this] { return !running_ || !tasks_.empty(); });
                if (!running_) {
                    // Abandon pending tasks: their promises break and waiting
                    // callers unblock with false.
                    tasks_.clear();
                    return;
                }
                if (!tasks_.empty()) {
                    task = std::move(tasks_.front());
                    tasks_.pop_front();
                }
            }
            if (task) {
                task();
            } else if (connected_.load()) {
                pollAndInterlock(/*checkMotor=*/true);
            }
        }
    }

    /// Run fn on the worker thread and wait for its result.
    bool runOnWorker(std::function<bool()> fn) {
        auto task = std::make_shared<std::packaged_task<bool()>>(std::move(fn));
        auto fut = task->get_future();
        {
            std::lock_guard<std::mutex> lk(queueMutex_);
            if (!running_) {
                return false;
            }
            tasks_.emplace_back([task] { (*task)(); });
        }
        queueCv_.notify_all();
        try {
            return fut.get();
        } catch (...) {
            return false;  // broken promise on shutdown
        }
    }

    /// Enqueue fn without waiting (async position moves).
    bool postToWorker(std::function<void()> fn) {
        {
            std::lock_guard<std::mutex> lk(queueMutex_);
            if (!running_) {
                return false;
            }
            tasks_.emplace_back(std::move(fn));
        }
        queueCv_.notify_all();
        return true;
    }

    // ── Everything below runs on the worker thread only ──────────────────

    // ── Connection ──

    bool doConnect() {
        if (connected_.load()) {
            return true;
        }
        photogate::PhotogateConfig pcfg;
        pcfg.serial_port = cfg_.photogate_port;
        pcfg.baud_rate = cfg_.photogate_baud;
        pcfg.gate_count = cfg_.gate_count;
        pg_ = std::make_unique<photogate::Photogate>(pcfg);
        if (!pg_->connect()) {
            logf(RailLogLevel::ERROR, "photogate connect failed: %s", cfg_.photogate_port.c_str());
            pg_.reset();
            return false;
        }
        if (!ensureMode(dais::ControlMode::Speed)) {
            pg_->disconnect();
            pg_.reset();
            return false;
        }
        safety_ = RailSafetyState::NORMAL;
        last_ = RailGates{};
        everValid_ = false;
        suppressLowerStop_ = false;
        connected_.store(true);
        logf(RailLogLevel::INFO, "connected: photogate=%s motor=%s",
             cfg_.photogate_port.c_str(), cfg_.motor_port.c_str());
        publishSnapshot();
        return true;
    }

    void doDisconnect() {
        shutdownHardware();
        connected_.store(false);
        publishSnapshot();
    }

    void shutdownHardware() {
        if (motor_ && motor_->is_connected()) {
            motor_->set_velocity_command(0.0);
            motor_->disable();
            motor_->disconnect();
        }
        motor_.reset();
        enabled_ = false;
        if (pg_ && pg_->is_connected()) {
            pg_->disconnect();
        }
        pg_.reset();
    }

    /// Recreate the motor when the requested control mode differs (the D-AIS
    /// H02_00 mode register is applied by configure_device at connect time).
    bool ensureMode(dais::ControlMode mode) {
        if (motor_ && motor_->is_connected() && mode_ == mode) {
            return true;
        }
        if (motor_ && motor_->is_connected()) {
            motor_->set_velocity_command(0.0);
            motor_->disable();
            motor_->disconnect();
        }
        dais::MotorConfig mcfg;
        mcfg.serial_port = cfg_.motor_port;
        mcfg.baud_rate = cfg_.motor_baud;
        mcfg.slave_id = cfg_.slave_id;
        mcfg.control_mode = mode;
        mcfg.position_max_rpm = cfg_.position_max_rpm;
        mcfg.position_accel_ms = cfg_.position_accel_ms;
        motor_ = std::make_unique<dais::Motor>(mcfg);
        if (!motor_->connect()) {
            logf(RailLogLevel::ERROR, "motor connect failed: %s", cfg_.motor_port.c_str());
            motor_.reset();
            return false;
        }
        mode_ = mode;
        enabled_ = false;
        return true;
    }

    // ── Gate reading & interlock (ported from bring-up tool) ──

    struct Snap {
        RailGates g{};
        bool fresh = false;
    };

    Snap readGates() {
        Snap snap;
        if (!pg_) {
            return snap;
        }
        snap.fresh = pg_->read_frames() > 0;
        if (cfg_.lower_gate < 0 || cfg_.home_gate < 0 || cfg_.upper_gate < 0 ||
            cfg_.lower_gate >= cfg_.gate_count || cfg_.home_gate >= cfg_.gate_count ||
            cfg_.upper_gate >= cfg_.gate_count) {
            return snap;
        }
        const auto lo = pg_->gate_state(cfg_.lower_gate);
        const auto hm = pg_->gate_state(cfg_.home_gate);
        const auto up = pg_->gate_state(cfg_.upper_gate);
        snap.g.lower = lo.blocked;
        snap.g.home = hm.blocked;
        snap.g.upper = up.blocked;
        snap.g.valid = (lo.host_rx_us != 0 && hm.host_rx_us != 0 && up.host_rx_us != 0);
        return snap;
    }

    bool waitForValidGates(double timeoutS) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeoutS));
        while (std::chrono::steady_clock::now() < deadline && !abort_.load()) {
            pollAndInterlock(/*checkMotor=*/false);
            if (everValid_) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return everValid_;
    }

    bool pollAndInterlock(bool checkMotor) {
        const auto now = std::chrono::steady_clock::now();
        Snap snap = readGates();

        if (!snap.g.valid || !snap.fresh) {
            if (everValid_) {
                const double ageMs =
                    std::chrono::duration<double, std::milli>(now - lastValidTime_).count();
                if (ageMs > cfg_.watchdog_ms) {
                    enterFault("photogate watchdog timeout");
                    publishSnapshot();
                    return false;
                }
            }
            publishSnapshot();
            return safety_ != RailSafetyState::SAFETY_FAULT;
        }

        everValid_ = true;
        lastValidTime_ = now;

        const bool lowerRise = !last_.lower && snap.g.lower;
        const bool upperRise = !last_.upper && snap.g.upper;
        last_ = snap.g;

        if (snap.g.lower && snap.g.upper) {
            enterFault("both lower and upper limits active");
            publishSnapshot();
            return false;
        }

        if (checkMotor && motor_ && motor_->is_connected() && motor_->has_error()) {
            enterFault("motor has_error()");
            publishSnapshot();
            return false;
        }

        if (safety_ == RailSafetyState::SAFETY_FAULT) {
            publishSnapshot();
            return false;
        }

        if (upperRise) {
            stopAndDisable();
            safety_ = RailSafetyState::UP_LIMITED;
            logf(RailLogLevel::WARN, "[interlock] upper rise -> UP_LIMITED");
        } else if (lowerRise && !suppressLowerStop_) {
            stopAndDisable();
            safety_ = RailSafetyState::DOWN_LIMITED;
            logf(RailLogLevel::WARN, "[interlock] lower rise -> DOWN_LIMITED");
        } else if (lowerRise && suppressLowerStop_) {
            logf(RailLogLevel::INFO, "[interlock] lower rise ignored (pass-through)");
        } else if (safety_ == RailSafetyState::UP_LIMITED && !snap.g.upper) {
            safety_ = RailSafetyState::NORMAL;
            logf(RailLogLevel::INFO, "[interlock] upper cleared -> NORMAL");
        } else if (safety_ == RailSafetyState::DOWN_LIMITED && !snap.g.lower) {
            safety_ = RailSafetyState::NORMAL;
            logf(RailLogLevel::INFO, "[interlock] lower cleared -> NORMAL");
        }
        publishSnapshot();
        return true;
    }

    double signedVelRadS(RailDirection dir, double rpm) const {
        const double mag = rpmToRadS(std::fabs(rpm));
        return (dir == RailDirection::UP) ? static_cast<double>(cfg_.up_sign) * mag
                                          : -static_cast<double>(cfg_.up_sign) * mag;
    }

    bool isUpCommand(double velRadS) const {
        return velRadS * static_cast<double>(cfg_.up_sign) > 1e-9;
    }

    bool isDownCommand(double velRadS) const {
        return velRadS * static_cast<double>(cfg_.up_sign) < -1e-9;
    }

    bool commandVelocity(RailDirection dir, double rpm) {
        if (!motor_ || !motor_->is_connected()) {
            logf(RailLogLevel::ERROR, "[reject] motor not connected");
            return false;
        }
        if (safety_ == RailSafetyState::SAFETY_FAULT) {
            logf(RailLogLevel::ERROR, "[reject] SAFETY_FAULT");
            return false;
        }
        const double vel = signedVelRadS(dir, rpm);
        if (safety_ == RailSafetyState::UP_LIMITED && isUpCommand(vel)) {
            logf(RailLogLevel::WARN, "[reject] UP_LIMITED — upward rejected");
            return false;
        }
        if (safety_ == RailSafetyState::DOWN_LIMITED && isDownCommand(vel) &&
            !suppressLowerStop_) {
            logf(RailLogLevel::WARN, "[reject] DOWN_LIMITED — downward rejected");
            return false;
        }
        if (!ensureEnabled()) {
            return false;
        }
        motor_->set_velocity_command(vel);
        return true;
    }

    void commandStop() {
        if (motor_ && motor_->is_connected()) {
            motor_->set_velocity_command(0.0);
        }
    }

    void stopAndDisable() {
        if (!motor_ || !motor_->is_connected()) {
            enabled_ = false;
            return;
        }
        motor_->set_velocity_command(0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!motor_->disable()) {
            logf(RailLogLevel::WARN, "motor disable returned false — servo may still be active");
        }
        enabled_ = false;
    }

    bool ensureEnabled() {
        if (!motor_ || !motor_->is_connected()) {
            return false;
        }
        if (safety_ == RailSafetyState::SAFETY_FAULT) {
            return false;
        }
        if (enabled_) {
            auto s = motor_->read_state();
            if (s.servo_state == 5 && !motor_->has_error()) {
                return true;
            }
            enabled_ = false;
        }
        motor_->disable();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!motor_->configure_device()) {
            enterFault("configure_device failed");
            return false;
        }
        if (!motor_->enable()) {
            enterFault("enable failed");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto s = motor_->read_state();
        if (s.servo_state != 5 || motor_->has_error()) {
            enterFault("servo not ready after enable");
            return false;
        }
        enabled_ = true;
        return true;
    }

    void enterFault(const char* why) {
        if (safety_ == RailSafetyState::SAFETY_FAULT) {
            return;
        }
        logf(RailLogLevel::ERROR, "[SAFETY_FAULT] %s", why);
        stopAndDisable();
        safety_ = RailSafetyState::SAFETY_FAULT;
        calib_.calibrated = false;
        calib_.homed = false;
    }

    /// After limit rise, interlock may leave us LIMITED; seed from absolute state.
    void syncLimitStateFromGates() {
        if (safety_ == RailSafetyState::SAFETY_FAULT) {
            return;
        }
        if (last_.lower && last_.upper) {
            enterFault("both limits active");
            return;
        }
        if (last_.upper) {
            safety_ = RailSafetyState::UP_LIMITED;
        } else if (last_.lower && !suppressLowerStop_) {
            safety_ = RailSafetyState::DOWN_LIMITED;
        } else {
            safety_ = RailSafetyState::NORMAL;
        }
    }

    // ── Edge-seek helpers (ported from bring-up tool) ──

    bool moveUntilEdge(RailDirection dir, double rpm, double timeoutS, bool wantRise,
                       bool RailGates::*field, const char* name) {
        logf(RailLogLevel::INFO, "[move] %s @ %.1f rpm, wait %s %s",
             dir == RailDirection::UP ? "UP" : "DOWN", rpm, name, wantRise ? "rise" : "fall");

        if (wantRise && (last_.*field)) {
            logf(RailLogLevel::INFO, "[move] already blocked on %s — treating rise as done", name);
            return true;
        }
        if (!wantRise && !(last_.*field)) {
            logf(RailLogLevel::INFO, "[move] already clear on %s — treating fall as done", name);
            return true;
        }

        if (!commandVelocity(dir, rpm)) {
            return false;
        }

        bool prev = last_.*field;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeoutS));
        while (std::chrono::steady_clock::now() < deadline && !abort_.load()) {
            if (!pollAndInterlock(/*checkMotor=*/true)) {
                commandStop();
                return false;
            }
            const bool cur = last_.*field;
            if ((wantRise && !prev && cur) || (!wantRise && prev && !cur)) {
                commandStop();
                std::this_thread::sleep_for(std::chrono::milliseconds(80));
                logf(RailLogLevel::INFO, "[edge] %s %s  pos=%.4f rad", name,
                     wantRise ? "rise" : "fall", motor_->read_state().position_rad);
                return true;
            }
            // Expected: seeking lower down ends in DOWN_LIMITED after rise — still success
            if (wantRise && cur && field == &RailGates::lower && dir == RailDirection::DOWN) {
                commandStop();
                logf(RailLogLevel::INFO, "[edge] lower rise (via limit interlock)");
                return true;
            }
            if (wantRise && cur && field == &RailGates::upper && dir == RailDirection::UP) {
                commandStop();
                logf(RailLogLevel::INFO, "[edge] upper rise (via limit interlock)");
                return true;
            }
            prev = cur;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        commandStop();
        logf(RailLogLevel::ERROR, "[timeout] %s %s", name, wantRise ? "rise" : "fall");
        return false;
    }

    bool leaveSensor(RailDirection dir, double rpm, double timeoutS, bool RailGates::*field,
                     const char* name) {
        if (!(last_.*field)) {
            return true;
        }
        return moveUntilEdge(dir, rpm, timeoutS, /*wantRise=*/false, field, name);
    }

    /// RAII: lower-limit stop is suppressed only for the duration of blade pass-through.
    struct ScopedSuppressLowerStop {
        explicit ScopedSuppressLowerStop(Impl& impl) : impl_(impl) {
            impl_.suppressLowerStop_ = true;
        }
        ~ScopedSuppressLowerStop() { impl_.suppressLowerStop_ = false; }
        ScopedSuppressLowerStop(const ScopedSuppressLowerStop&) = delete;
        ScopedSuppressLowerStop& operator=(const ScopedSuppressLowerStop&) = delete;

    private:
        Impl& impl_;
    };

    /// Drive DOWN through the lower photogate: on block do NOT stop; stop on clear
    /// (blade fully past). Lower interlock is suppressed only inside this function.
    bool passLowerBladeDown(double rpm, double timeoutS) {
        logf(RailLogLevel::INFO,
             "[pass] DOWN @ %.1f rpm — wait full blade past lower (rise then fall)", rpm);
        ScopedSuppressLowerStop guard(*this);
        bool sawBlocked = last_.lower;
        if (!commandVelocity(RailDirection::DOWN, rpm)) {
            return false;
        }

        bool prev = last_.lower;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(timeoutS));
        while (std::chrono::steady_clock::now() < deadline && !abort_.load()) {
            if (!pollAndInterlock(/*checkMotor=*/true)) {
                commandStop();
                return false;
            }
            const bool cur = last_.lower;
            if (!prev && cur) {
                sawBlocked = true;
                logf(RailLogLevel::INFO, "[pass] lower blocked — continue until blade clears");
            }
            if (sawBlocked && prev && !cur) {
                commandStop();
                std::this_thread::sleep_for(std::chrono::milliseconds(80));
                logf(RailLogLevel::INFO, "[pass] lower cleared (blade fully past) pos=%.4f rad",
                     motor_->read_state().position_rad);
                // Restore normal lower interlock before leaving (guard dtor also clears).
                suppressLowerStop_ = false;
                syncLimitStateFromGates();
                return true;
            }
            prev = cur;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        commandStop();
        logf(RailLogLevel::ERROR, "[timeout] pass lower blade down");
        return false;
    }

    std::optional<double> latchPosition() {
        commandStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        auto s = motor_->read_state();
        if (s.fault_code || s.comm_error || motor_->has_error()) {
            return std::nullopt;
        }
        return s.position_rad;
    }

    // ── Calibration (ported from bring-up tool) ──

    bool doCalibrate() {
        if (!connected_.load() && !doConnect()) {
            return false;
        }
        logf(RailLogLevel::INFO,
             "=== full-travel calibration (lower: pass-through + enter edge; "
             "home/upper: enter edge) ===");
        calib_ = RailCalibration{};
        suppressLowerStop_ = false;
        publishSnapshot();

        if (!ensureMode(dais::ControlMode::Speed)) {
            return false;
        }
        if (!waitForValidGates(5.0)) {
            logf(RailLogLevel::ERROR, "precheck failed: no photogate data");
            return false;
        }
        if (!pollAndInterlock(/*checkMotor=*/true)) {
            return false;
        }
        syncLimitStateFromGates();
        if (safety_ == RailSafetyState::SAFETY_FAULT) {
            return false;
        }

        const double T = cfg_.seek_timeout_s;
        const double coarse = cfg_.coarse_rpm;
        const double fine = cfg_.fine_rpm;
        const double backoff = cfg_.backoff_rpm;

        logf(RailLogLevel::INFO, "[2] pass lower blade downward (no stop on block)");
        // If already sitting in/on lower, back off upward first so we approach from above.
        if (last_.lower) {
            if (!leaveSensor(RailDirection::UP, backoff, T, &RailGates::lower, "lower")) {
                enterFault("failed to leave lower before pass-through");
                return false;
            }
            commandVelocity(RailDirection::UP, backoff);
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            commandStop();
            pollAndInterlock(true);
            syncLimitStateFromGates();
        }
        if (!passLowerBladeDown(coarse, T)) {
            enterFault("failed lower blade pass-through");
            return false;
        }
        // Small extra downward settle past the clear edge.
        commandVelocity(RailDirection::DOWN, fine);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        commandStop();
        pollAndInterlock(true);
        syncLimitStateFromGates();
        logf(RailLogLevel::INFO, "blade fully below lower gate; latch enter edge going up");

        logf(RailLogLevel::INFO, "[3] latch lower (UP, lower rise = blade top edge)");
        if (!moveUntilEdge(RailDirection::UP, fine, T, true, &RailGates::lower, "lower")) {
            enterFault("failed lower upper-edge rise latch");
            return false;
        }
        auto thetaLower = latchPosition();
        if (!thetaLower) {
            enterFault("read theta_lower failed");
            return false;
        }
        logf(RailLogLevel::INFO, "theta_lower = %.6f rad", *thetaLower);

        logf(RailLogLevel::INFO, "[4] latch home");
        if (!last_.home) {
            if (!moveUntilEdge(RailDirection::UP, coarse, T, true, &RailGates::home, "home")) {
                enterFault("failed coarse home rise");
                return false;
            }
        }
        if (!leaveSensor(RailDirection::DOWN, backoff, T, &RailGates::home, "home")) {
            enterFault("failed home backoff");
            return false;
        }
        commandVelocity(RailDirection::DOWN, backoff);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        commandStop();
        pollAndInterlock(true);
        syncLimitStateFromGates();

        if (!moveUntilEdge(RailDirection::UP, fine, T, true, &RailGates::home, "home")) {
            enterFault("failed fine home rise");
            return false;
        }
        auto thetaHome = latchPosition();
        if (!thetaHome) {
            enterFault("read theta_home failed");
            return false;
        }
        logf(RailLogLevel::INFO, "theta_home = %.6f rad", *thetaHome);

        logf(RailLogLevel::INFO, "[5] latch upper");
        if (!last_.upper) {
            if (!moveUntilEdge(RailDirection::UP, coarse, T, true, &RailGates::upper, "upper")) {
                if (!last_.upper) {
                    enterFault("failed coarse upper rise");
                    return false;
                }
            }
        }
        syncLimitStateFromGates();
        if (!leaveSensor(RailDirection::DOWN, backoff, T, &RailGates::upper, "upper")) {
            enterFault("failed upper backoff");
            return false;
        }
        commandVelocity(RailDirection::DOWN, backoff);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        commandStop();
        pollAndInterlock(true);
        syncLimitStateFromGates();

        if (!moveUntilEdge(RailDirection::UP, fine, T, true, &RailGates::upper, "upper")) {
            enterFault("failed fine upper rise");
            return false;
        }
        auto thetaUpper = latchPosition();
        if (!thetaUpper) {
            enterFault("read theta_upper failed");
            return false;
        }
        logf(RailLogLevel::INFO, "theta_upper = %.6f rad", *thetaUpper);

        logf(RailLogLevel::INFO, "[6] validate three-point geometry");
        const double us = static_cast<double>(cfg_.up_sign);
        const double dHl = us * (*thetaHome - *thetaLower);
        const double dUh = us * (*thetaUpper - *thetaHome);
        if (!(dHl > 0.0 && dUh > 0.0)) {
            logf(RailLogLevel::ERROR, "order invalid: home-lower=%.6f upper-home=%.6f", dHl, dUh);
            enterFault("calibration geometry invalid");
            return false;
        }

        RailCalibration r;
        r.theta_lower = *thetaLower;
        r.theta_home = *thetaHome;
        r.theta_upper = *thetaUpper;
        r.lower_travel_rad = dHl;
        r.upper_travel_rad = dUh;
        r.total_travel_rad = dHl + dUh;
        const double lead = cfg_.screw_lead_m;
        r.lower_travel_m = r.lower_travel_rad * lead / (2.0 * PI);
        r.upper_travel_m = r.upper_travel_rad * lead / (2.0 * PI);
        r.total_travel_m = r.total_travel_rad * lead / (2.0 * PI);
        logf(RailLogLevel::INFO, "lower_travel = %.6f rad (%.4f m)", r.lower_travel_rad,
             r.lower_travel_m);
        logf(RailLogLevel::INFO, "upper_travel = %.6f rad (%.4f m)", r.upper_travel_rad,
             r.upper_travel_m);
        logf(RailLogLevel::INFO, "total_travel = %.6f rad (%.4f m)", r.total_travel_rad,
             r.total_travel_m);

        logf(RailLogLevel::INFO, "[8] closed-loop return to theta_home");
        if (!leaveSensor(RailDirection::DOWN, backoff, T, &RailGates::upper, "upper")) {
            enterFault("failed leave upper before return");
            return false;
        }
        stopAndDisable();

        if (!doMoveToTheta(r.theta_home, cfg_.return_timeout_s, cfg_.home_tol_rad)) {
            logf(RailLogLevel::ERROR, "return to home failed");
            return false;
        }

        r.calibrated = true;
        r.homed = true;
        calib_ = r;
        publishSnapshot();
        logf(RailLogLevel::INFO, "=== calibration OK: calibrated=1 homed=1 rail position=0 ===");
        return true;
    }

    // ── Position loop ──

    bool doMoveToTheta(double thetaTarget, double timeoutS, double tolRad) {
        if (!ensureMode(dais::ControlMode::Position)) {
            return false;
        }
        if (!ensureEnabled()) {
            logf(RailLogLevel::ERROR, "position-mode enable failed");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Direction precheck against active limits (moving away is allowed).
        const auto s0 = motor_->read_state();
        const double towardUp = static_cast<double>(cfg_.up_sign) * (thetaTarget - s0.position_rad);
        if (safety_ == RailSafetyState::SAFETY_FAULT) {
            logf(RailLogLevel::ERROR, "[reject] SAFETY_FAULT");
            motor_->disable();
            enabled_ = false;
            return false;
        }
        if (safety_ == RailSafetyState::UP_LIMITED && towardUp > 0.0) {
            logf(RailLogLevel::WARN, "[reject] UP_LIMITED — upward position move rejected");
            motor_->disable();
            enabled_ = false;
            return false;
        }
        if (safety_ == RailSafetyState::DOWN_LIMITED && towardUp < 0.0) {
            logf(RailLogLevel::WARN, "[reject] DOWN_LIMITED — downward position move rejected");
            motor_->disable();
            enabled_ = false;
            return false;
        }

        motor_->set_position_command(thetaTarget);

        const auto t0 = std::chrono::steady_clock::now();
        bool ok = false;
        while (!abort_.load()) {
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (elapsed > timeoutS) {
                break;
            }
            if (!pollAndInterlock(/*checkMotor=*/true)) {
                motor_->disable();
                enabled_ = false;
                return false;
            }
            // Interlock intervened (limit rise -> stop_and_disable clears enabled_).
            if (!enabled_) {
                logf(RailLogLevel::WARN, "position move interrupted by limit interlock");
                return false;
            }
            auto s = motor_->read_state();
            if (s.fault_code || s.comm_error || motor_->has_error()) {
                logf(RailLogLevel::ERROR, "motor fault during position move");
                motor_->disable();
                enabled_ = false;
                return false;
            }
            if (std::fabs(s.position_rad - thetaTarget) <= tolRad) {
                ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        motor_->disable();
        enabled_ = false;
        publishSnapshot();
        if (!ok) {
            logf(RailLogLevel::ERROR, abort_.load() ? "position move aborted"
                                                    : "position move timeout");
        }
        return ok;
    }

    bool doMoveToRail(double railM, double timeoutS) {
        if (!connected_.load() && !doConnect()) {
            return false;
        }
        if (!calib_.calibrated) {
            logf(RailLogLevel::ERROR, "[reject] not calibrated — position command refused");
            return false;
        }
        const double lo = -calib_.lower_travel_m + cfg_.soft_limit_margin_m;
        const double hi = calib_.upper_travel_m - cfg_.soft_limit_margin_m;
        if (railM < lo || railM > hi) {
            logf(RailLogLevel::ERROR, "[reject] target %.4f m outside soft limits [%.4f, %.4f]",
                 railM, lo, hi);
            return false;
        }
        return doMoveToTheta(thetaFromRail(railM), timeoutS, cfg_.home_tol_rad);
    }

    /// Common entry for blocking/async rail moves: marks motion in flight and
    /// posts the move to the worker.
    bool startMove(double railM, double timeoutS) {
        {
            std::lock_guard<std::mutex> lk(motionMutex_);
            if (!motionDone_) {
                logf(RailLogLevel::WARN, "[reject] another position move is in flight");
                return false;
            }
            motionDone_ = false;
            motionOk_ = false;
        }
        const bool queued = postToWorker([this, railM, timeoutS] {
            const bool ok = doMoveToRail(railM, timeoutS);
            {
                std::lock_guard<std::mutex> lk(motionMutex_);
                motionDone_ = true;
                motionOk_ = ok;
            }
            motionCv_.notify_all();
        });
        if (!queued) {
            std::lock_guard<std::mutex> lk(motionMutex_);
            motionDone_ = true;
            motionOk_ = false;
            return false;
        }
        return true;
    }

    // ── Velocity loop ──

    bool doSetVelocityMps(double railMps) {
        if (!connected_.load() && !doConnect()) {
            return false;
        }
        if (railMps == 0.0) {
            commandStop();
            return true;
        }
        if (!ensureMode(dais::ControlMode::Speed)) {
            return false;
        }
        const RailDirection dir = (railMps > 0.0) ? RailDirection::UP : RailDirection::DOWN;
        const double rpm = std::fabs(railMps) * 60.0 / cfg_.screw_lead_m;
        return commandVelocity(dir, rpm);
    }

    bool doJog(RailDirection dir, double rpm) {
        if (!connected_.load() && !doConnect()) {
            return false;
        }
        if (!ensureMode(dais::ControlMode::Speed)) {
            return false;
        }
        return commandVelocity(dir, rpm);
    }

    // ── Fault recovery ──

    bool doClearFault() {
        if (!connected_.load()) {
            return false;
        }
        if (safety_ != RailSafetyState::SAFETY_FAULT) {
            return true;
        }
        // Require fresh gate data before leaving the fault state.
        everValid_ = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline && !abort_.load()) {
            Snap snap = readGates();
            if (snap.g.valid && snap.fresh) {
                last_ = snap.g;
                everValid_ = true;
                lastValidTime_ = std::chrono::steady_clock::now();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!everValid_) {
            logf(RailLogLevel::ERROR, "clearFault refused: no fresh photogate data");
            return false;
        }
        safety_ = RailSafetyState::NORMAL;
        syncLimitStateFromGates();
        publishSnapshot();
        logf(RailLogLevel::WARN, "SAFETY_FAULT cleared -> %s (calibration must be redone)",
             safetyName(safety_));
        return safety_ != RailSafetyState::SAFETY_FAULT;
    }

    // ── Rail coordinate conversion (valid when calibrated) ──

    double railFromTheta(double theta) const {
        return static_cast<double>(cfg_.up_sign) * (theta - calib_.theta_home) *
               cfg_.screw_lead_m / (2.0 * PI);
    }

    double thetaFromRail(double railM) const {
        return calib_.theta_home +
               static_cast<double>(cfg_.up_sign) * railM * (2.0 * PI) / cfg_.screw_lead_m;
    }

    // ── Status snapshot (published by worker, read by any thread) ──

    void publishSnapshot() {
        RailStatus st;
        st.connected = connected_.load();
        st.gates = last_;
        st.safety = safety_;
        st.calibrated = calib_.calibrated;
        st.homed = calib_.homed;
        if (motor_ && motor_->is_connected()) {
            const auto s = motor_->read_state();
            st.motor_position_rad = s.position_rad;
            st.motor_velocity_rpm = s.velocity_rpm;
            if (calib_.calibrated) {
                st.rail_position_m = railFromTheta(s.position_rad);
                st.rail_velocity_mps = static_cast<double>(cfg_.up_sign) * s.velocity_rpm *
                                       cfg_.screw_lead_m / 60.0;
            }
        }
        std::lock_guard<std::mutex> lk(stateMutex_);
        snapshot_ = st;
        calibPub_ = calib_;
    }

    // ── Members ──────────────────────────────────────────────────────────

    RailControllerConfig cfg_;
    RailLogFn log_;

    // Worker & queue
    std::thread worker_;
    std::mutex queueMutex_;
    std::condition_variable queueCv_;
    std::deque<std::function<void()>> tasks_;
    bool running_ = true;
    std::atomic<bool> abort_{false};

    // Hardware (worker thread only)
    std::unique_ptr<photogate::Photogate> pg_;
    std::unique_ptr<dais::Motor> motor_;
    dais::ControlMode mode_ = dais::ControlMode::Speed;
    bool enabled_ = false;

    // Interlock state (worker thread only)
    RailSafetyState safety_ = RailSafetyState::NORMAL;
    RailGates last_{};
    bool everValid_ = false;
    std::chrono::steady_clock::time_point lastValidTime_{};
    bool suppressLowerStop_ = false;
    RailCalibration calib_{};

    // Published state (any thread)
    std::atomic<bool> connected_{false};
    mutable std::mutex stateMutex_;
    RailStatus snapshot_{};
    RailCalibration calibPub_{};

    // Async motion tracking
    mutable std::mutex motionMutex_;
    std::condition_variable motionCv_;
    bool motionDone_ = true;
    bool motionOk_ = true;
};

// ── RailController facade ────────────────────────────────────────────────

RailController::RailController(RailControllerConfig cfg, RailLogFn log)
    : impl_(std::make_unique<Impl>(std::move(cfg), std::move(log))) {}

RailController::~RailController() = default;

bool RailController::connect() { return impl_->connect(); }
void RailController::disconnect() { impl_->disconnect(); }
bool RailController::isConnected() const { return impl_->isConnected(); }
RailStatus RailController::status() const { return impl_->status(); }
RailCalibration RailController::calibration() const { return impl_->calibration(); }
bool RailController::calibrate() { return impl_->calibrate(); }
bool RailController::setVelocityMps(double rail_mps) { return impl_->setVelocityMps(rail_mps); }
bool RailController::jog(RailDirection dir, double rpm) { return impl_->jog(dir, rpm); }
bool RailController::stop() { return impl_->stop(); }
bool RailController::moveToRail(double rail_m, double timeout_s) {
    return impl_->moveToRail(rail_m, timeout_s);
}
bool RailController::moveToRailAsync(double rail_m) { return impl_->moveToRailAsync(rail_m); }
bool RailController::waitMotionDone(double timeout_s) { return impl_->waitMotionDone(timeout_s); }
bool RailController::moveToHome(double timeout_s) { return impl_->moveToHome(timeout_s); }
bool RailController::clearFault() { return impl_->clearFault(); }

// ── Flat YAML config loader ──────────────────────────────────────────────

bool loadRailConfig(const std::string& path, RailControllerConfig& cfg, std::string* error) {
    std::ifstream in(path);
    if (!in) {
        if (error) {
            *error = "cannot open config: " + path;
        }
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        trimInplace(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            continue;
        }
        std::string key = line.substr(0, colon);
        std::string val = line.substr(colon + 1);
        trimInplace(key);
        trimInplace(val);
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
            val = val.substr(1, val.size() - 2);
        }
        try {
            if (key == "photogate_port") {
                cfg.photogate_port = val;
            } else if (key == "photogate_baud") {
                cfg.photogate_baud = std::stoi(val);
            } else if (key == "gate_count") {
                cfg.gate_count = std::stoi(val);
                if (cfg.gate_count <= 0) {
                    if (error) {
                        *error = "gate_count must be > 0, got " + val;
                    }
                    return false;
                }
            } else if (key == "lower_gate") {
                cfg.lower_gate = std::stoi(val);
            } else if (key == "home_gate") {
                cfg.home_gate = std::stoi(val);
            } else if (key == "upper_gate") {
                cfg.upper_gate = std::stoi(val);
            } else if (key == "motor_port") {
                cfg.motor_port = val;
            } else if (key == "motor_baud") {
                cfg.motor_baud = std::stoi(val);
            } else if (key == "slave_id") {
                cfg.slave_id = std::stoi(val);
            } else if (key == "up_sign") {
                cfg.up_sign = std::stoi(val);
            } else if (key == "screw_lead_m") {
                cfg.screw_lead_m = std::stod(val);
            } else if (key == "watchdog_ms") {
                cfg.watchdog_ms = std::stod(val);
            } else if (key == "coarse_rpm") {
                cfg.coarse_rpm = std::stod(val);
            } else if (key == "fine_rpm") {
                cfg.fine_rpm = std::stod(val);
            } else if (key == "backoff_rpm") {
                cfg.backoff_rpm = std::stod(val);
            } else if (key == "seek_timeout_s") {
                cfg.seek_timeout_s = std::stod(val);
            } else if (key == "home_tol_rad") {
                cfg.home_tol_rad = std::stod(val);
            } else if (key == "return_timeout_s") {
                cfg.return_timeout_s = std::stod(val);
            } else if (key == "position_max_rpm") {
                cfg.position_max_rpm = static_cast<uint16_t>(std::stoi(val));
            } else if (key == "position_accel_ms") {
                cfg.position_accel_ms = static_cast<uint16_t>(std::stoi(val));
            } else if (key == "soft_limit_margin_m") {
                cfg.soft_limit_margin_m = std::stod(val);
            }
        } catch (...) {
            if (error) {
                *error = "config parse failed: " + key + ": " + val;
            }
            return false;
        }
    }
    if (cfg.up_sign != 1 && cfg.up_sign != -1) {
        if (error) {
            *error = "up_sign must be +1 or -1, got " + std::to_string(cfg.up_sign);
        }
        return false;
    }
    return true;
}

}  // namespace omr_hardware
