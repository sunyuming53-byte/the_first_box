#include "core/arm_impl.hpp"
#include <cstring>

namespace rm {

// ──────────────────────────────────────────────
//  Constructor
// ──────────────────────────────────────────────

Arm::Impl::Impl(const ArmConfig& config)
    : ip_(config.ip), port_(config.tcp_port)
{
    int ret = rm_init(RM_TRIPLE_MODE_E);
    impl::check(ret, "rm_init");

    // Defer TCP connection to first command (lazy connect).
    // This allows ArmNode construction without arm hardware.

    running_ = true;
    worker_ = std::thread(&Impl::workerLoop, this);
}

// ──────────────────────────────────────────────
//  Destructor
// ──────────────────────────────────────────────

Arm::Impl::~Impl() {
    running_ = false;
    cmd_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    if (handle_) {
        rm_delete_robot_arm(handle_);
        handle_ = nullptr;
    }
}

// ──────────────────────────────────────────────
//  Lazy connection (called before first command)
// ──────────────────────────────────────────────

bool Arm::Impl::ensureConnected() {
    if (handle_) return true;

    handle_ = rm_create_robot_arm(ip_.c_str(), port_);
    if (handle_ && handle_->id >= 0) return true;

    // Connection failed — clean up and report
    if (handle_) {
        rm_delete_robot_arm(handle_);
        handle_ = nullptr;
    }
    return false;
}

bool Arm::Impl::isConnected() const {
    return handle_ != nullptr;
}

// ──────────────────────────────────────────────
//  Worker loop
// ──────────────────────────────────────────────

void Arm::Impl::workerLoop() {
    while (running_) {
        std::function<void()> cmd;
        {
            std::unique_lock lock(cmd_mutex_);
            cmd_cv_.wait(lock, [this] { return !cmd_queue_.empty() || !running_; });
            if (!running_ && cmd_queue_.empty()) break;
            if (cmd_queue_.empty()) continue;
            cmd = std::move(cmd_queue_.front());
            cmd_queue_.pop();
        }
        if (!ensureConnected()) {
            std::lock_guard lock(done_mutex_);
            last_motion_ok_ = false;
            motion_done_ = true;
            done_cv_.notify_one();
            continue;
        }
        cmd();
    }
}

} // namespace rm
