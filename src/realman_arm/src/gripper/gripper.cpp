#include "core/arm_impl.hpp"

namespace rm {

// ══════════════════════════════════════════════
//  Gripper commands
// ══════════════════════════════════════════════

// ── setGripperRoute — set stroke range ──

void Arm::Impl::setGripperRoute(int min, int max) {
    if (!ensureConnected()) throw ArmError(-1, "Not connected to arm");
    int ret = rm_set_gripper_route(handle_, min, max);
    impl::check(ret, "rm_set_gripper_route");
}

// ── gripper — position control ──

void Arm::Impl::gripper(int position, bool blocking, int timeout) {
    if (!ensureConnected()) throw ArmError(-1, "Not connected to arm");

    if (blocking) {
        std::lock_guard lock(done_mutex_);
        motion_done_ = false;
        last_motion_ok_ = true;
    }

    {
        std::lock_guard lock(cmd_mutex_);
        cmd_queue_.push([this, position, blocking, timeout]() {
            int ret = rm_set_gripper_position(handle_, position, blocking, timeout);
            if (blocking) {
                last_motion_ok_ = (ret == 0);
                {
                    std::lock_guard lock(done_mutex_);
                    motion_done_ = true;
                }
                done_cv_.notify_one();
            }
        });
    }
    cmd_cv_.notify_one();

    if (blocking) {
        std::unique_lock lock(done_mutex_);
        done_cv_.wait(lock, [this] { return motion_done_.load(); });
        if (!last_motion_ok_) {
            throw ArmError(-1, "gripper failed");
        }
    }
}

// ── gripperRelease — open to max ──

void Arm::Impl::gripperRelease(int speed, bool blocking, int timeout) {
    if (!ensureConnected()) throw ArmError(-1, "Not connected to arm");

    if (blocking) {
        std::lock_guard lock(done_mutex_);
        motion_done_ = false;
        last_motion_ok_ = true;
    }

    {
        std::lock_guard lock(cmd_mutex_);
        cmd_queue_.push([this, speed, blocking, timeout]() {
            int ret = rm_set_gripper_release(handle_, speed, blocking, timeout);
            if (blocking) {
                last_motion_ok_ = (ret == 0);
                {
                    std::lock_guard lock(done_mutex_);
                    motion_done_ = true;
                }
                done_cv_.notify_one();
            }
        });
    }
    cmd_cv_.notify_one();

    if (blocking) {
        std::unique_lock lock(done_mutex_);
        done_cv_.wait(lock, [this] { return motion_done_.load(); });
        if (!last_motion_ok_) {
            throw ArmError(-1, "gripperRelease failed");
        }
    }
}

// ── gripperPick — force-controlled grasp ──

void Arm::Impl::gripperPick(int speed, int force, bool blocking, int timeout) {
    if (!ensureConnected()) throw ArmError(-1, "Not connected to arm");

    if (blocking) {
        std::lock_guard lock(done_mutex_);
        motion_done_ = false;
        last_motion_ok_ = true;
    }

    {
        std::lock_guard lock(cmd_mutex_);
        cmd_queue_.push([this, speed, force, blocking, timeout]() {
            int ret = rm_set_gripper_pick(handle_, speed, force, blocking, timeout);
            if (blocking) {
                last_motion_ok_ = (ret == 0);
                {
                    std::lock_guard lock(done_mutex_);
                    motion_done_ = true;
                }
                done_cv_.notify_one();
            }
        });
    }
    cmd_cv_.notify_one();

    if (blocking) {
        std::unique_lock lock(done_mutex_);
        done_cv_.wait(lock, [this] { return motion_done_.load(); });
        if (!last_motion_ok_) {
            throw ArmError(-1, "gripperPick failed");
        }
    }
}

// ── gripperPickOn — continuous force grasp ──

void Arm::Impl::gripperPickOn(int speed, int force, bool blocking, int timeout) {
    if (!ensureConnected()) throw ArmError(-1, "Not connected to arm");

    if (blocking) {
        std::lock_guard lock(done_mutex_);
        motion_done_ = false;
        last_motion_ok_ = true;
    }

    {
        std::lock_guard lock(cmd_mutex_);
        cmd_queue_.push([this, speed, force, blocking, timeout]() {
            int ret = rm_set_gripper_pick_on(handle_, speed, force, blocking, timeout);
            if (blocking) {
                last_motion_ok_ = (ret == 0);
                {
                    std::lock_guard lock(done_mutex_);
                    motion_done_ = true;
                }
                done_cv_.notify_one();
            }
        });
    }
    cmd_cv_.notify_one();

    if (blocking) {
        std::unique_lock lock(done_mutex_);
        done_cv_.wait(lock, [this] { return motion_done_.load(); });
        if (!last_motion_ok_) {
            throw ArmError(-1, "gripperPickOn failed");
        }
    }
}

// ── gripperState — read gripper status ──

GripperState Arm::Impl::gripperState() const {
    if (!handle_) throw ArmError(-1, "Not connected to arm");

    rm_gripper_state_t gs{};
    int ret = rm_get_gripper_state(handle_, &gs);
    if (ret != 0) {
        throw ArmError(ret, "rm_get_gripper_state failed");
    }

    GripperState state;
    state.enable_state  = gs.enable_state;
    state.status        = gs.status;
    state.error         = gs.error;
    state.mode          = gs.mode;
    state.current_force = gs.current_force;
    state.temperature   = gs.temperature;
    state.actpos        = gs.actpos;
    return state;
}

} // namespace rm
