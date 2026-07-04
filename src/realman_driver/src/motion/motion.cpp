#include "core/arm_impl.hpp"
#include <vector>

namespace rm {

// ══════════════════════════════════════════════
//  Motion commands
// ══════════════════════════════════════════════

// ── moveJ — joint space motion to joint targets ──

void Arm::Impl::moveJ(const JointPosition& target, SpeedRatio speed,
                      bool blocking, int tc) {
    // Reject values that look like degrees (joint range is ±π rad ≈ ±180°)
    for (size_t i = 0; i < target.radians.size(); ++i) {
        if (std::abs(target.radians[i]) > 2.0 * M_PI) {
            throw ArmError(-1,
                "Joint " + std::to_string(i) + " value " + std::to_string(target.radians[i])
                + " looks like degrees. This API expects radians (max ±π).");
        }
    }

    // Convert radians to degrees
    std::vector<float> joints_deg;
    for (double r : target.radians) {
        joints_deg.push_back(static_cast<float>(r * 180.0 / M_PI));
    }
    joints_deg.resize(ARM_DOF, 0.0f);

    int block_flag = blocking ? 1 : 0;
    if (blocking) {
        std::lock_guard lock(done_mutex_);
        motion_done_ = false;
        last_motion_ok_ = true;
    }

    {
        std::lock_guard lock(cmd_mutex_);
        cmd_queue_.push([this, j = std::move(joints_deg), speed, tc, block_flag]() {
            int ret = rm_movej(handle_, j.data(), static_cast<int>(speed),
                               0, tc, block_flag);
            if (block_flag) {
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
            throw ArmError(-1, "moveJ failed");
        }
    }
}

// ── moveJ_P — joint space motion to Cartesian pose ──

void Arm::Impl::moveJ_P(const CartesianPose& target, SpeedRatio speed,
                        bool blocking, int tc) {
    rm_pose_t pose = impl::toRmPose(target);

    int block_flag = blocking ? 1 : 0;
    if (blocking) {
        std::lock_guard lock(done_mutex_);
        motion_done_ = false;
        last_motion_ok_ = true;
    }

    {
        std::lock_guard lock(cmd_mutex_);
        cmd_queue_.push([this, pose, speed, tc, block_flag]() {
            int ret = rm_movej_p(handle_, pose, static_cast<int>(speed),
                                 0, tc, block_flag);
            if (block_flag) {
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
            throw ArmError(-1, "moveJ_P failed");
        }
    }
}

// ── moveL — Cartesian linear motion ──

void Arm::Impl::moveL(const CartesianPose& target, SpeedRatio speed,
                      bool blocking, int tc) {
    rm_pose_t pose = impl::toRmPose(target);

    int block_flag = blocking ? 1 : 0;
    if (blocking) {
        std::lock_guard lock(done_mutex_);
        motion_done_ = false;
        last_motion_ok_ = true;
    }

    {
        std::lock_guard lock(cmd_mutex_);
        cmd_queue_.push([this, pose, speed, tc, block_flag]() {
            int ret = rm_movel(handle_, pose, static_cast<int>(speed),
                               0, tc, block_flag);
            if (block_flag) {
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
            throw ArmError(-1, "moveL failed");
        }
    }
}

// ── moveC — Cartesian arc motion ──

void Arm::Impl::moveC(const CartesianPose& mid, const CartesianPose& end,
                      SpeedRatio speed, int loop, bool blocking) {
    rm_pose_t pose_via = impl::toRmPose(mid);
    rm_pose_t pose_to  = impl::toRmPose(end);

    int block_flag = blocking ? 1 : 0;
    if (blocking) {
        std::lock_guard lock(done_mutex_);
        motion_done_ = false;
        last_motion_ok_ = true;
    }

    {
        std::lock_guard lock(cmd_mutex_);
        cmd_queue_.push([this, pose_via, pose_to, speed, loop, block_flag]() {
            int ret = rm_movec(handle_, pose_via, pose_to,
                               static_cast<int>(speed), 0, loop, 0, block_flag);
            if (block_flag) {
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
            throw ArmError(-1, "moveC failed");
        }
    }
}

// ── stop — emergency stop ──

void Arm::Impl::stop() {
    if (!ensureConnected()) throw ArmError(-1, "Not connected to arm");
    int ret = rm_set_arm_stop(handle_);
    impl::check(ret, "rm_set_arm_stop");
}

// ── V1 stubs — not implemented ──

void Arm::Impl::moveJ_CANFD(const JointPosition& /*target*/, int /*mode*/) {
    throw ArmError(-1, "moveJ_CANFD not implemented in V1");
}

void Arm::Impl::moveP_CANFD(const CartesianPose& /*target*/, int /*mode*/) {
    throw ArmError(-1, "moveP_CANFD not implemented in V1");
}

std::vector<std::string> Arm::Impl::getWorkFrames() {
    throw ArmError(-1, "getWorkFrames not implemented in V1");
}

void Arm::Impl::setWorkFrame(const std::string& /*name*/) {
    throw ArmError(-1, "setWorkFrame not implemented in V1");
}

void Arm::Impl::enableForceControl(const std::array<double, 6>& /*params*/) {
    throw ArmError(-1, "enableForceControl not implemented in V1");
}

void Arm::Impl::disableForceControl() {
    throw ArmError(-1, "disableForceControl not implemented in V1");
}

// ── onMotionComplete — register callback ──

void Arm::Impl::onMotionComplete(Arm::MotionCallback cb) {
    std::lock_guard lock(cb_mutex_);
    motion_cb_ = std::move(cb);
}

} // namespace rm
