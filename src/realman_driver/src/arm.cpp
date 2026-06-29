#include "realman/arm.hpp"
#include "realman/error.hpp"
#include <rm_interface.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <cstring>
#include <vector>
#include <cmath>

namespace {

void check(int ret, const char* operation) {
    if (ret != 0) {
        throw rm::ArmError(ret, std::string(operation) + " failed (code " + std::to_string(ret) + ")");
    }
}

rm_pose_t toRmPose(const rm::CartesianPose& cp) {
    rm_pose_t pose{};
    pose.position.x = static_cast<float>(cp.x);
    pose.position.y = static_cast<float>(cp.y);
    pose.position.z = static_cast<float>(cp.z);
    pose.euler.rx = static_cast<float>(cp.roll);
    pose.euler.ry = static_cast<float>(cp.pitch);
    pose.euler.rz = static_cast<float>(cp.yaw);
    pose.quaternion.w = 1.0f;
    pose.quaternion.x = 0.0f;
    pose.quaternion.y = 0.0f;
    pose.quaternion.z = 0.0f;
    return pose;
}

} // anonymous namespace

namespace rm {

class Arm::Impl {
public:
    explicit Impl(const ArmConfig& config);
    ~Impl();

    void moveJ(const JointPosition& target, SpeedRatio speed, bool blocking, int tc);
    void moveJ_P(const CartesianPose& target, SpeedRatio speed, bool blocking, int tc);
    void moveL(const CartesianPose& target, SpeedRatio speed, bool blocking, int tc);
    void moveC(const CartesianPose& mid, const CartesianPose& end,
               SpeedRatio speed, int loop, bool blocking);
    void stop();
    void setGripperRoute(int min, int max);
    void gripper(int position, bool blocking, int timeout);
    void gripperRelease(int speed, bool blocking, int timeout);
    void gripperPick(int speed, int force, bool blocking, int timeout);
    void gripperPickOn(int speed, int force, bool blocking, int timeout);
    GripperState gripperState() const;

    JointPosition jointPosition() const;
    CartesianPose  toolPose() const;
    ArmState       state() const;

    void moveJ_CANFD(const JointPosition& target, int mode);
    void moveP_CANFD(const CartesianPose& target, int mode);
    std::vector<std::string> getWorkFrames();
    void setWorkFrame(const std::string& name);
    void enableForceControl(const std::array<double, 6>& params);
    void disableForceControl();
    void onMotionComplete(Arm::MotionCallback cb);
    bool isConnected() const;

private:
    void workerLoop();
    void pollState() const;

    rm_robot_handle* handle_{nullptr};
    std::thread worker_;
    std::atomic<bool> running_{true};

    std::mutex cmd_mutex_;
    std::condition_variable cmd_cv_;
    std::queue<std::function<void()>> cmd_queue_;

    mutable std::mutex state_mutex_;
    mutable JointPosition joint_pos_;
    mutable CartesianPose  tool_pose_;
    mutable ArmState       arm_state_;

    std::mutex done_mutex_;
    std::condition_variable done_cv_;
    std::atomic<bool> motion_done_{true};
    std::atomic<bool> last_motion_ok_{true};

    std::mutex cb_mutex_;
    Arm::MotionCallback motion_cb_;
};

// ──────────────────────────────────────────────
//  Constructor
// ──────────────────────────────────────────────

Arm::Impl::Impl(const ArmConfig& config) {
    int ret = rm_init(RM_TRIPLE_MODE_E);
    check(ret, "rm_init");

    handle_ = rm_create_robot_arm(config.ip.c_str(), config.tcp_port);
    if (!handle_ || handle_->id < 0) {
        throw ArmError(-1, "Failed to connect to arm at " + config.ip +
                            ":" + std::to_string(config.tcp_port));
    }

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
        cmd();
    }
}

// ──────────────────────────────────────────────
//  moveJ — joint space motion to joint targets
// ──────────────────────────────────────────────

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

// ──────────────────────────────────────────────
//  moveJ_P — joint space motion to Cartesian pose
// ──────────────────────────────────────────────

void Arm::Impl::moveJ_P(const CartesianPose& target, SpeedRatio speed,
                        bool blocking, int tc) {
    rm_pose_t pose = toRmPose(target);

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

// ──────────────────────────────────────────────
//  moveL — Cartesian linear motion
// ──────────────────────────────────────────────

void Arm::Impl::moveL(const CartesianPose& target, SpeedRatio speed,
                      bool blocking, int tc) {
    rm_pose_t pose = toRmPose(target);

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

// ──────────────────────────────────────────────
//  moveC — Cartesian arc motion
// ──────────────────────────────────────────────

void Arm::Impl::moveC(const CartesianPose& mid, const CartesianPose& end,
                      SpeedRatio speed, int loop, bool blocking) {
    rm_pose_t pose_via = toRmPose(mid);
    rm_pose_t pose_to  = toRmPose(end);

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

// ──────────────────────────────────────────────
//  stop — emergency stop
// ──────────────────────────────────────────────

void Arm::Impl::stop() {
    int ret = rm_set_arm_stop(handle_);
    check(ret, "rm_set_arm_stop");
}

// ──────────────────────────────────────────────
//  setGripperRoute — set stroke range
// ──────────────────────────────────────────────

void Arm::Impl::setGripperRoute(int min, int max) {
    int ret = rm_set_gripper_route(handle_, min, max);
    check(ret, "rm_set_gripper_route");
}

// ──────────────────────────────────────────────
//  gripper — position control
// ──────────────────────────────────────────────

void Arm::Impl::gripper(int position, bool blocking, int timeout) {
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

// ──────────────────────────────────────────────
//  gripperRelease — open to max
// ──────────────────────────────────────────────

void Arm::Impl::gripperRelease(int speed, bool blocking, int timeout) {
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

// ──────────────────────────────────────────────
//  gripperPick — force-controlled grasp
// ──────────────────────────────────────────────

void Arm::Impl::gripperPick(int speed, int force, bool blocking, int timeout) {
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

// ──────────────────────────────────────────────
//  gripperPickOn — continuous force grasp
// ──────────────────────────────────────────────

void Arm::Impl::gripperPickOn(int speed, int force, bool blocking, int timeout) {
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

// ──────────────────────────────────────────────
//  gripperState — read gripper status
// ──────────────────────────────────────────────

GripperState Arm::Impl::gripperState() const {
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

// ──────────────────────────────────────────────
//  State accessors (read cached, poll on read)
// ──────────────────────────────────────────────

void Arm::Impl::pollState() const {
    rm_current_arm_state_t cs{};
    int ret = rm_get_current_arm_state(handle_, &cs);
    if (ret != 0) return;

    rm_arm_all_state_t as{};
    ret = rm_get_arm_all_state(handle_, &as);
    if (ret != 0) return;

    std::lock_guard lock(state_mutex_);

    // Joint angles: degrees → radians
    joint_pos_.radians.clear();
    for (int i = 0; i < ARM_DOF; ++i) {
        joint_pos_.radians.push_back(static_cast<double>(cs.joint[i]) * M_PI / 180.0);
    }

    // Tool pose
    tool_pose_.x     = static_cast<double>(cs.pose.position.x);
    tool_pose_.y     = static_cast<double>(cs.pose.position.y);
    tool_pose_.z     = static_cast<double>(cs.pose.position.z);
    tool_pose_.roll  = static_cast<double>(cs.pose.euler.rx);
    tool_pose_.pitch = static_cast<double>(cs.pose.euler.ry);
    tool_pose_.yaw   = static_cast<double>(cs.pose.euler.rz);

    // Arm state
    arm_state_.joint_position = joint_pos_;
    arm_state_.tool_pose      = tool_pose_;
    for (int i = 0; i < 6; ++i) {
        arm_state_.joint_current[i]     = static_cast<double>(as.joint_current[i]);
        arm_state_.joint_temperature[i] = static_cast<double>(as.joint_temperature[i]);
    }
    arm_state_.error_code    = (as.err.err_len > 0) ? as.err.err[0] : 0;
    arm_state_.error_message = "";
    arm_state_.is_moving     = false;
}

JointPosition Arm::Impl::jointPosition() const {
    pollState();
    std::lock_guard lock(state_mutex_);
    return joint_pos_;
}

CartesianPose Arm::Impl::toolPose() const {
    pollState();
    std::lock_guard lock(state_mutex_);
    return tool_pose_;
}

ArmState Arm::Impl::state() const {
    pollState();
    std::lock_guard lock(state_mutex_);
    return arm_state_;
}

// ──────────────────────────────────────────────
//  isConnected — check arm handle validity
// ──────────────────────────────────────────────

bool Arm::Impl::isConnected() const {
    return handle_ != nullptr;
}

// ──────────────────────────────────────────────
//  V1 stubs — not implemented
// ──────────────────────────────────────────────

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

void Arm::Impl::onMotionComplete(Arm::MotionCallback cb) {
    std::lock_guard lock(cb_mutex_);
    motion_cb_ = std::move(cb);
}

// ══════════════════════════════════════════════
//  Arm facade — delegate to Impl
// ══════════════════════════════════════════════

Arm::Arm(const ArmConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

Arm::~Arm() = default;

Arm::Arm(Arm&&) noexcept = default;
Arm& Arm::operator=(Arm&&) noexcept = default;

void Arm::moveJ(const JointPosition& target, SpeedRatio speed,
                bool blocking, int trajectory_connect) {
    impl_->moveJ(target, speed, blocking, trajectory_connect);
}

void Arm::moveJ_P(const CartesianPose& target, SpeedRatio speed,
                  bool blocking, int trajectory_connect) {
    impl_->moveJ_P(target, speed, blocking, trajectory_connect);
}

void Arm::moveL(const CartesianPose& target, SpeedRatio speed,
                bool blocking, int trajectory_connect) {
    impl_->moveL(target, speed, blocking, trajectory_connect);
}

void Arm::moveC(const CartesianPose& mid, const CartesianPose& end,
                SpeedRatio speed, int loop, bool blocking) {
    impl_->moveC(mid, end, speed, loop, blocking);
}

void Arm::stop() {
    impl_->stop();
}

void Arm::setGripperRoute(int min, int max) {
    impl_->setGripperRoute(min, max);
}

void Arm::gripper(int position, bool blocking, int timeout) {
    impl_->gripper(position, blocking, timeout);
}

void Arm::gripperRelease(int speed, bool blocking, int timeout) {
    impl_->gripperRelease(speed, blocking, timeout);
}

void Arm::gripperPick(int speed, int force, bool blocking, int timeout) {
    impl_->gripperPick(speed, force, blocking, timeout);
}

void Arm::gripperPickOn(int speed, int force, bool blocking, int timeout) {
    impl_->gripperPickOn(speed, force, blocking, timeout);
}

GripperState Arm::gripperState() const {
    return impl_->gripperState();
}

JointPosition Arm::jointPosition() const {
    return impl_->jointPosition();
}

CartesianPose Arm::toolPose() const {
    return impl_->toolPose();
}

ArmState Arm::state() const {
    return impl_->state();
}

bool Arm::isConnected() const {
    return impl_->isConnected();
}

void Arm::moveJ_CANFD(const JointPosition& target, int mode) {
    impl_->moveJ_CANFD(target, mode);
}

void Arm::moveP_CANFD(const CartesianPose& target, int mode) {
    impl_->moveP_CANFD(target, mode);
}

std::vector<std::string> Arm::getWorkFrames() {
    return impl_->getWorkFrames();
}

void Arm::setWorkFrame(const std::string& name) {
    impl_->setWorkFrame(name);
}

void Arm::enableForceControl(const std::array<double, 6>& params) {
    impl_->enableForceControl(params);
}

void Arm::disableForceControl() {
    impl_->disableForceControl();
}

void Arm::onMotionComplete(MotionCallback cb) {
    impl_->onMotionComplete(std::move(cb));
}

} // namespace rm
