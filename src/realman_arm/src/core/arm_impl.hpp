// PRIVATE header — NOT part of the public API.
// Included only by Impl method implementations under src/.
#pragma once
#include "realman/core/arm.hpp"
#include "realman/motion/types.hpp"
#include "realman/gripper/types.hpp"
#include <rm_interface.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <functional>
#include <cmath>

namespace rm {

class Arm::Impl {
public:
    explicit Impl(const ArmConfig& config);
    ~Impl();

    // ── Connection ──
    bool ensureConnected();
    bool isConnected() const;
    void workerLoop();

    // ── Motion ──
    void moveJ(const JointPosition& target, SpeedRatio speed, bool blocking, int tc);
    void moveJ_P(const CartesianPose& target, SpeedRatio speed, bool blocking, int tc);
    void moveL(const CartesianPose& target, SpeedRatio speed, bool blocking, int tc);
    void moveC(const CartesianPose& mid, const CartesianPose& end,
               SpeedRatio speed, int loop, bool blocking);
    void stop();

    // ── Gripper ──
    void setGripperRoute(int min, int max);
    void gripper(int position, bool blocking, int timeout);
    void gripperRelease(int speed, bool blocking, int timeout);
    void gripperPick(int speed, int force, bool blocking, int timeout);
    void gripperPickOn(int speed, int force, bool blocking, int timeout);
    GripperState gripperState() const;

    // ── State ──
    void pollState() const;
    JointPosition jointPosition() const;
    CartesianPose  toolPose() const;
    ArmState       state() const;

    // ── Callbacks ──
    void onMotionComplete(Arm::MotionCallback cb);

    // ── V1 stubs ──
    void moveJ_CANFD(const JointPosition& target, int mode);
    void moveP_CANFD(const CartesianPose& target, int mode);
    std::vector<std::string> getWorkFrames();
    void setWorkFrame(const std::string& name);
    void enableForceControl(const std::array<double, 6>& params);
    void disableForceControl();

private:
    rm_robot_handle* handle_{nullptr};
    std::string ip_;
    int port_{0};
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

// ── Helpers shared across .cpp files ──

namespace impl {

inline void check(int ret, const char* operation) {
    if (ret != 0) {
        throw ArmError(ret, std::string(operation) + " failed (code " + std::to_string(ret) + ")");
    }
}

inline rm_pose_t toRmPose(const CartesianPose& cp) {
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

} // namespace impl

} // namespace rm
