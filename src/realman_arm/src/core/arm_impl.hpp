// PRIVATE header — NOT part of the public API.
// Included only by Impl method implementations under src/.
#pragma once
#include "realman/core/arm.hpp"
#include "realman/gripper/types.hpp"
#include "realman/motion/types.hpp"

#include <cmath>
#include <rm_interface.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

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
    void moveC(const CartesianPose& mid, const CartesianPose& end, SpeedRatio speed, int loop,
               bool blocking);
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
    CartesianPose toolPose() const;
    ArmState state() const;

    // ── Callbacks ──
    void onMotionComplete(Arm::MotionCallback cb);

    // ── V1 stubs ──
    static void moveJ_CANFD(const JointPosition& target, int mode);
    static void moveP_CANFD(const CartesianPose& target, int mode);
    static std::vector<std::string> getWorkFrames();
    static void setWorkFrame(const std::string& name);
    static void enableForceControl(const std::array<double, 6>& params);
    static void disableForceControl();

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
    mutable CartesianPose tool_pose_;
    mutable ArmState arm_state_;

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
    pose.quaternion.w = 1.0F;
    pose.quaternion.x = 0.0F;
    pose.quaternion.y = 0.0F;
    pose.quaternion.z = 0.0F;
    return pose;
}

}  // namespace impl

}  // namespace rm
