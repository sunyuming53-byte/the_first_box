#pragma once
#include "realman/types.hpp"
#include "realman/error.hpp"
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <array>

namespace rm {

class Arm {
public:
    using MotionCallback = std::function<void(bool success)>;

    explicit Arm(const ArmConfig& config);
    ~Arm();

    // Non-copyable, movable (move members defined in .cpp where Impl is complete)
    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;
    Arm(Arm&&) noexcept;
    Arm& operator=(Arm&&) noexcept;

    // ── Motion ──
    void moveJ(const JointPosition& target, SpeedRatio speed = 50,
               bool blocking = true, int trajectory_connect = 0);
    void moveJ_P(const CartesianPose& target, SpeedRatio speed = 50,
                 bool blocking = true, int trajectory_connect = 0);
    void moveL(const CartesianPose& target, SpeedRatio speed = 50,
               bool blocking = true, int trajectory_connect = 0);
    void moveC(const CartesianPose& mid, const CartesianPose& end,
               SpeedRatio speed = 50, int loop = 1, bool blocking = true);
    void stop();

    // ── CANFD ──
    void moveJ_CANFD(const JointPosition& target, int mode = 0);
    void moveP_CANFD(const CartesianPose& target, int mode = 0);

    // ── State (read cached, non-blocking) ──
    JointPosition jointPosition() const;
    CartesianPose  toolPose() const;
    ArmState       state() const;

    // ── Frames ──
    std::vector<std::string> getWorkFrames();
    void setWorkFrame(const std::string& name);

    // ── Gripper (CTAG2F90D / EG2-4C2, via arm end-effector RS-485) ──
    void setGripperRoute(int min, int max);
    void gripper(int position, bool blocking = true, int timeout = 30);
    void gripperRelease(int speed, bool blocking = true, int timeout = 30);
    void gripperPick(int speed, int force, bool blocking = true, int timeout = 30);
    void gripperPickOn(int speed, int force, bool blocking = true, int timeout = 30);
    GripperState gripperState() const;

    // ── Force control (requires 6-axis force sensor) ──
    void enableForceControl(const std::array<double, 6>& params);
    void disableForceControl();

    // ── Callback ──
    void onMotionComplete(MotionCallback cb);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rm
