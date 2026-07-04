#pragma once
#include <rm_interface.h>

#include <memory>

namespace rm::hal {

/// Thin abstraction over the RealMan C SDK (libapi_c.so).
/// Enables unit testing of business logic without physical hardware.
class RmHal {
public:
    virtual ~RmHal() = default;

    // ── Connection ──
    virtual int init(int mode) = 0;
    virtual rm_robot_handle* createRobotArm(const char* ip, int port) = 0;
    virtual int deleteRobotArm(rm_robot_handle* handle) = 0;

    // ── Motion ──
    virtual int movej(rm_robot_handle* handle, const float* joints, int speed, int block_flag,
                      int tc) = 0;
    virtual int movej_p(rm_robot_handle* handle, rm_pose_t pose, int speed, int tc,
                        int block_flag) = 0;
    virtual int movel(rm_robot_handle* handle, rm_pose_t pose, int speed, int tc,
                      int block_flag) = 0;
    virtual int movec(rm_robot_handle* handle, rm_pose_t via, rm_pose_t to, int speed, int loop,
                      int block_flag) = 0;
    virtual int stop(rm_robot_handle* handle) = 0;

    // ── State ──
    virtual int getCurrentArmState(rm_robot_handle* handle, rm_current_arm_state_t* state) = 0;
    virtual int getArmAllState(rm_robot_handle* handle, rm_arm_all_state_t* state) = 0;

    // ── Gripper ──
    virtual int setGripperRoute(rm_robot_handle* handle, int min, int max) = 0;
    virtual int setGripperPosition(rm_robot_handle* handle, int position, int blocking,
                                   int timeout) = 0;
    virtual int setGripperRelease(rm_robot_handle* handle, int speed, int blocking,
                                  int timeout) = 0;
    virtual int setGripperPick(rm_robot_handle* handle, int speed, int force, int blocking,
                               int timeout) = 0;
    virtual int setGripperPickOn(rm_robot_handle* handle, int speed, int force, int blocking,
                                 int timeout) = 0;
    virtual int getGripperState(rm_robot_handle* handle, rm_gripper_state_t* state) = 0;
};

/// Production implementation — delegates directly to the C SDK.
class RmHalImpl : public RmHal {
public:
    int init(int mode) override;
    rm_robot_handle* createRobotArm(const char* ip, int port) override;
    int deleteRobotArm(rm_robot_handle* handle) override;
    int movej(rm_robot_handle* handle, const float* joints, int speed, int block_flag,
              int tc) override;
    int movej_p(rm_robot_handle* handle, rm_pose_t pose, int speed, int tc,
                int block_flag) override;
    int movel(rm_robot_handle* handle, rm_pose_t pose, int speed, int tc, int block_flag) override;
    int movec(rm_robot_handle* handle, rm_pose_t via, rm_pose_t to, int speed, int loop,
              int block_flag) override;
    int stop(rm_robot_handle* handle) override;
    int getCurrentArmState(rm_robot_handle* handle, rm_current_arm_state_t* state) override;
    int getArmAllState(rm_robot_handle* handle, rm_arm_all_state_t* state) override;
    int setGripperRoute(rm_robot_handle* handle, int min, int max) override;
    int setGripperPosition(rm_robot_handle* handle, int position, int blocking,
                           int timeout) override;
    int setGripperRelease(rm_robot_handle* handle, int speed, int blocking, int timeout) override;
    int setGripperPick(rm_robot_handle* handle, int speed, int force, int blocking,
                       int timeout) override;
    int setGripperPickOn(rm_robot_handle* handle, int speed, int force, int blocking,
                         int timeout) override;
    int getGripperState(rm_robot_handle* handle, rm_gripper_state_t* state) override;
};

}  // namespace rm::hal
