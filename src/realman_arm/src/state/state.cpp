#include "core/arm_impl.hpp"

namespace rm {

// ──────────────────────────────────────────────
//  pollState — read arm state from hardware
// ──────────────────────────────────────────────

void Arm::Impl::pollState() const {
    if (!handle_) return;
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

// ──────────────────────────────────────────────
//  State accessors (read cached, poll on read)
// ──────────────────────────────────────────────

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

} // namespace rm
