#include "core/arm_impl.hpp"

// ══════════════════════════════════════════════
//  Arm facade — delegates to Impl
// ══════════════════════════════════════════════

namespace rm {

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
void Arm::stop()                                   { impl_->stop(); }
void Arm::setGripperRoute(int min, int max)        { impl_->setGripperRoute(min, max); }
void Arm::gripper(int pos, bool b, int t)          { impl_->gripper(pos, b, t); }
void Arm::gripperRelease(int s, bool b, int t)     { impl_->gripperRelease(s, b, t); }
void Arm::gripperPick(int s, int f, bool b, int t) { impl_->gripperPick(s, f, b, t); }
void Arm::gripperPickOn(int s, int f, bool b, int t) { impl_->gripperPickOn(s, f, b, t); }
GripperState Arm::gripperState() const             { return impl_->gripperState(); }
JointPosition Arm::jointPosition() const           { return impl_->jointPosition(); }
CartesianPose Arm::toolPose() const                { return impl_->toolPose(); }
ArmState Arm::state() const                        { return impl_->state(); }
bool Arm::isConnected() const                      { return impl_->isConnected(); }
void Arm::moveJ_CANFD(const JointPosition& t, int m) { impl_->moveJ_CANFD(t, m); }
void Arm::moveP_CANFD(const CartesianPose& t, int m) { impl_->moveP_CANFD(t, m); }
std::vector<std::string> Arm::getWorkFrames()      { return impl_->getWorkFrames(); }
void Arm::setWorkFrame(const std::string& n)       { impl_->setWorkFrame(n); }
void Arm::enableForceControl(const std::array<double, 6>& p) { impl_->enableForceControl(p); }
void Arm::disableForceControl()                    { impl_->disableForceControl(); }
void Arm::onMotionComplete(MotionCallback cb)      { impl_->onMotionComplete(std::move(cb)); }

} // namespace rm
