#include "omr_controller/clients/guideway_client.hpp"

#include <string>
#include <utility>

namespace omr_controller {

GuidewayClientImpl::GuidewayClientImpl(GuidewayConfig cfg, rclcpp::Logger logger)
    : logger_(std::move(logger)) {
    // No serial port is opened here (lazy connection); the RailController
    // constructor only starts its worker/safety thread.
    rail_ = std::make_unique<omr_hardware::RailController>(
        std::move(cfg), [this](omr_hardware::RailLogLevel lvl, const std::string& msg) {
            switch (lvl) {
                case omr_hardware::RailLogLevel::INFO:
                    RCLCPP_INFO(logger_, "[guideway] %s", msg.c_str());
                    break;
                case omr_hardware::RailLogLevel::WARN:
                    RCLCPP_WARN(logger_, "[guideway] %s", msg.c_str());
                    break;
                case omr_hardware::RailLogLevel::ERROR:
                    RCLCPP_ERROR(logger_, "[guideway] %s", msg.c_str());
                    break;
            }
        });
}

bool GuidewayClientImpl::ensureConnected() {
    if (rail_->isConnected()) {
        return true;
    }
    return rail_->connect();
}

bool GuidewayClientImpl::connect() { return ensureConnected(); }

void GuidewayClientImpl::disconnect() { rail_->disconnect(); }

bool GuidewayClientImpl::isConnected() const { return rail_->isConnected(); }

GuidewayStatus GuidewayClientImpl::status() const { return rail_->status(); }

GuidewayCalibration GuidewayClientImpl::calibration() const { return rail_->calibration(); }

bool GuidewayClientImpl::isCalibrated() const { return rail_->calibration().calibrated; }

bool GuidewayClientImpl::isHomed() const { return rail_->calibration().homed; }

GuidewaySafetyState GuidewayClientImpl::safetyState() const { return rail_->status().safety; }

double GuidewayClientImpl::railPositionM() const { return rail_->status().rail_position_m; }

double GuidewayClientImpl::railVelocityMps() const { return rail_->status().rail_velocity_mps; }

bool GuidewayClientImpl::calibrate() {
    if (!ensureConnected()) {
        return false;
    }
    return rail_->calibrate();
}

bool GuidewayClientImpl::setVelocity(double rail_mps) {
    if (!ensureConnected()) {
        return false;
    }
    return rail_->setVelocityMps(rail_mps);
}

bool GuidewayClientImpl::jog(GuidewayDirection dir, double rpm) {
    if (!ensureConnected()) {
        return false;
    }
    return rail_->jog(dir, rpm);
}

bool GuidewayClientImpl::stop() {
    if (!rail_->isConnected()) {
        return true;  // nothing moving
    }
    return rail_->stop();
}

bool GuidewayClientImpl::moveToRail(double rail_m, double timeout_s) {
    if (!ensureConnected()) {
        return false;
    }
    return rail_->moveToRail(rail_m, timeout_s);
}

bool GuidewayClientImpl::moveToRailAsync(double rail_m) {
    if (!ensureConnected()) {
        return false;
    }
    return rail_->moveToRailAsync(rail_m);
}

bool GuidewayClientImpl::motionDone() const { return rail_->status().motion_done; }

bool GuidewayClientImpl::waitMotionDone(double timeout_s) {
    return rail_->waitMotionDone(timeout_s);
}

bool GuidewayClientImpl::moveToHome(double timeout_s) {
    if (!ensureConnected()) {
        return false;
    }
    return rail_->moveToHome(timeout_s);
}

bool GuidewayClientImpl::clearFault() {
    if (!ensureConnected()) {
        return false;
    }
    return rail_->clearFault();
}

}  // namespace omr_controller
