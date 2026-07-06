#pragma once

#include "omr_controller/types.hpp"

namespace omr_controller {

class MotorClient {
public:
    virtual ~MotorClient() = default;

    virtual bool enable() = 0;
    virtual bool disable() = 0;
    virtual bool isEnabled() const = 0;
    virtual bool setVelocity(double rad_per_s) = 0;
    virtual MotorState getState() const = 0;
    virtual bool stop() = 0;
};

class MotorClientStub : public MotorClient {
public:
    bool enable() override { return false; }
    bool disable() override { return false; }
    bool isEnabled() const override { return false; }
    bool setVelocity(double /*rad_per_s*/) override { return false; }
    MotorState getState() const override { return {}; }
    bool stop() override { return false; }
};

}  // namespace omr_controller
