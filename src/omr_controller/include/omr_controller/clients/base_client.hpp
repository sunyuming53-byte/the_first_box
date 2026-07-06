#pragma once

#include <array>

namespace omr_controller {

class BaseClient {
 public:
  virtual ~BaseClient() = default;

  virtual bool move(double linear_x, double angular_z) = 0;
  virtual bool stop() = 0;
  virtual std::array<double, 3> getPose() const = 0;
};

class BaseClientStub : public BaseClient {
 public:
  bool move(double /*linear_x*/, double /*angular_z*/) override { return false; }
  bool stop() override { return false; }
  std::array<double, 3> getPose() const override { return {0.0, 0.0, 0.0}; }
};

}  // namespace omr_controller
