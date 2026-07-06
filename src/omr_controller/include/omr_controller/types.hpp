#pragma once

#include <opencv2/core/types.hpp>

#include <string>
#include <vector>

namespace omr_controller {

struct JointGoal {
  std::vector<double> positions;
  double speed_ratio{50};
  double time_from_start_sec{0.0};
};

struct TaskResult {
  bool success{false};
  std::string message;
};

struct ArmState {
  std::vector<double> joint_positions;
  bool connected{false};
};

struct MotorState {
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  bool connected{false};
  bool servo_enabled{false};
};

struct DetectionResult {
  std::string label;
  cv::Point2d center{0, 0};
  double confidence{0.0};
};

}  // namespace omr_controller
