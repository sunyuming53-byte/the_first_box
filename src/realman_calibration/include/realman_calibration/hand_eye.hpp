#pragma once

#include "pose_proc.hpp"
#include "expected_polyfill.hpp"
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <span>
#include <string>
#include <vector>

namespace rm::calib {

/// Result of a hand-eye calibration solve.
struct HandEyeResult {
    cv::Mat R;                     // 3×3, CV_64F  — camera→end-effector (or camera→base)
    cv::Mat t;                     // 3×1, CV_64F  — translation part
    HandEyeMode mode;
    double reproj_error{0.0};
    std::string method;

    /// Persist result as a YAML file via cv::FileStorage.
    void save_yaml(const std::filesystem::path& path) const;
};

/// Solve the AX = XB hand-eye calibration problem.
///
/// Uses cv::calibrateHandEye with cv::CALIB_HAND_EYE_TSAI.
class HandEyeSolver {
public:
    explicit HandEyeSolver(HandEyeMode mode);

    /// @param R_tool   Relative rotation matrices from PoseProcessor (A_i)
    /// @param t_tool   Relative translation vectors from PoseProcessor (A_i)
    /// @param rvecs    Per-image rotation vectors from CameraCalibrator
    /// @param tvecs    Per-image translation vectors from CameraCalibrator
    [[nodiscard]] auto solve(std::span<const cv::Mat> R_tool,
                              std::span<const cv::Mat> t_tool,
                              std::span<const cv::Mat> rvecs,
                              std::span<const cv::Mat> tvecs)
        -> Result<HandEyeResult>;

private:
    HandEyeMode mode_;
};

}  // namespace rm::calib
