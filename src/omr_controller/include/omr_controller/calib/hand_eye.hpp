#pragma once

#include "omr_vision/calibration/expected_polyfill.hpp"

#include <cstdint>

#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include "omr_controller/calib/pose_proc.hpp"
#include <opencv2/core/mat.hpp>

namespace omr_controller::calib {

/// Hand-eye calibration method.
enum class HandEyeMethod : uint8_t {
    Tsai,
    Park,
    Horaud,
    Daniilidis,
    Auto,  // run all four, select by condition number (primary) then reproj error (tiebreak)
};

/// Result of a hand-eye calibration solve.
struct HandEyeResult {
    cv::Mat R;  // 3×3, CV_64F  — camera→end-effector (or camera→base)
    cv::Mat t;  // 3×1, CV_64F  — translation part
    HandEyeMode mode = HandEyeMode::EyeInHand;
    double reproj_error{0.0};
    std::string method;
    double condition_number{0.0};
    HandEyeMethod used_method{HandEyeMethod::Auto};

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
    /// @param method   Hand-eye method (default Auto: run all four, pick best)
    [[nodiscard]] auto solve(std::span<const cv::Mat> R_tool, std::span<const cv::Mat> t_tool,
                             std::span<const cv::Mat> rvecs, std::span<const cv::Mat> tvecs,
                             HandEyeMethod method = HandEyeMethod::Auto) -> Result<HandEyeResult>;

private:
    HandEyeMode mode_;
};

}  // namespace omr_controller::calib
