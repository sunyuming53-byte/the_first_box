#pragma once

#include <array>
#include "expected_polyfill.hpp"
#include <opencv2/core/mat.hpp>
#include <span>
#include <string>
#include <vector>

namespace rm::calib {

/// Hand-eye calibration configuration mode.
enum class HandEyeMode {
    EyeInHand,   // camera rigidly mounted on end-effector
    EyeToHand,   // camera fixed in the world
};

struct PoseProcResult {
    /// Relative rotation matrices (3×3, CV_64F).  One per adjacent pose pair.
    std::vector<cv::Mat> R_motions;

    /// Relative translation vectors (3×1, CV_64F).  One per adjacent pose pair.
    std::vector<cv::Mat> t_motions;
};

/// Converts absolute arm poses into relative camera-motion matrices (A_i).
///
/// - EyeInHand:  poses are T_end2base.  A_i = inv(T_{i+1}) * T_i.
/// - EyeToHand:  poses are T_end2base.  A_i = T_{i+1} * inv(T_i).
///
/// Each pose is {tx,ty,tz,rx,ry,rz} in meters and radians (Euler RPY).
class PoseProcessor {
public:
    explicit PoseProcessor(HandEyeMode mode);

    [[nodiscard]] auto process(std::span<const std::array<double, 6>> poses)
        -> Result<PoseProcResult>;

private:
    HandEyeMode mode_;
};

}  // namespace rm::calib
