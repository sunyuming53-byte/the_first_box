#pragma once

#include "expected_polyfill.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

namespace rm::calib {

struct CameraCalibInput {
    std::vector<cv::Mat> images;   // grayscale (CV_8UC1)
    cv::Size board_size;
    float square_size_m{0.030f};
};

struct CameraCalibResult {
    cv::Mat K;                          // 3×3, CV_64F  — camera matrix
    cv::Mat dist;                       // distortion coefficients
    std::vector<cv::Mat> rvecs;         // per-image rotation vectors
    std::vector<cv::Mat> tvecs;         // per-image translation vectors
    double reproj_error{0.0};           // RMS re-projection error (px)
    int images_used{0};                 // number of images that passed corner detection
};

/// Single-camera intrinsic calibration via chessboard.
///
/// Finds corners with cv::findChessboardCorners + sub-pixel refinement,
/// then calibrates with cv::calibrateCamera.  Images that fail corner
/// detection are silently skipped.
class CameraCalibrator {
public:
    explicit CameraCalibrator(const CameraCalibInput& input);

    [[nodiscard]] auto compute() -> Result<CameraCalibResult>;

private:
    CameraCalibInput input_;
};

}  // namespace rm::calib
