#pragma once

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

namespace rm::calib::test {

/// Generate synthetic chessboard images for testing calibration.
///
/// Renders a perspective-distorted chessboard pattern (alternating
/// black and white squares) from random camera poses with controlled
/// diversity (≥15° rotation spread).  All output images are grayscale
/// (CV_8UC1) and pass `cv::findChessboardCorners`.
///
/// @param count       Number of images to generate (≥1)
/// @param board_size  Number of inner corners (cols, rows)
/// @param square_m    Square side length in metres
/// @param K           Camera intrinsic matrix (3×3, CV_64F)
/// @param dist        Distortion coefficients (CV_64F)
/// @param image_size  Output image dimensions
/// @return Vector of grayscale images suitable for CameraCalibrator
std::vector<cv::Mat> generate_chessboard_images(int count,
                                                cv::Size board_size,
                                                float square_m,
                                                cv::Mat K,
                                                cv::Mat dist,
                                                cv::Size image_size);

/// Generate synthetic Charuco board images for testing calibration.
///
/// Uses `cv::aruco::CharucoBoard::create` and
/// `cv::aruco::drawPlanarBoard` to render the board pattern, then
/// applies a perspective warp from random camera poses to create
/// realistic test images.
///
/// @param count       Number of images to generate (≥1)
/// @param squares_xy  Number of squares in x / y
/// @param square_len  Square side length in metres
/// @param marker_len  Marker side length in metres
/// @param dict        ArUco dictionary to use
/// @param K           Camera intrinsic matrix (3×3, CV_64F)
/// @param dist        Distortion coefficients (CV_64F)
/// @param image_size  Output image dimensions
/// @return Vector of grayscale images
std::vector<cv::Mat> generate_charuco_images(int count,
                                              cv::Size squares_xy,
                                              float square_len,
                                              float marker_len,
                                              cv::aruco::Dictionary dict,
                                              cv::Mat K,
                                              cv::Mat dist,
                                              cv::Size image_size);

}  // namespace rm::calib::test
