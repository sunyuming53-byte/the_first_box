#pragma once

#include <cstdint>

#include <array>

#include <opencv2/core/mat.hpp>

namespace omr_vision::camera {

struct CameraConfig {
    int width{1280};
    int height{720};
    int fps{30};
    bool enable_depth{true};
};

struct CameraIntrinsics {
    cv::Mat K;           // (3, 3) CV_64F
    cv::Mat dist_coeff;  // (1, 5) CV_64F

    [[nodiscard]] auto fx() const -> double { return K.at<double>(0, 0); }
    [[nodiscard]] auto fy() const -> double { return K.at<double>(1, 1); }
    [[nodiscard]] auto cx() const -> double { return K.at<double>(0, 2); }
    [[nodiscard]] auto cy() const -> double { return K.at<double>(1, 2); }
    [[nodiscard]] auto width() const -> int { return static_cast<int>(cx() * 2); }
    [[nodiscard]] auto height() const -> int { return static_cast<int>(cy() * 2); }
};

struct CameraFrame {
    cv::Mat color;  // (H, W) CV_8UC3, BGR
    cv::Mat depth;  // (H, W) CV_16UC1, mm, aligned to color
    CameraIntrinsics color_intrinsics;
    CameraIntrinsics depth_intrinsics;
    int64_t frame_id{0};

    // Back-project pixel (u,v) to 3D point in camera frame (meters)
    [[nodiscard]] auto point_3d(int u, int v) const -> std::array<double, 3>;
};

}  // namespace omr_vision::camera
