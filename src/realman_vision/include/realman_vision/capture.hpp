#pragma once

#include "realman_vision/camera/types.hpp"
#include <filesystem>
#include <vector>
#include <array>
#include <opencv2/core/types.hpp>

namespace rm::vision {

struct ArmPose {
    double tx{}, ty{}, tz{};   // meters
    double rx{}, ry{}, rz{};   // radians, Euler RPY
};

struct CaptureConfig {
    std::filesystem::path output_dir{"calib_data"};
    int total_images{18};
    bool save_depth{false};  // calibration only needs RGB
    cv::Size board_size{11, 8};
};

// Interactive frame capture for calibration data collection.
// Drives the OpenCV GUI: shows live camera feed with corner overlay,
// saves frames + arm poses on 's', quits on 'q'/'ESC'.
class FrameCapture {
public:
    explicit FrameCapture(const CaptureConfig& cfg);

    // Draw corner overlay on color image. Returns viewable BGR image.
    [[nodiscard]] auto draw_overlay(const camera::CameraFrame& frame,
                                    const std::vector<cv::Point2f>& corners,
                                    int saved_count,
                                    int total_required) -> cv::Mat;

    // Save raw color image (and depth if configured).
    void save(const camera::CameraFrame& frame, int index);

    // Save frame + append arm pose to CSV.
    void save_with_pose(const camera::CameraFrame& frame, int index,
                        const ArmPose& pose);

    [[nodiscard]] auto get_board_size() const -> cv::Size { return cfg_.board_size; }
    [[nodiscard]] auto get_total_required() const -> int { return cfg_.total_images; }
    [[nodiscard]] auto get_output_dir() const -> std::filesystem::path { return cfg_.output_dir; }

private:
    CaptureConfig cfg_;
};

} // namespace rm::vision
