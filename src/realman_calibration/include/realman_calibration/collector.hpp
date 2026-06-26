#pragma once

#include "realman_vision/camera/types.hpp"
#include <array>
#include <expected>
#include <filesystem>
#include <memory>
#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

namespace rm::calib {

struct CalibDataConfig {
    int total_images{18};
    rm::vision::camera::CameraConfig camera;
    std::string arm_ip{"192.168.1.18"};
    std::filesystem::path output_dir{"data/calib_session"};
    cv::Size board_size{11, 8};
    float square_size_m{0.030f};
};

struct CalibSession {
    std::filesystem::path dir;
    std::vector<std::array<double, 6>> arm_poses;  // {tx,ty,tz,rx,ry,rz} meters, rad
};

/// Interactive calibration data collector.
///
/// Main loop: grab camera frame + read arm pose → display overlay →
/// wait for 's' key to save → validate rotation diversity across all
/// 3 axes exceeds 30° before accepting next capture.
class CalibDataCollector {
public:
    explicit CalibDataCollector(const CalibDataConfig& cfg);
    ~CalibDataCollector();

    CalibDataCollector(const CalibDataCollector&) = delete;
    auto operator=(const CalibDataCollector&) -> CalibDataCollector& = delete;

    /// Run the interactive capture loop.  Blocks until enough valid
    /// images are collected or an unrecoverable error occurs.
    [[nodiscard]] auto run() -> std::expected<CalibSession, std::string>;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rm::calib
