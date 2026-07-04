#pragma once

#include "realman/motion/types.hpp"
#include "realman_vision/camera/types.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "expected_polyfill.hpp"
#include <opencv2/core/types.hpp>

namespace rm::calib {

struct CalibDataConfig {
    int total_images{18};
    rm::vision::camera::CameraConfig camera;
    std::string arm_ip{"192.168.1.18"};
    std::filesystem::path output_dir{"data/calib_session"};
    cv::Size board_size{11, 8};
    float square_size_m{0.030F};
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

    /// Run the calibration data collection loop.
    ///
    /// If @p waypoints is non-empty: auto-collection mode — moves the arm
    /// through each waypoint, auto-captures on board detection, and stops
    /// when enough images with sufficient rotation diversity are collected
    /// or all waypoints are exhausted.  Unreachable waypoints are silently
    /// skipped.
    ///
    /// If @p waypoints is empty: interactive mode — displays live preview
    /// and waits for the user to press 's' to save each frame.
    [[nodiscard]] auto run(std::vector<rm::JointPosition> waypoints = {}) -> Result<CalibSession>;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rm::calib
