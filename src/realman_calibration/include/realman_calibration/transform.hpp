#pragma once

#include "pose_proc.hpp"
#include <array>
#include <expected>
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <string>

namespace rm::calib {

/// Application-side consumer of a calibrated hand-eye transform.
///
/// Load a result YAML (produced by HandEyeResult::save_yaml), then
/// transform camera-frame points/poses into the robot base frame.
///
/// Eye-in-hand requires the current arm pose at every call; eye-to-hand
/// ignores the arm_pose argument (use the default).
///
/// Internally stores a precomputed 4×4 homogeneous matrix.
class HandEyeTransform {
public:
    /// Load a calibrated transform from a YAML file written by
    /// HandEyeResult::save_yaml.
    [[nodiscard]] static auto load(const std::filesystem::path& yaml_path)
        -> std::expected<HandEyeTransform, std::string>;

    /// Transform a 3D point from camera frame to robot base frame.
    ///
    /// Eye-in-hand  (eye_in_hand=true):  p_base = H_ee_base * T_ * p_cam
    /// Eye-to-hand  (eye_to_hand=true):  p_base = T_ * p_cam
    [[nodiscard]] auto point_camera_to_base(double x, double y, double z,
                                             const std::array<double, 6>& arm_pose = {})
        -> std::array<double, 3>;

    /// Transform a 6-DOF pose from camera frame to robot base frame.
    ///
    /// arm_pose is only used in eye-in-hand mode.
    [[nodiscard]] auto pose_camera_to_base(double x, double y, double z,
                                            double rx, double ry, double rz,
                                            const std::array<double, 6>& arm_pose = {})
        -> std::array<double, 6>;

    [[nodiscard]] auto mode() const -> HandEyeMode { return mode_; }

private:
    HandEyeTransform(cv::Mat T, HandEyeMode mode);

    cv::Mat T_;        // 4×4 homogeneous matrix, CV_64F
    HandEyeMode mode_;
};

}  // namespace rm::calib
