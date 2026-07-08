#include "omr_vision/calibration/format_polyfill.hpp"
#include "omr_controller/calib/transform.hpp"

#include <cmath>

#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

namespace omr_controller::calib {

// ──────────────────────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────────────────────
namespace {

/// RPY (Z-Y-X Euler, radians) → 3×3 CV_64F rotation matrix.
[[nodiscard]] auto rpy_to_rotation(double roll, double pitch, double yaw) -> cv::Mat {
    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    auto R = cv::Mat_<double>(3, 3);

    R(0, 0) = cy * cp;
    R(0, 1) = cy * sp * sr - sy * cr;
    R(0, 2) = cy * sp * cr + sy * sr;
    R(1, 0) = sy * cp;
    R(1, 1) = sy * sp * sr + cy * cr;
    R(1, 2) = sy * sp * cr - cy * sr;
    R(2, 0) = -sp;
    R(2, 1) = cp * sr;
    R(2, 2) = cp * cr;

    return R;
}

/// 3×3 rotation matrix → RPY (Z-Y-X Euler, radians).
[[nodiscard]] auto rotation_to_rpy(const cv::Mat& R) -> std::array<double, 3> {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double sp = -R.at<double>(2, 0);

    // Handle gimbal lock
    if (std::abs(sp) > 0.99999) {
        pitch = (sp > 0.0) ? M_PI_2 : -M_PI_2;
        roll = std::atan2(-R.at<double>(0, 1), R.at<double>(1, 1));
        yaw = 0.0;
    } else {
        pitch = std::asin(sp);
        roll =
            std::atan2(R.at<double>(2, 1) / std::cos(pitch), R.at<double>(2, 2) / std::cos(pitch));
        yaw =
            std::atan2(R.at<double>(1, 0) / std::cos(pitch), R.at<double>(0, 0) / std::cos(pitch));
    }

    return {roll, pitch, yaw};
}

/// Arm pose {tx,ty,tz,rx,ry,rz} → 4×4 homogeneous matrix.
[[nodiscard]] auto arm_pose_to_homogeneous(const std::array<double, 6>& arm_pose) -> cv::Mat {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat R = rpy_to_rotation(arm_pose[3], arm_pose[4], arm_pose[5]);
    cv::Mat roi = T(cv::Rect(0, 0, 3, 3));
    R.copyTo(roi);
    T.at<double>(0, 3) = arm_pose[0];
    T.at<double>(1, 3) = arm_pose[1];
    T.at<double>(2, 3) = arm_pose[2];
    return T;
}

/// Transform a 3D point by a 4×4 homogeneous matrix.
[[nodiscard]] auto transform_point(const cv::Mat& T, double x, double y, double z)
    -> std::array<double, 3> {
    cv::Mat p = (cv::Mat_<double>(4, 1) << x, y, z, 1.0);
    cv::Mat q = T * p;
    return {q.at<double>(0), q.at<double>(1), q.at<double>(2)};
}

}  // anonymous namespace

// ──────────────────────────────────────────────────────────────
// HandEyeTransform
// ──────────────────────────────────────────────────────────────

HandEyeTransform::HandEyeTransform(cv::Mat T, HandEyeMode mode) : T_{std::move(T)}, mode_{mode} {}

auto HandEyeTransform::load(const std::filesystem::path& yaml_path) -> Result<HandEyeTransform> {
    cv::FileStorage fs(yaml_path.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return Unexpected<std::string>(
            std::format("Cannot open hand-eye result file: {}", yaml_path.string()));
    }

    cv::Mat R;
    cv::Mat t;
    std::string mode_str;

    fs["rotation_matrix"] >> R;
    fs["translation_vector"] >> t;
    fs["mode"] >> mode_str;

    if (R.empty() || t.empty()) {
        return Unexpected<std::string>("YAML missing rotation_matrix or translation_vector");
    }

    HandEyeMode mode = HandEyeMode::EyeInHand;
    if (mode_str == "eye_to_hand") {
        mode = HandEyeMode::EyeToHand;
    } else if (mode_str != "eye_in_hand") {
        return Unexpected<std::string>(std::format(
            "Unknown mode '{}' in YAML (expected 'eye_in_hand' or 'eye_to_hand')", mode_str));
    }

    // Build 4×4 homogeneous T_ = [R  t; 0  1]
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat roi_R = T(cv::Rect(0, 0, 3, 3));
    R.copyTo(roi_R);
    cv::Mat roi_t = T(cv::Rect(3, 0, 1, 3));
    t.copyTo(roi_t);

    fs.release();
    return HandEyeTransform{T, mode};
}

auto HandEyeTransform::point_camera_to_base(double x, double y, double z,
                                            const std::array<double, 6>& arm_pose)
    -> std::array<double, 3> {
    switch (mode_) {
        case HandEyeMode::EyeInHand: {
            // p_base = H_ee2base * T_ * p_cam
            auto H_ee2base = arm_pose_to_homogeneous(arm_pose);
            auto p = transform_point(T_, x, y, z);  // T_ * p_cam  (camera → end-effector)
            return transform_point(H_ee2base, p[0], p[1], p[2]);  // end-effector → base
        }
        case HandEyeMode::EyeToHand: {
            // p_base = T_ * p_cam  (camera → base directly)
            return transform_point(T_, x, y, z);
        }
    }
    return {x, y, z};  // unreachable
}

auto HandEyeTransform::pose_camera_to_base(double x, double y, double z, double rx, double ry,
                                           double rz, const std::array<double, 6>& arm_pose)
    -> std::array<double, 6> {
    // Build 4×4 homogeneous for the camera-frame pose
    auto R_cam = rpy_to_rotation(rx, ry, rz);
    cv::Mat T_cam = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat roi_cam = T_cam(cv::Rect(0, 0, 3, 3));
    R_cam.copyTo(roi_cam);
    T_cam.at<double>(0, 3) = x;
    T_cam.at<double>(1, 3) = y;
    T_cam.at<double>(2, 3) = z;

    cv::Mat T_base;

    switch (mode_) {
        case HandEyeMode::EyeInHand: {
            // T_base = H_ee2base * T_ * T_cam
            auto H_ee2base = arm_pose_to_homogeneous(arm_pose);
            T_base = H_ee2base * T_ * T_cam;
            break;
        }
        case HandEyeMode::EyeToHand: {
            // T_base = T_ * T_cam
            T_base = T_ * T_cam;
            break;
        }
    }

    // Decompose result into {tx,ty,tz,rx,ry,rz}
    auto R_result = T_base(cv::Rect(0, 0, 3, 3));
    auto rpy = rotation_to_rpy(R_result);

    return {
        T_base.at<double>(0, 3),
        T_base.at<double>(1, 3),
        T_base.at<double>(2, 3),
        rpy[0],
        rpy[1],
        rpy[2],
    };
}

}  // namespace omr_controller::calib
