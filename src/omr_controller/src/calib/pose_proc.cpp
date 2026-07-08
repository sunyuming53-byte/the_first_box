#include "omr_vision/calibration/format_polyfill.hpp"
#include "omr_controller/calib/pose_proc.hpp"

#include <cmath>

#include <span>

namespace omr_controller::calib {

// ──────────────────────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────────────────────
namespace {

/// RPY (Z-Y-X Euler, radians) → 3×3 CV_64F rotation matrix.
[[nodiscard]] auto rpy_to_rotation(double roll, double pitch, double yaw) -> cv::Mat {
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    // Rx(roll)
    double r11 = 1.0;
    double r12 = 0.0;
    double r13 = 0.0;
    double r21 = 0.0;
    double r22 = cr;
    double r23 = -sr;
    double r31 = 0.0;
    double r32 = sr;
    double r33 = cr;

    // Ry(pitch) * Rx(roll)
    double c11 = (cp * r11) + (sp * r31);
    double c12 = (cp * r12) + (sp * r32);
    double c13 = (cp * r13) + (sp * r33);
    double c21 = r21;
    double c22 = r22;
    double c23 = r23;
    double c31 = (-sp * r11) + (cp * r31);
    double c32 = (-sp * r12) + (cp * r32);
    double c33 = (-sp * r13) + (cp * r33);

    // Rz(yaw) * (Ry * Rx)
    R.at<double>(0, 0) = cy * c11 - sy * c21;  // c11=cp, c21=0
    R.at<double>(0, 1) = cy * c12 - sy * c22;  // c12=cp*0+sp*sr=sp*sr, c22=cr
    R.at<double>(0, 2) = cy * c13 - sy * c23;  // c13=cp*0+sp*cr=sp*cr, c23=-sr
    R.at<double>(1, 0) = sy * c11 + cy * c21;
    R.at<double>(1, 1) = sy * c12 + cy * c22;
    R.at<double>(1, 2) = sy * c13 + cy * c23;
    R.at<double>(2, 0) = c31;  // -sp
    R.at<double>(2, 1) = c32;  // cp*sr
    R.at<double>(2, 2) = c33;  // cp*cr

    return R;
}

/// Pose {tx,ty,tz,rx,ry,rz} → 4×4 homogeneous matrix.
[[nodiscard]] auto pose_to_homogeneous(const std::array<double, 6>& pose) -> cv::Mat {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat R = rpy_to_rotation(pose[3], pose[4], pose[5]);
    cv::Mat roi = T(cv::Rect(0, 0, 3, 3));
    R.copyTo(roi);
    T.at<double>(0, 3) = pose[0];
    T.at<double>(1, 3) = pose[1];
    T.at<double>(2, 3) = pose[2];
    return T;
}

/// Extract 3×3 rotation (top-left) and 3×1 translation (top-right col) from 4×4 homogeneous.
[[nodiscard]] auto extract_R_t(const cv::Mat& T) -> std::pair<cv::Mat, cv::Mat> {
    auto R = T(cv::Rect(0, 0, 3, 3)).clone();
    auto t = T(cv::Rect(3, 0, 1, 3)).clone();
    return {R, t};
}

}  // anonymous namespace

// ──────────────────────────────────────────────────────────────
// PoseProcessor
// ──────────────────────────────────────────────────────────────

PoseProcessor::PoseProcessor(HandEyeMode mode) : mode_{mode} {}

auto PoseProcessor::process(std::span<const std::array<double, 6>> poses)
    -> Result<PoseProcResult> {
    if (poses.size() < 2) {
        return Unexpected<std::string>(std::format("Need at least 2 poses (got {})", poses.size()));
    }

    const auto N = poses.size();
    PoseProcResult result;
    result.R_motions.reserve(N - 1);
    result.t_motions.reserve(N - 1);

    switch (mode_) {
        case HandEyeMode::EyeInHand: {
            // poses are T_end2base
            // A_i = inv(T_{i+1}) * T_i   (i.e. relative motion of end-effector)
            for (auto i = 0UZ; i < N - 1; ++i) {
                auto T_i = pose_to_homogeneous(poses[i]);
                auto T_i1 = pose_to_homogeneous(poses[i + 1]);
                auto A = T_i1.inv() * T_i;
                auto [R, t] = extract_R_t(A);
                result.R_motions.push_back(std::move(R));
                result.t_motions.push_back(std::move(t));
            }
            break;
        }
        case HandEyeMode::EyeToHand: {
            // poses are T_end2base
            // A_i = T_{i+1} * inv(T_i)
            for (auto i = 0UZ; i < N - 1; ++i) {
                auto T_i = pose_to_homogeneous(poses[i]);
                auto T_i1 = pose_to_homogeneous(poses[i + 1]);
                auto A = T_i1 * T_i.inv();
                auto [R, t] = extract_R_t(A);
                result.R_motions.push_back(std::move(R));
                result.t_motions.push_back(std::move(t));
            }
            break;
        }
    }

    return result;
}

}  // namespace omr_controller::calib
