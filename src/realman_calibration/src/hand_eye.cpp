#include "realman_calibration/hand_eye.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include <cmath>
#include <format>
#include <span>

namespace rm::calib {

// ──────────────────────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────────────────────
namespace {

/// Convert Rodrigues vectors to 3×3 rotation matrices.
[[nodiscard]] auto rvecs_to_R(std::span<const cv::Mat> rvecs) -> std::vector<cv::Mat> {
    std::vector<cv::Mat> R_mats;
    R_mats.reserve(rvecs.size());
    for (const auto& rv : rvecs) {
        cv::Mat R;
        cv::Rodrigues(rv, R);
        R_mats.push_back(R);
    }
    return R_mats;
}

/// Compute reprojection error: for each pair (R_tool, t_tool, R_cam, t_cam),
/// the solved X (R, t) should satisfy the hand-eye equation.
[[nodiscard]] auto compute_reproj_error(std::span<const cv::Mat> R_tool,
                                         std::span<const cv::Mat> t_tool,
                                         std::span<const cv::Mat> R_cam,
                                         std::span<const cv::Mat> t_cam,
                                         const cv::Mat& R_X,
                                         const cv::Mat& t_X,
                                         HandEyeMode mode) -> double
{
    if (R_tool.empty()) return 0.0;

    double total_err = 0.0;
    const auto N = std::min(R_tool.size(), R_cam.size());

    for (auto i = 0uz; i < N; ++i) {
        cv::Mat A_R = R_tool[i];
        cv::Mat A_t = t_tool[i];
        cv::Mat B_R = R_cam[i];
        cv::Mat B_t = t_cam[i];

        cv::Mat B_R_t;
        cv::Rodrigues(cv::Mat::eye(3, 3, CV_64F), B_R_t);  // placeholder, not used

        // Build 4×4 A and B matrices
        cv::Mat A = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat A_roi_R = A(cv::Rect(0, 0, 3, 3));
        A_R.copyTo(A_roi_R);
        cv::Mat A_roi_t = A(cv::Rect(3, 0, 1, 3));
        A_t.copyTo(A_roi_t);

        cv::Mat B = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat B_roi_R = B(cv::Rect(0, 0, 3, 3));
        B_R.copyTo(B_roi_R);
        cv::Mat B_roi_t = B(cv::Rect(3, 0, 1, 3));
        B_t.copyTo(B_roi_t);

        cv::Mat X = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat X_roi_R = X(cv::Rect(0, 0, 3, 3));
        R_X.copyTo(X_roi_R);
        cv::Mat X_roi_t = X(cv::Rect(3, 0, 1, 3));
        t_X.copyTo(X_roi_t);

        // A * X should equal X * B  (hand-eye equation)
        cv::Mat left  = A * X;
        cv::Mat right = X * B;

        // error = norm(left - right)
        auto diff = left - right;
        total_err += cv::norm(diff, cv::NORM_L2);
    }

    return total_err / static_cast<double>(N);
}

} // anonymous namespace

// ──────────────────────────────────────────────────────────────
// HandEyeSolver
// ──────────────────────────────────────────────────────────────

HandEyeSolver::HandEyeSolver(HandEyeMode mode) : mode_{mode} {}

auto HandEyeSolver::solve(std::span<const cv::Mat> R_tool,
                           std::span<const cv::Mat> t_tool,
                           std::span<const cv::Mat> rvecs,
                           std::span<const cv::Mat> tvecs)
    -> std::expected<HandEyeResult, std::string>
{
    if (R_tool.size() < 3) {
        return std::unexpected(
            std::format("Need at least 3 relative motions (got {})", R_tool.size()));
    }

    // Convert camera rvecs to rotation matrices
    auto R_cam = rvecs_to_R(rvecs);

    if (R_cam.size() != R_tool.size()) {
        return std::unexpected(
            std::format("Mismatched data sizes: {} tool motions vs {} camera views",
                        R_tool.size(), R_cam.size()));
    }

    cv::Mat R_result, t_result;

    switch (mode_) {
    case HandEyeMode::EyeInHand: {
        // Eye-in-hand: R_tool = end-effector motions, R_cam = pattern-in-camera
        // calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
        //                  R_cam2gripper, t_cam2gripper, method)
        std::vector<cv::Mat> R_tool_vec(R_tool.begin(), R_tool.end());
        std::vector<cv::Mat> t_tool_vec(t_tool.begin(), t_tool.end());
        std::vector<cv::Mat> R_cam_vec(R_cam.begin(), R_cam.end());
        std::vector<cv::Mat> tvecs_vec(tvecs.begin(), tvecs.end());
        cv::calibrateHandEye(R_tool_vec, t_tool_vec,
                             R_cam_vec, tvecs_vec,
                             R_result, t_result,
                             cv::CALIB_HAND_EYE_TSAI);
        break;
    }
    case HandEyeMode::EyeToHand: {
        // Eye-to-hand: R_cam = pattern motions (as gripper2base),
        //              R_tool = base2end motions (as target2cam)
        std::vector<cv::Mat> R_cam_vec2(R_cam.begin(), R_cam.end());
        std::vector<cv::Mat> tvecs_vec2(tvecs.begin(), tvecs.end());
        std::vector<cv::Mat> R_tool_vec2(R_tool.begin(), R_tool.end());
        std::vector<cv::Mat> t_tool_vec2(t_tool.begin(), t_tool.end());
        cv::calibrateHandEye(R_cam_vec2, tvecs_vec2,
                             R_tool_vec2, t_tool_vec2,
                             R_result, t_result,
                             cv::CALIB_HAND_EYE_TSAI);
        break;
    }
    }

    // Compute reprojection error
    double err = compute_reproj_error(R_tool, t_tool, R_cam, tvecs,
                                       R_result, t_result, mode_);

    return HandEyeResult{
        .R             = R_result,
        .t             = t_result,
        .mode          = mode_,
        .reproj_error  = err,
        .method        = "Tsai",
    };
}

// ──────────────────────────────────────────────────────────────
// HandEyeResult::save_yaml
// ──────────────────────────────────────────────────────────────

void HandEyeResult::save_yaml(const std::filesystem::path& path) const {
    cv::FileStorage fs(path.string(), cv::FileStorage::WRITE);
    if (!fs.isOpened()) return;

    fs << "version" << 1;
    fs << "mode" << (mode == HandEyeMode::EyeInHand ? "eye_in_hand" : "eye_to_hand");
    fs << "rotation_matrix" << R;
    fs << "translation_vector" << t;
    fs << "reprojection_error" << reproj_error;
    fs << "method" << method;

    fs.release();
}

} // namespace rm::calib
