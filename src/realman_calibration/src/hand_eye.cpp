#include "realman_calibration/hand_eye.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include <cmath>
#include "realman_calibration/format_polyfill.hpp"
#include <span>

namespace rm::calib {

// ──────────────────────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────────────────────
namespace {

/// Map method enum to OpenCV flag.
[[nodiscard]] auto method_to_cv_flag(HandEyeMethod m) -> cv::HandEyeCalibrationMethod {
    switch (m) {
    case HandEyeMethod::Tsai:       return cv::CALIB_HAND_EYE_TSAI;
    case HandEyeMethod::Park:       return cv::CALIB_HAND_EYE_PARK;
    case HandEyeMethod::Horaud:     return cv::CALIB_HAND_EYE_HORAUD;
    case HandEyeMethod::Daniilidis: return cv::CALIB_HAND_EYE_DANIILIDIS;
    default:                        return cv::CALIB_HAND_EYE_TSAI;
    }
}

/// Map method enum to display string.
[[nodiscard]] auto method_to_string(HandEyeMethod m) -> const char* {
    switch (m) {
    case HandEyeMethod::Tsai:       return "Tsai";
    case HandEyeMethod::Park:       return "Park";
    case HandEyeMethod::Horaud:     return "Horaud";
    case HandEyeMethod::Daniilidis: return "Daniilidis";
    case HandEyeMethod::Auto:       return "Auto";
    }
    return "Unknown";
}

/// Compute condition number of the stacked rotation system.
/// Stacks all R_tool matrices (each 3×3) into a 3N×3 matrix and
/// returns max_sv / min_sv from SVD.  Returns 0.0 if fewer than 4 pairs.
[[nodiscard]] auto compute_condition_number(std::span<const cv::Mat> R_tool) -> double {
    if (R_tool.size() < 4) return 0.0;

    const auto N = static_cast<int>(R_tool.size());
    cv::Mat stacked(3 * N, 3, CV_64F);
    for (int i = 0; i < N; ++i) {
        R_tool[i].copyTo(stacked(cv::Rect(0, 3 * i, 3, 3)));
    }

    cv::Mat w;
    cv::SVD::compute(stacked, w);
    const double max_sv = w.at<double>(0, 0);
    const double min_sv = w.at<double>(2, 0);  // 3 singular values for 3×N matrix
    if (min_sv < 1e-15) return 0.0;
    return max_sv / min_sv;
}

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
                           std::span<const cv::Mat> tvecs,
                           HandEyeMethod method)
    -> Result<HandEyeResult>
{
    if (R_tool.size() < 3) {
        return Unexpected<std::string>(
            std::format("Need at least 3 relative motions (got {})", R_tool.size()));
    }

    // Convert camera rvecs to rotation matrices
    auto R_cam = rvecs_to_R(rvecs);

    if (R_cam.size() != R_tool.size()) {
        return Unexpected<std::string>(
            std::format("Mismatched data sizes: {} tool motions vs {} camera views",
                        R_tool.size(), R_cam.size()));
    }

    const double cond_num = compute_condition_number(R_tool);

    const std::array<HandEyeMethod, 4> all_methods = {
        HandEyeMethod::Tsai,
        HandEyeMethod::Park,
        HandEyeMethod::Horaud,
        HandEyeMethod::Daniilidis,
    };

    cv::Mat best_R, best_t;
    double best_reproj = std::numeric_limits<double>::max();
    HandEyeMethod best_method = HandEyeMethod::Tsai;

    auto try_method = [&](HandEyeMethod m) -> bool {
        cv::Mat R_m, t_m;
        const auto cv_flag = method_to_cv_flag(m);

        switch (mode_) {
        case HandEyeMode::EyeInHand: {
            std::vector<cv::Mat> R_tool_vec(R_tool.begin(), R_tool.end());
            std::vector<cv::Mat> t_tool_vec(t_tool.begin(), t_tool.end());
            std::vector<cv::Mat> R_cam_vec(R_cam.begin(), R_cam.end());
            std::vector<cv::Mat> tvecs_vec(tvecs.begin(), tvecs.end());
            cv::calibrateHandEye(R_tool_vec, t_tool_vec,
                                 R_cam_vec, tvecs_vec,
                                 R_m, t_m, cv_flag);
            break;
        }
        case HandEyeMode::EyeToHand: {
            std::vector<cv::Mat> R_cam_vec(R_cam.begin(), R_cam.end());
            std::vector<cv::Mat> tvecs_vec(tvecs.begin(), tvecs.end());
            std::vector<cv::Mat> R_tool_vec(R_tool.begin(), R_tool.end());
            std::vector<cv::Mat> t_tool_vec(t_tool.begin(), t_tool.end());
            cv::calibrateHandEye(R_cam_vec, tvecs_vec,
                                 R_tool_vec, t_tool_vec,
                                 R_m, t_m, cv_flag);
            break;
        }
        }

        // Validate OpenCV output — calibrateHandEye returns void and
        // can silently produce garbage with degenerate/noisy data
        if (R_m.empty() || t_m.empty()) {
            return false;
        }
        if (std::abs(cv::determinant(R_m) - 1.0) > 0.01) {
            return false;
        }

        const double err = compute_reproj_error(R_tool, t_tool, R_cam, tvecs,
                                                 R_m, t_m, mode_);

        if (err < best_reproj) {
            best_reproj = err;
            best_R = R_m;
            best_t = t_m;
            best_method = m;
        }
        return true;
    };

    bool any_valid = false;

    if (method == HandEyeMethod::Auto) {
        for (auto m : all_methods) {
            if (try_method(m)) any_valid = true;
        }
    } else {
        any_valid = try_method(method);
    }


    if (!any_valid) {
        return Unexpected<std::string>(
            method == HandEyeMethod::Auto
                ? "All hand-eye methods failed to produce a valid result"
                : std::format("Hand-eye method '{}' failed to produce a valid result",
                              method_to_string(method)));
    }

    return HandEyeResult{
        .R                = best_R,
        .t                = best_t,
        .mode             = mode_,
        .reproj_error     = best_reproj,
        .method           = method_to_string(best_method),
        .condition_number = cond_num,
        .used_method      = best_method,
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
    fs << "condition_number" << condition_number;

    fs.release();
}

} // namespace rm::calib
