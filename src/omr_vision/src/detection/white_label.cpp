#include "omr_vision/detection/white_label.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace omr_vision::detection {
namespace {

[[nodiscard]] auto objectPoints(const WhiteLabelConfig& cfg) -> std::vector<cv::Point3f> {
    const float hw = static_cast<float>(cfg.width_m * 0.5);
    const float hh = static_cast<float>(cfg.height_m * 0.5);
    return {
        {-hw, hh, 0.0F},
        {hw, hh, 0.0F},
        {hw, -hh, 0.0F},
        {-hw, -hh, 0.0F},
    };
}

[[nodiscard]] auto orderCorners(const cv::Point2f pts[4]) -> std::array<cv::Point2f, 4> {
    std::array<cv::Point2f, 4> corners = {pts[0], pts[1], pts[2], pts[3]};
    std::sort(corners.begin(), corners.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        if (std::abs(a.y - b.y) > 1.0F) {
            return a.y < b.y;
        }
        return a.x < b.x;
    });
    if (corners[0].x > corners[1].x) {
        std::swap(corners[0], corners[1]);
    }
    if (corners[2].x > corners[3].x) {
        std::swap(corners[2], corners[3]);
    }
    return {corners[0], corners[1], corners[3], corners[2]};
}

[[nodiscard]] auto orderCorners(const std::vector<cv::Point>& pts) -> std::array<cv::Point2f, 4> {
    cv::Point2f raw[4] = {pts[0], pts[1], pts[2], pts[3]};
    return orderCorners(raw);
}

[[nodiscard]] auto meanReprojError(const std::vector<cv::Point3f>& obj,
                                   const std::vector<cv::Point2f>& img, const cv::Vec3d& rvec,
                                   const cv::Vec3d& tvec, const cv::Mat& K, const cv::Mat& dist)
    -> double {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj, rvec, tvec, K, dist, projected);
    double sum = 0.0;
    for (size_t i = 0; i < img.size(); ++i) {
        const double dx = static_cast<double>(img[i].x - projected[i].x);
        const double dy = static_cast<double>(img[i].y - projected[i].y);
        sum += std::sqrt(dx * dx + dy * dy);
    }
    return sum / static_cast<double>(img.size());
}

[[nodiscard]] auto rvecTvecToHomogeneous(const cv::Vec3d& rvec, const cv::Vec3d& tvec) -> cv::Mat {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = tvec[0];
    T.at<double>(1, 3) = tvec[1];
    T.at<double>(2, 3) = tvec[2];
    return T;
}

[[nodiscard]] auto angleDeg(const cv::Point& a, const cv::Point& b, const cv::Point& c) -> double {
    const cv::Point2f ab(static_cast<float>(a.x - b.x), static_cast<float>(a.y - b.y));
    const cv::Point2f cb(static_cast<float>(c.x - b.x), static_cast<float>(c.y - b.y));
    const double dot = ab.dot(cb);
    const double norm = cv::norm(ab) * cv::norm(cb);
    if (norm < 1e-6) {
        return 180.0;
    }
    const double cosang = std::clamp(dot / norm, -1.0, 1.0);
    return std::acos(cosang) * 180.0 / CV_PI;
}

[[nodiscard]] auto isValidQuad(const std::vector<cv::Point>& approx) -> bool {
    if (approx.size() != 4 || !cv::isContourConvex(approx)) {
        return false;
    }
    for (int i = 0; i < 4; ++i) {
        const double ang = angleDeg(approx[(i + 3) % 4], approx[i], approx[(i + 1) % 4]);
        if (ang < 55.0 || ang > 125.0) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] auto buildWhiteMask(const cv::Mat& gray, const WhiteLabelConfig& cfg) -> cv::Mat {
    cv::Mat work = gray;
    cv::Mat clahe_img;
    if (cfg.use_clahe) {
        static const auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(gray, clahe_img);
        work = clahe_img;
    }

    cv::Mat combined = cv::Mat::zeros(work.size(), CV_8UC1);
    std::vector<int> thresholds;
    thresholds.push_back(cfg.binary_threshold);
    if (cfg.use_otsu) {
        cv::Mat tmp;
        const double otsu = cv::threshold(work, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        thresholds.push_back(static_cast<int>(std::lround(otsu)));
    }
    if (cfg.use_multi_threshold) {
        for (int t : {150, 180, 210, 240}) {
            thresholds.push_back(t);
        }
    }
    std::sort(thresholds.begin(), thresholds.end());
    thresholds.erase(std::unique(thresholds.begin(), thresholds.end()), thresholds.end());

    for (int t : thresholds) {
        cv::Mat mask;
        cv::threshold(work, mask, t, 255, cv::THRESH_BINARY);
        cv::bitwise_or(combined, mask, combined);
    }

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(combined, combined, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(combined, combined, cv::MORPH_CLOSE, kernel);
    return combined;
}

/// Score a candidate as a plain bright rectangular label after perspective unwarp.
[[nodiscard]] auto contentScore(const cv::Mat& bgr, const std::array<cv::Point2f, 4>& corners,
                                const WhiteLabelConfig& cfg) -> double {
    const int short_px = std::max(8, cfg.warp_short_px);
    const int long_px = std::max(16, cfg.warp_long_px);
    // Destination: long side vertical to match height_m > width_m.
    std::vector<cv::Point2f> dst = {
        {0.F, 0.F},
        {static_cast<float>(short_px - 1), 0.F},
        {static_cast<float>(short_px - 1), static_cast<float>(long_px - 1)},
        {0.F, static_cast<float>(long_px - 1)},
    };
    std::vector<cv::Point2f> src = {corners[0], corners[1], corners[2], corners[3]};
    const cv::Mat H = cv::getPerspectiveTransform(src, dst);
    cv::Mat warp;
    cv::warpPerspective(bgr, warp, H, cv::Size(short_px, long_px));

    cv::Mat gray;
    cv::cvtColor(warp, gray, cv::COLOR_BGR2GRAY);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray, mean, stddev);
    const double brightness = mean[0] / 255.0;
    const double uniformity = 1.0 - std::clamp(stddev[0] / 70.0, 0.0, 1.0);

    // Interior should stay bright (plain white paper).
    const int margin_x = std::max(1, short_px / 10);
    const int margin_y = std::max(1, long_px / 10);
    const cv::Rect inner(margin_x, margin_y, short_px - 2 * margin_x, long_px - 2 * margin_y);
    const double inner_mean = cv::mean(gray(inner))[0] / 255.0;

    if (brightness < 0.45 || inner_mean < 0.50) {
        return 0.0;
    }
    return std::clamp(0.45 * brightness + 0.35 * uniformity + 0.20 * inner_mean, 0.0, 1.0);
}

[[nodiscard]] auto passesScaleGate(double long_side_px, const WhiteLabelConfig& cfg) -> bool {
    if (cfg.focal_length_px <= 1.0 || cfg.height_m <= 0.0) {
        return true;
    }
    // Expected long-side pixels at z_min..z_max; allow closer than z_min (large labels).
    const double expected_at_zmin =
        cfg.focal_length_px * cfg.height_m / std::max(cfg.z_min_m, 1e-3);
    const double expected_at_zmax =
        cfg.focal_length_px * cfg.height_m / std::max(cfg.z_max_m, 1e-3);
    const double lo = expected_at_zmax * 0.35;
    const double hi = expected_at_zmin * 4.0;  // ~z_min/4 still accepted if fully in view
    return long_side_px >= lo && long_side_px <= hi;
}

}  // namespace

auto detectWhiteLabel(const cv::Mat& bgr, const WhiteLabelConfig& cfg, cv::Mat* debug_mask)
    -> std::optional<WhiteLabelDetection> {
    if (bgr.empty() || bgr.type() != CV_8UC3) {
        return std::nullopt;
    }

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);

    cv::Mat mask = buildWhiteMask(gray, cfg);
    if (debug_mask != nullptr) {
        *debug_mask = mask.clone();
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double target_aspect =
        std::max(cfg.height_m, cfg.width_m) / std::min(cfg.height_m, cfg.width_m);
    const double aspect_min = target_aspect * (1.0 - cfg.aspect_ratio_tol);
    const double aspect_max = target_aspect * (1.0 + cfg.aspect_ratio_tol);

    std::optional<WhiteLabelDetection> best;
    double best_score = -1.0;

    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < cfg.min_area_px) {
            continue;
        }

        const cv::RotatedRect rect = cv::minAreaRect(contour);
        const double side_a = static_cast<double>(rect.size.width);
        const double side_b = static_cast<double>(rect.size.height);
        if (side_a < 1.0 || side_b < 1.0) {
            continue;
        }
        const double long_side = std::max(side_a, side_b);
        const double short_side = std::min(side_a, side_b);
        const double aspect = long_side / short_side;
        if (aspect < aspect_min || aspect > aspect_max) {
            continue;
        }

        const double rect_area = side_a * side_b;
        const double rectangularity = area / std::max(rect_area, 1.0);
        if (rectangularity < cfg.min_rectangularity) {
            continue;
        }
        if (!passesScaleGate(long_side, cfg)) {
            continue;
        }

        std::array<cv::Point2f, 4> corners{};
        std::vector<cv::Point> approx;
        const double peri = cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, 0.04 * peri, true);
        if (isValidQuad(approx)) {
            corners = orderCorners(approx);
        } else {
            cv::Point2f raw[4];
            rect.points(raw);
            corners = orderCorners(raw);
        }

        const double content = contentScore(bgr, corners, cfg);
        if (content < cfg.min_content_score) {
            continue;
        }

        const double aspect_err = std::abs(aspect - target_aspect) / target_aspect;
        const double score = area * (1.0 - aspect_err) * rectangularity * (0.4 + 0.6 * content);
        if (score <= best_score) {
            continue;
        }

        WhiteLabelDetection det;
        det.corners_img = corners;
        det.rect = rect;
        det.content_score = content;
        det.confidence = std::clamp((1.0 - aspect_err) * content, 0.0, 1.0);
        best = det;
        best_score = score;
    }

    return best;
}

namespace {

[[nodiscard]] auto detectionCenter(const WhiteLabelDetection& det) -> cv::Point2d {
    double cx = 0.0;
    double cy = 0.0;
    for (const auto& p : det.corners_img) {
        cx += p.x;
        cy += p.y;
    }
    return {cx * 0.25, cy * 0.25};
}

}  // namespace

WhiteLabelTemporalConfirm::WhiteLabelTemporalConfirm(int required_frames,
                                                     double max_center_shift_px)
    : required_frames_(std::max(1, required_frames)),
      max_center_shift_px_(std::max(1.0, max_center_shift_px)) {}

void WhiteLabelTemporalConfirm::reset() {
    streak_ = 0;
    last_center_ = {0.0, 0.0};
}

auto WhiteLabelTemporalConfirm::update(const std::optional<WhiteLabelDetection>& detection)
    -> std::optional<WhiteLabelDetection> {
    if (!detection.has_value()) {
        reset();
        return std::nullopt;
    }

    const cv::Point2d center = detectionCenter(*detection);
    if (streak_ > 0) {
        const double dx = center.x - last_center_.x;
        const double dy = center.y - last_center_.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > max_center_shift_px_) {
            streak_ = 1;
            last_center_ = center;
            return std::nullopt;
        }
        ++streak_;
    } else {
        streak_ = 1;
    }
    last_center_ = center;

    if (streak_ < required_frames_) {
        return std::nullopt;
    }
    return detection;
}

auto estimateWhiteLabelPoseFromDetection(const WhiteLabelDetection& detection,
                                         const camera::CameraIntrinsics& color_intrinsics,
                                         const WhiteLabelConfig& cfg)
    -> std::optional<WhiteLabelPose> {
    if (color_intrinsics.K.empty() || color_intrinsics.K.rows != 3 ||
        color_intrinsics.K.cols != 3) {
        return std::nullopt;
    }

    const auto obj = objectPoints(cfg);
    std::vector<cv::Point2f> img = {detection.corners_img[0], detection.corners_img[1],
                                    detection.corners_img[2], detection.corners_img[3]};

    cv::Mat dist = color_intrinsics.dist_coeff;
    if (dist.empty()) {
        dist = cv::Mat::zeros(1, 5, CV_64F);
    }

    const int pnp_flag = (cfg.pnp_method == WhiteLabelPnpMethod::Iterative) ? cv::SOLVEPNP_ITERATIVE
                                                                            : cv::SOLVEPNP_IPPE;

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    const bool ok = cv::solvePnP(obj, img, color_intrinsics.K, dist, rvec, tvec,
                                 /*useExtrinsicGuess=*/false, pnp_flag);
    if (!ok) {
        return std::nullopt;
    }

    cv::solvePnP(obj, img, color_intrinsics.K, dist, rvec, tvec, /*useExtrinsicGuess=*/true,
                 cv::SOLVEPNP_ITERATIVE);

    const double reproj = meanReprojError(obj, img, rvec, tvec, color_intrinsics.K, dist);
    if (reproj > cfg.max_reproj_error_px) {
        return std::nullopt;
    }

    WhiteLabelPose pose;
    pose.rvec = rvec;
    pose.tvec = tvec;
    pose.T_cam_label = rvecTvecToHomogeneous(rvec, tvec);
    pose.reproj_error_px = reproj;
    pose.detection = detection;
    return pose;
}

auto estimateWhiteLabelPose(const cv::Mat& bgr, const camera::CameraIntrinsics& color_intrinsics,
                            const WhiteLabelConfig& cfg) -> std::optional<WhiteLabelPose> {
    if (color_intrinsics.K.empty() || color_intrinsics.K.rows != 3 ||
        color_intrinsics.K.cols != 3) {
        return std::nullopt;
    }

    WhiteLabelConfig det_cfg = cfg;
    if (det_cfg.focal_length_px <= 0.0) {
        det_cfg.focal_length_px = color_intrinsics.fy();
    }

    auto detection = detectWhiteLabel(bgr, det_cfg);
    if (!detection.has_value()) {
        return std::nullopt;
    }
    return estimateWhiteLabelPoseFromDetection(*detection, color_intrinsics, cfg);
}

}  // namespace omr_vision::detection
