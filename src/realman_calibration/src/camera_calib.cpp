#include "realman_calibration/camera_calib.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <cmath>
#include "realman_calibration/format_polyfill.hpp"
#include <vector>

namespace rm::calib {

// ──────────────────────────────────────────────────────────────
// CameraCalibrator
// ──────────────────────────────────────────────────────────────

CameraCalibrator::CameraCalibrator(const CameraCalibInput& input)
    : input_{input}
{}

auto CameraCalibrator::compute() -> Result<CameraCalibResult> {
    // ── build 3D object points for one chessboard pose ──
    std::vector<cv::Point3f> obj;
    obj.reserve(input_.board_size.width * input_.board_size.height);
    for (int r = 0; r < input_.board_size.height; ++r) {
        for (int c = 0; c < input_.board_size.width; ++c) {
            obj.emplace_back(static_cast<float>(c) * input_.square_size_m,
                             static_cast<float>(r) * input_.square_size_m,
                             0.0f);
        }
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    cv::Size image_size;

    // ── detect corners on each image ──
    for (const auto& img : input_.images) {
        if (img.empty() || img.type() != CV_8UC1) continue;

        image_size = img.size();

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, input_.board_size, corners);

        if (!found) continue;             // silently skip

        cv::cornerSubPix(img, corners, cv::Size(5, 5),
                         cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
                                          30, 0.1));

        object_points.push_back(obj);
        image_points.push_back(corners);
    }

    // ── require at least 3 valid images ──
    if (object_points.size() < 3) {
        return Unexpected<std::string>(
            std::format("Need at least 3 valid calibration images (got {})",
                        object_points.size()));
    }

    // ── calibrate ──
    cv::Mat K, dist;
    std::vector<cv::Mat> rvecs, tvecs;

    cv::calibrateCamera(object_points, image_points, image_size,
                        K, dist, rvecs, tvecs);

    // ── compute reprojection error ──
    double total_error = 0.0;
    int total_points = 0;

    for (auto i = 0uz; i < object_points.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i], K, dist, projected);

        auto err = cv::norm(image_points[i], projected, cv::NORM_L2);
        total_error += err * err;
        total_points += static_cast<int>(object_points[i].size());
    }

    double reproj_error = std::sqrt(total_error / static_cast<double>(total_points));

    return CameraCalibResult{
        .K            = K,
        .dist         = dist,
        .rvecs        = std::move(rvecs),
        .tvecs        = std::move(tvecs),
        .reproj_error = reproj_error,
        .images_used  = static_cast<int>(object_points.size()),
    };
}

} // namespace rm::calib
