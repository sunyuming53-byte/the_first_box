#include "realman_calibration/camera_calib.hpp"
#include "realman_calibration/format_polyfill.hpp"

#include <cmath>

#include <array>
#include <vector>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace rm::calib {

// ──────────────────────────────────────────────────────────────
// CameraCalibrator
// ──────────────────────────────────────────────────────────────

CameraCalibrator::CameraCalibrator(const CameraCalibInput& input) : input_{input} {}  // NOLINT(modernize-pass-by-value)

auto CameraCalibrator::compute() -> Result<CameraCalibResult> {  // NOLINT(readability-function-size)
    if (input_.board_type == BoardType::Chessboard) {
        // ── build 3D object points for one chessboard pose ──
        std::vector<cv::Point3f> obj;
        obj.reserve(input_.board_size.width * input_.board_size.height);
        for (int r = 0; r < input_.board_size.height; ++r) {
            for (int c = 0; c < input_.board_size.width; ++c) {
                obj.emplace_back(static_cast<float>(c) * input_.square_size_m,
                                 static_cast<float>(r) * input_.square_size_m, 0.0F);
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

            if (!found) continue;  // silently skip

            cv::cornerSubPix(
                img, corners, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

            object_points.push_back(obj);
            image_points.push_back(corners);
        }

        // ── require at least 3 valid images ──
        if (object_points.size() < 3) {
            return Unexpected<std::string>(std::format(
                "Need at least 3 valid calibration images (got {})", object_points.size()));
        }

        // ── calibrate ──
        cv::Mat K;
        cv::Mat dist;
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;

        cv::calibrateCamera(object_points, image_points, image_size, K, dist, rvecs, tvecs);

        // ── compute reprojection error ──
        double total_error = 0.0;
        int total_points = 0;

        for (auto i = 0UZ; i < object_points.size(); ++i) {
            std::vector<cv::Point2f> projected;
            cv::projectPoints(object_points[i], rvecs[i], tvecs[i], K, dist, projected);

            auto err = cv::norm(image_points[i], projected, cv::NORM_L2);
            total_error += err * err;
            total_points += static_cast<int>(object_points[i].size());
        }

        double reproj_error = std::sqrt(total_error / static_cast<double>(total_points));

        return CameraCalibResult{
            .K = K,
            .dist = dist,
            .rvecs = std::move(rvecs),
            .tvecs = std::move(tvecs),
            .reproj_error = reproj_error,
            .images_used = static_cast<int>(object_points.size()),
        };
    }

    if (input_.board_type == BoardType::Charuco) {
        // ── create Charuco board ──
        cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
            input_.board_size.width, input_.board_size.height, input_.square_size_m,
            input_.marker_size_m, cv::aruco::getPredefinedDictionary(input_.dictionary_id));

        cv::Ptr<cv::aruco::Dictionary> dict =
            cv::aruco::getPredefinedDictionary(input_.dictionary_id);

        std::vector<std::vector<cv::Point2f>> all_charuco_corners;
        std::vector<std::vector<int>> all_charuco_ids;
        cv::Size image_size;

        // ── detect Charuco corners on each image ──
        for (const auto& img : input_.images) {
            if (img.empty() || img.type() != CV_8UC1) continue;

            image_size = img.size();

            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            cv::aruco::detectMarkers(img, dict, marker_corners, marker_ids);

            if (marker_ids.empty()) continue;

            std::vector<cv::Point2f> charuco_corners;
            std::vector<int> charuco_ids;
            cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, img, board,
                                                 charuco_corners, charuco_ids);

            if (charuco_ids.size() < 4) continue;

            all_charuco_corners.push_back(std::move(charuco_corners));
            all_charuco_ids.push_back(std::move(charuco_ids));
        }

        // ── require at least 3 valid images ──
        if (all_charuco_corners.size() < 3) {
            return Unexpected<std::string>(std::format(
                "Need at least 3 valid calibration images (got {})", all_charuco_corners.size()));
        }

        // ── calibrate ──
        cv::Mat K;
        cv::Mat dist;
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;

        // Compute 3D object points from corner IDs using board geometry.
        // CharucoBoard corners are row-major: corners_x = squaresX - 1.
        int corners_per_row = input_.board_size.width - 1;
        std::vector<std::vector<cv::Point3f>> all_obj_pts;
        std::vector<std::vector<cv::Point2f>> all_img_pts;
        for (size_t i = 0; i < all_charuco_corners.size(); ++i) {
            std::vector<cv::Point3f> obj_pts;
            obj_pts.reserve(all_charuco_ids[i].size());
            for (int id : all_charuco_ids[i]) {
                int row = id / corners_per_row;
                int col = id % corners_per_row;
                obj_pts.emplace_back(static_cast<float>(col + 1) * input_.square_size_m,
                                     static_cast<float>(row + 1) * input_.square_size_m, 0.0F);
            }
            all_obj_pts.push_back(std::move(obj_pts));
            all_img_pts.push_back(all_charuco_corners[i]);
        }

        int flags = cv::CALIB_FIX_ASPECT_RATIO;
        // Provide a reasonable initial K guess: fx=fy=max(image dim), cx=w/2, cy=h/2
        K = (cv::Mat_<double>(3, 3)
                 << static_cast<double>(std::max(image_size.width, image_size.height)),
             0.0, static_cast<double>(image_size.width) / 2.0, 0.0,
             static_cast<double>(std::max(image_size.width, image_size.height)),
             static_cast<double>(image_size.height) / 2.0, 0.0, 0.0, 1.0);
        flags |= cv::CALIB_USE_INTRINSIC_GUESS;
        cv::calibrateCamera(all_obj_pts, all_img_pts, image_size, K, dist, rvecs, tvecs, flags);

        // ── compute reprojection error ──
        double total_error = 0.0;
        int total_points = 0;

        for (auto i = 0UZ; i < all_charuco_corners.size(); ++i) {
            std::vector<cv::Point2f> projected;
            cv::projectPoints(all_obj_pts[i], rvecs[i], tvecs[i], K, dist, projected);

            auto err = cv::norm(all_charuco_corners[i], projected, cv::NORM_L2);
            total_error += err * err;
            total_points += static_cast<int>(all_obj_pts[i].size());
        }

        double reproj_error = std::sqrt(total_error / static_cast<double>(total_points));

        return CameraCalibResult{
            .K = K,
            .dist = dist,
            .rvecs = std::move(rvecs),
            .tvecs = std::move(tvecs),
            .reproj_error = reproj_error,
            .images_used = static_cast<int>(all_charuco_corners.size()),
        };
    }

    return Unexpected<std::string>("Unknown board type");
}

}  // namespace rm::calib
