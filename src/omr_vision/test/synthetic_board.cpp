#include "synthetic_board.h"

#include <cmath>

#include <algorithm>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace omr_vision::test {
// NOLINTBEGIN(performance-unnecessary-value-param,readability-math-missing-parentheses,google-readability-braces-around-statements,readability-braces-around-statements,cppcoreguidelines-pro-bounds-constant-array-index,modernize-use-std-numbers)

namespace {

/// Generate diverse camera poses for synthetic image generation.
///
/// Rotation angles spread linearly from `min_angle_deg` to ~70°
/// across the sequence.  Translation depth is chosen so the board
/// fills ~50—70 % of the shorter image dimension, with small random
/// x/y offsets that keep every grid vertex inside the frame.
///
/// @return (rvecs, tvecs) — one Rodrigues rotation vector and one
///         translation vector per image, type CV_64FC1.
auto generate_poses(int count, cv::Size board_size, float square_m, cv::Size image_size,
                    const cv::Mat& K, float min_angle_deg = 15.0F)
    -> std::pair<std::vector<cv::Mat>, std::vector<cv::Mat>> {
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    rvecs.reserve(static_cast<std::size_t>(count));
    tvecs.reserve(static_cast<std::size_t>(count));

    auto& rng = cv::theRNG();

    // Compute a safe depth: board fills ~60 % of the smaller image dim.
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double f_min = std::min(fx, fy);
    // Physical board extent = (inner corners + 1) squares × square size
    double board_w = (board_size.width + 1) * static_cast<double>(square_m);
    double board_h = (board_size.height + 1) * static_cast<double>(square_m);
    double board_max_dim = std::max(board_w, board_h);
    int image_min_dim = std::min(image_size.width, image_size.height);
    double z_base = f_min * board_max_dim / (0.5 * image_min_dim);

    // Board is centred at origin by build_grid_3d, so translation = (0,0,z)
    // with small random jitter in x/y to create viewpoint diversity.
    for (int i = 0; i < count; ++i) {
        // ── rotation: spread angles linearly from min → ~70° ──
        float frac = count > 1 ? static_cast<float>(i) / static_cast<float>(count - 1) : 0.5F;
        float angle_deg = min_angle_deg + (frac * (55.0F - min_angle_deg));
        angle_deg += static_cast<float>(rng.gaussian(4.0));
        angle_deg = std::max(angle_deg, 5.0F);
        float angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0F;

        cv::Vec3f axis(rng.uniform(-1.0F, 1.0F), rng.uniform(-1.0F, 1.0F),
                       rng.uniform(-1.0F, 1.0F));
        float nrm = cv::norm(axis);
        if (nrm < 1e-6F) {
            axis = cv::Vec3f(0.0F, 0.0F, 1.0F);
        } else {
            axis /= nrm;
        }

        rvecs.push_back(cv::Mat(axis * angle_rad).clone());

        // ── translation: jitter depth ±15 % and small x/y wander ──
        double z = z_base * (0.85 + rng.uniform(0.0F, 0.3F));
        double mx = (image_size.width / fx) * z * 0.05;
        double my = (image_size.height / fy) * z * 0.05;
        tvecs.push_back((cv::Mat_<double>(3, 1) << rng.uniform(-mx, mx), rng.uniform(-my, my), z));
    }

    return {std::move(rvecs), std::move(tvecs)};
}

/// Build a flat grid of 3-D points covering every vertex of the
/// chessboard (including outer border).  A board with `board_size`
/// inner corners has (board_size.width+1) × (board_size.height+1)
/// squares, requiring (board_size.width+2) × (board_size.height+2)
/// vertices.
///
/// The grid is centred at the origin (Z=0).
auto build_grid_3d(cv::Size board_size, float square_m) -> std::vector<cv::Point3f> {
    int cols = board_size.width + 2;  // include both outer edges
    int rows = board_size.height + 2;
    float half_w = 0.5F * static_cast<float>(board_size.width + 1) * square_m;
    float half_h = 0.5F * static_cast<float>(board_size.height + 1) * square_m;
    std::vector<cv::Point3f> pts;
    pts.reserve(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            pts.emplace_back((-half_w + static_cast<float>(c) * square_m),
                             (-half_h + static_cast<float>(r) * square_m), 0.0F);
        }
    }
    return pts;
}

/// Render a perspective-distorted chessboard onto a white canvas by
/// projecting every grid vertex and filling alternating squares with
/// black / white polygons.  Each black square is expanded by 0.5 px
/// outward from its centroid so that adjacent squares touch with no
/// gaps — required for `cv::findChessboardCorners` to succeed.
void render_chessboard(cv::Mat& canvas, const std::vector<cv::Point3f>& grid_3d,
                       const cv::Mat& rvec, const cv::Mat& tvec, const cv::Mat& K,
                       const cv::Mat& dist, cv::Size board_size) {
    // Project all grid vertices → image plane (floating-point)
    std::vector<cv::Point2f> projected;
    cv::projectPoints(grid_3d, rvec, tvec, K, dist, projected);

    int cols = board_size.width + 2;  // matches build_grid_3d
    int sq_cols = board_size.width + 1;
    int sq_rows = board_size.height + 1;
    for (int r = 0; r < sq_rows; ++r) {
        for (int c = 0; c < sq_cols; ++c) {
            if (((r + c) % 2) == 0) continue;  // white square — background already set

            // Gather the four projected corners for this square
            std::array<cv::Point2f, 4> corners = {
                projected[r * cols + c],
                projected[r * cols + (c + 1)],
                projected[(r + 1) * cols + (c + 1)],
                projected[(r + 1) * cols + c],
            };

            // Expand slightly outward from centroid to close sub-pixel gaps
            cv::Point2f centroid(0, 0);
            for (const auto& p : corners)
                centroid += p;
            centroid *= (1.0F / 4.0F);

            std::array<cv::Point, 4> poly;
            for (int k = 0; k < 4; ++k) {
                cv::Point2f dir = corners[k] - centroid;
                float len = cv::norm(dir);
                if (len > 1e-6F) dir *= (1.0F / len);
                cv::Point2f expanded = corners[k] + dir * 0.5F;
                poly[k] = cv::Point(static_cast<int>(std::round(expanded.x)),
                                    static_cast<int>(std::round(expanded.y)));
            }

            cv::fillConvexPoly(canvas, poly, cv::Scalar(0), cv::LINE_8);
        }
    }
}

}  // namespace

// ─────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────

std::vector<cv::Mat> generate_chessboard_images(int count, cv::Size board_size, float square_m,
                                                cv::Mat K, cv::Mat dist, cv::Size image_size) {
    auto grid_3d = build_grid_3d(board_size, square_m);
    auto [rvecs, tvecs] = generate_poses(count, board_size, square_m, image_size, K);

    std::vector<cv::Mat> images;
    images.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i) {
        cv::Mat img(image_size, CV_8UC1, cv::Scalar(255));
        render_chessboard(img, grid_3d, rvecs[i], tvecs[i], K, dist, board_size);
        images.push_back(std::move(img));
    }

    return images;
}

std::vector<cv::Mat> generate_charuco_images(int count, cv::Size squares_xy, float square_len,
                                             float marker_len, cv::aruco::Dictionary dict,
                                             cv::Mat K, cv::Mat dist, cv::Size image_size) {
    // ── create board & render clean pattern ──
    // CharucoBoard::create expects Ptr<Dictionary> — wrap the by-value param.
    auto dict_ptr = cv::Ptr<cv::aruco::Dictionary>(new cv::aruco::Dictionary(dict));
    auto board = cv::aruco::CharucoBoard::create(squares_xy.width, squares_xy.height, square_len,
                                                 marker_len, dict_ptr);

    int px_per_sq = 200;
    cv::Size board_px(squares_xy.width * px_per_sq, squares_xy.height * px_per_sq);

    cv::Mat board_color;
    cv::aruco::drawPlanarBoard(board, board_px, board_color, /*margin=*/0,
                               /*borderBits=*/1);
    // OpenCV 4.x drawPlanarBoard outputs a grayscale image directly;
    // only convert if it came back as BGR (3-channel).
    cv::Mat board_gray;
    if (board_color.channels() == 3) {
        cv::cvtColor(board_color, board_gray, cv::COLOR_BGR2GRAY);
    } else {
        board_gray = board_color;
    }

    // ── physical corners of the board (Z=0 plane) ──
    std::vector<cv::Point3f> corners_3d = {
        {0.0F, 0.0F, 0.0F},
        {static_cast<float>(squares_xy.width) * square_len, 0.0F, 0.0F},
        {static_cast<float>(squares_xy.width) * square_len,
         static_cast<float>(squares_xy.height) * square_len, 0.0F},
        {0.0F, static_cast<float>(squares_xy.height) * square_len, 0.0F},
    };

    // ── pixel corners of the rendered board ──
    std::vector<cv::Point2f> src_pts = {
        {0.0F, 0.0F},
        {static_cast<float>(board_px.width - 1), 0.0F},
        {static_cast<float>(board_px.width - 1), static_cast<float>(board_px.height - 1)},
        {0.0F, static_cast<float>(board_px.height - 1)},
    };

    auto [rvecs, tvecs] = generate_poses(count, squares_xy, square_len, image_size, K);

    std::vector<cv::Mat> images;
    images.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i) {
        std::vector<cv::Point2f> dst_pts;
        cv::projectPoints(corners_3d, rvecs[i], tvecs[i], K, dist, dst_pts);

        cv::Mat H = cv::getPerspectiveTransform(src_pts, dst_pts);

        cv::Mat warped;
        cv::warpPerspective(board_gray, warped, H, image_size, cv::INTER_LINEAR,
                            cv::BORDER_CONSTANT, cv::Scalar(255));
        images.push_back(warped);
    }

    return images;
}

}  // namespace omr_vision::test
// NOLINTEND(performance-unnecessary-value-param,readability-math-missing-parentheses,google-readability-braces-around-statements,readability-braces-around-statements,cppcoreguidelines-pro-bounds-constant-array-index,modernize-use-std-numbers)
