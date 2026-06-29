#include "realman_calibration/camera_calib.hpp"
#include "synthetic_board.h"

#include <gtest/gtest.h>
#include <cmath>

namespace {

// ── Ground-truth intrinsics for synthetic image generation ──
constexpr int   kWidth  = 640;
constexpr int   kHeight = 480;
constexpr float kFx     = 800.0f;
constexpr float kFy     = 800.0f;
constexpr float kCx     = 320.0f;
constexpr float kCy     = 240.0f;
constexpr int   kBoardW = 8;   // inner corners (cols)
constexpr int   kBoardH = 5;   // inner corners (rows)
constexpr float kSquareM = 0.030f;

cv::Mat ground_truth_K() {
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        kFx, 0.0, kCx,
        0.0, kFy, kCy,
        0.0, 0.0, 1.0);
    return K;
}

cv::Mat ground_truth_dist() {
    // mild radial + tangential distortion
    cv::Mat d = (cv::Mat_<double>(1, 5) << 0.1, -0.05, 0.0, 0.0, 0.0);
    return d;
}

std::vector<cv::Mat> make_images(int count, cv::Size img_size = {kWidth, kHeight}) {
    return rm::calib::test::generate_chessboard_images(
        count,
        cv::Size{kBoardW, kBoardH},
        kSquareM,
        ground_truth_K(),
        ground_truth_dist(),
        img_size);
}

// ─────────────────────────────────────────────────────────
// Test 1 – happy path
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, HappyPath) {
    auto images = make_images(10);
    ASSERT_EQ(images.size(), 10u);

    rm::calib::CameraCalibInput input;
    input.images       = std::move(images);
    input.board_size   = cv::Size{kBoardW, kBoardH};
    input.square_size_m = kSquareM;

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& r = result.value();
    auto gt_K = ground_truth_K();

    // reprojection error should be tiny on synthetic data
    EXPECT_LT(r.reproj_error, 0.5);

    // at least 3 of 10 images should succeed (all 10 should, practically)
    EXPECT_GE(r.images_used, 3);

    // K within 1 % of ground truth on each diagonal element
    for (int i = 0; i < 3; ++i) {
        double diff = std::abs(r.K.at<double>(i, i) - gt_K.at<double>(i, i));
        EXPECT_LT(diff, 0.01 * gt_K.at<double>(i, i))
            << "K(" << i << "," << i << ") off by " << diff;
    }
}

// ─────────────────────────────────────────────────────────
// Test 2 – insufficient images (< 3)
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, InsufficientImages) {
    auto images = make_images(2);
    ASSERT_EQ(images.size(), 2u);

    rm::calib::CameraCalibInput input;
    input.images       = std::move(images);
    input.board_size   = cv::Size{kBoardW, kBoardH};
    input.square_size_m = kSquareM;

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    ASSERT_FALSE(result.has_value());
    std::string msg = result.error();
    bool has_key_phrase = (msg.find("at least 3") != std::string::npos ||
                           msg.find("Need at least") != std::string::npos);
    EXPECT_TRUE(has_key_phrase) << "Unexpected error: " << msg;
}

// ─────────────────────────────────────────────────────────
// Test 3 – empty input
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, EmptyInput) {
    rm::calib::CameraCalibInput input;
    input.images.clear();
    input.board_size   = cv::Size{kBoardW, kBoardH};
    input.square_size_m = kSquareM;

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    EXPECT_FALSE(result.has_value());
}

// ─────────────────────────────────────────────────────────
// Test 4 – mixed resolution edge
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, MixedResolution) {
    // generate half at 800×600, half at 640×480
    auto big   = make_images(5, cv::Size{800, 600});
    auto small = make_images(5, cv::Size{640, 480});

    std::vector<cv::Mat> mixed;
    mixed.insert(mixed.end(), big.begin(), big.end());
    mixed.insert(mixed.end(), small.begin(), small.end());
    ASSERT_EQ(mixed.size(), 10u);

    rm::calib::CameraCalibInput input;
    input.images       = std::move(mixed);
    input.board_size   = cv::Size{kBoardW, kBoardH};
    input.square_size_m = kSquareM;

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& r = result.value();

    // All 10 images should be detected despite differing resolutions
    EXPECT_EQ(r.images_used, 10);
}

}  // namespace
