#include "realman_calibration/camera_calib.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include "synthetic_board.h"
#include <opencv2/aruco/charuco.hpp>
// NOLINTBEGIN(performance-unnecessary-value-param,performance-unnecessary-copy-initialization)

namespace {

// ── Ground-truth intrinsics for synthetic image generation ──
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr float kFx = 800.0F;
constexpr float kFy = 800.0F;
constexpr float kCx = 320.0F;
constexpr float kCy = 240.0F;
constexpr int kBoardW = 8;  // inner corners (cols)
constexpr int kBoardH = 5;  // inner corners (rows)
constexpr float kSquareM = 0.030F;

cv::Mat ground_truth_K() {
    cv::Mat K = (cv::Mat_<double>(3, 3) << kFx, 0.0, kCx, 0.0, kFy, kCy, 0.0, 0.0, 1.0);
    return K;
}

cv::Mat ground_truth_dist() {
    // mild radial + tangential distortion
    cv::Mat d = (cv::Mat_<double>(1, 5) << 0.1, -0.05, 0.0, 0.0, 0.0);
    return d;
}

std::vector<cv::Mat> make_images(int count, cv::Size img_size = {kWidth, kHeight}) {
    return rm::calib::test::generate_chessboard_images(count, cv::Size{kBoardW, kBoardH}, kSquareM,
                                                       ground_truth_K(), ground_truth_dist(),
                                                       img_size);
}

// ─────────────────────────────────────────────────────────
// Test 1 – happy path
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, HappyPath) {
    auto images = make_images(10);
    ASSERT_EQ(images.size(), 10U);

    rm::calib::CameraCalibInput input;
    input.images = std::move(images);
    input.board_size = cv::Size{kBoardW, kBoardH};
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
    ASSERT_EQ(images.size(), 2U);

    rm::calib::CameraCalibInput input;
    input.images = std::move(images);
    input.board_size = cv::Size{kBoardW, kBoardH};
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
    input.board_size = cv::Size{kBoardW, kBoardH};
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
    auto big = make_images(5, cv::Size{800, 600});
    auto small = make_images(5, cv::Size{640, 480});

    std::vector<cv::Mat> mixed;
    mixed.insert(mixed.end(), big.begin(), big.end());
    mixed.insert(mixed.end(), small.begin(), small.end());
    ASSERT_EQ(mixed.size(), 10U);

    rm::calib::CameraCalibInput input;
    input.images = std::move(mixed);
    input.board_size = cv::Size{kBoardW, kBoardH};
    input.square_size_m = kSquareM;

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& r = result.value();

    // All 10 images should be detected despite differing resolutions
    EXPECT_EQ(r.images_used, 10);
}

// ── Charuco helpers ────────────────────────────────────────

constexpr int kCharucoSqX = 5;
constexpr int kCharucoSqY = 7;
constexpr float kSquareLenM = 0.04F;
constexpr float kMarkerLenM = 0.02F;

constexpr int kCharucoDict = cv::aruco::DICT_6X6_250;

cv::aruco::Dictionary charuco_dict() { return cv::aruco::getPredefinedDictionary(kCharucoDict); }

std::vector<cv::Mat> make_charuco_images(int count, cv::Size img_size = {kWidth, kHeight}) {
    return rm::calib::test::generate_charuco_images(
        count, cv::Size{kCharucoSqX, kCharucoSqY}, kSquareLenM, kMarkerLenM, charuco_dict(),
        ground_truth_K(), ground_truth_dist(), img_size);
}

// ─────────────────────────────────────────────────────────
// Test 5 – Charuco happy path
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, CharucoHappyPath) {
    auto images = make_charuco_images(10);
    ASSERT_EQ(images.size(), 10U);

    rm::calib::CameraCalibInput input;
    input.images = std::move(images);
    input.board_type = rm::calib::BoardType::Charuco;
    input.board_size = cv::Size{kCharucoSqX, kCharucoSqY};
    input.square_size_m = kSquareLenM;
    input.marker_size_m = kMarkerLenM;
    input.dictionary_id = kCharucoDict;

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& r = result.value();
    auto gt_K = ground_truth_K();

    EXPECT_LT(r.reproj_error, 5.0);  // warp-based charuco gen limits sub-px accuracy
    EXPECT_GE(r.images_used, 3);

    for (int i = 0; i < 3; ++i) {
        double diff = std::abs(r.K.at<double>(i, i) - gt_K.at<double>(i, i));
        EXPECT_LT(diff, 0.01 * gt_K.at<double>(i, i))
            << "K(" << i << "," << i << ") off by " << diff;
    }
}

// ─────────────────────────────────────────────────────────
// Test 6 – correct board type detection
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, BoardTypeDetection) {
    // Chessboard input → Chessboard mode
    {
        auto images = make_images(10);
        ASSERT_EQ(images.size(), 10U);

        rm::calib::CameraCalibInput input;
        input.images = std::move(images);
        input.board_size = cv::Size{kBoardW, kBoardH};
        input.square_size_m = kSquareM;
        input.board_type = rm::calib::BoardType::Chessboard;

        rm::calib::CameraCalibrator calibrator(input);
        auto result = calibrator.compute();

        ASSERT_TRUE(result.has_value()) << result.error();
        EXPECT_GE(result->images_used, 3);
    }

    // Charuco input → Charuco mode
    {
        auto images = make_charuco_images(10);
        ASSERT_EQ(images.size(), 10U);

        rm::calib::CameraCalibInput input;
        input.images = std::move(images);
        input.board_type = rm::calib::BoardType::Charuco;
        input.board_size = cv::Size{kCharucoSqX, kCharucoSqY};
        input.square_size_m = kSquareLenM;
        input.marker_size_m = kMarkerLenM;
        input.dictionary_id = kCharucoDict;

        rm::calib::CameraCalibrator calibrator(input);
        auto result = calibrator.compute();

        ASSERT_TRUE(result.has_value()) << result.error();
        EXPECT_GE(result->images_used, 3);
    }
}

// ─────────────────────────────────────────────────────────
// Test 7 – partial occlusion still converges
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, PartialOcclusion) {
    auto images = make_charuco_images(10);
    ASSERT_EQ(images.size(), 10U);

    // zero out the top 30 % of each image (set to white)
    for (auto& img : images) {
        int occlude_rows = static_cast<int>(0.30 * img.rows);
        img(cv::Rect{0, 0, img.cols, occlude_rows}).setTo(255);
    }

    rm::calib::CameraCalibInput input;
    input.images = std::move(images);
    input.board_type = rm::calib::BoardType::Charuco;
    input.board_size = cv::Size{kCharucoSqX, kCharucoSqY};
    input.square_size_m = kSquareLenM;
    input.marker_size_m = kMarkerLenM;
    input.dictionary_id = kCharucoDict;

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    ASSERT_TRUE(result.has_value()) << result.error();
}

// ─────────────────────────────────────────────────────────
// Test 8 – wrong dictionary returns Unexpected
// ─────────────────────────────────────────────────────────
TEST(CameraCalibTest, WrongDictionary) {
    auto images = make_charuco_images(10);
    ASSERT_EQ(images.size(), 10U);

    rm::calib::CameraCalibInput input;
    input.images = std::move(images);
    input.board_type = rm::calib::BoardType::Charuco;
    input.board_size = cv::Size{kCharucoSqX, kCharucoSqY};
    input.square_size_m = kSquareLenM;
    input.marker_size_m = kMarkerLenM;
    input.dictionary_id = cv::aruco::DICT_4X4_50;  // wrong dict

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    ASSERT_FALSE(result.has_value());
}

}  // namespace
// NOLINTEND(performance-unnecessary-value-param,performance-unnecessary-copy-initialization)
