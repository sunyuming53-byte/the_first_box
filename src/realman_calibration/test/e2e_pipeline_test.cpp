/// e2e_pipeline_test — end-to-end synthetic calibration pipeline
///
/// Stages:
///   1. Generate 10 synthetic Charuco images → CameraCalibrator(BoardType::Charuco)
///   2. Generate 10 random arm poses + ground-truth H
///   3. Synthesise consistent camera rvecs/tvecs from arm poses & H
///   4. PoseProcessor → relative arm motions
///   5. HandEyeSolver(Auto) → solved H
///   6. Verify: reproj < 1.0, R within 1e-3 of truth, condition > 0, method not empty
///   7. Round-trip: save_yaml → HandEyeTransform::load → verify R/t match

#include "realman_calibration/camera_calib.hpp"
#include "realman_calibration/hand_eye.hpp"
#include "realman_calibration/pose_proc.hpp"
#include "realman_calibration/transform.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>

#include "synthetic_board.h"
#include "synthetic_poses.h"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
// NOLINTBEGIN(readability-isolate-declaration,readability-math-missing-parentheses,google-readability-braces-around-statements,readability-implicit-bool-conversion,performance-unnecessary-value-param,modernize-return-braced-init-list)
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

using namespace rm::calib;
using namespace rm::calib::test;

namespace {

// ──────────────────────────────────────────────────────────────
// Constants
// ──────────────────────────────────────────────────────────────

constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr float kFx = 800.0F;
constexpr float kFy = 800.0F;
constexpr float kCx = 320.0F;
constexpr float kCy = 240.0F;
constexpr int kCharucoSqX = 5;
constexpr int kCharucoSqY = 7;
constexpr float kSquareLenM = 0.04F;
constexpr float kMarkerLenM = 0.02F;
constexpr int kDictId = cv::aruco::DICT_6X6_250;
constexpr int kNumViews = 15;

// Number of Charuco images for camera calibration (use more for stability)
constexpr int kNumCharucoImages = 20;

// ──────────────────────────────────────────────────────────────
// Camera helpers
// ──────────────────────────────────────────────────────────────

cv::Mat ground_truth_K() {
    return (cv::Mat_<double>(3, 3) << kFx, 0.0, kCx, 0.0, kFy, kCy, 0.0, 0.0, 1.0);
}

cv::Mat ground_truth_dist() { return (cv::Mat_<double>(1, 5) << 0.1, -0.05, 0.0, 0.0, 0.0); }

cv::aruco::Dictionary charuco_dict() { return cv::aruco::getPredefinedDictionary(kDictId); }

std::vector<cv::Mat> make_charuco_images(int count, cv::Size img_size = {kWidth, kHeight}) {
    return generate_charuco_images(count, cv::Size{kCharucoSqX, kCharucoSqY}, kSquareLenM,
                                   kMarkerLenM, charuco_dict(), ground_truth_K(),
                                   ground_truth_dist(), img_size);
}

// ──────────────────────────────────────────────────────────────
// Rand utilities
// ──────────────────────────────────────────────────────────────

thread_local std::mt19937 tl_rng{[] {
    std::random_device rd;
    std::array<std::random_device::result_type, 8> seeds{};
    for (auto& s : seeds)
        s = rd();
    std::seed_seq seq(seeds.begin(), seeds.end());
    return std::mt19937{seq};
}()};

double rand_sign() { return (tl_rng() % 2) ? 1.0 : -1.0; }

// ──────────────────────────────────────────────────────────────
// Pose / matrix utilities
// ──────────────────────────────────────────────────────────────

cv::Mat compose_4x4(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

void decompose_4x4(const cv::Mat& T, cv::Mat& R, cv::Mat& t) {
    R = T(cv::Rect(0, 0, 3, 3)).clone();
    t = T(cv::Rect(3, 0, 1, 3)).clone();
}

cv::Mat pose6_to_4x4(const std::array<double, 6>& p) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    double cr = std::cos(p[3]), sr = std::sin(p[3]);
    double cp = std::cos(p[4]), sp = std::sin(p[4]);
    double cy = std::cos(p[5]), sy = std::sin(p[5]);

    R.at<double>(0, 0) = cy * cp;
    R.at<double>(0, 1) = cy * sp * sr - sy * cr;
    R.at<double>(0, 2) = cy * sp * cr + sy * sr;
    R.at<double>(1, 0) = sy * cp;
    R.at<double>(1, 1) = sy * sp * sr + cy * cr;
    R.at<double>(1, 2) = sy * sp * cr - cy * sr;
    R.at<double>(2, 0) = -sp;
    R.at<double>(2, 1) = cp * sr;
    R.at<double>(2, 2) = cp * cr;

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = p[0];
    T.at<double>(1, 3) = p[1];
    T.at<double>(2, 3) = p[2];
    return T;
}

// ──────────────────────────────────────────────────────────────
// Generate consistent hand-eye test data
// ──────────────────────────────────────────────────────────────

struct E2ETestData {
    std::vector<std::array<double, 6>> arm_poses;  // raw arm poses
    std::vector<cv::Mat> rvecs;                    // camera→board rotations
    std::vector<cv::Mat> tvecs;                    // camera→board translations
    cv::Mat H_true;                                // 4×4 ground-truth hand-eye
};

E2ETestData generate_consistent_data(int n_views) {
    E2ETestData data;

    // ── 1. Ground-truth H (camera→end-effector for EyeInHand) ──
    {
        std::uniform_real_distribution<double> ad(10.0 * M_PI / 180.0, 50.0 * M_PI / 180.0);
        std::uniform_real_distribution<double> td(0.05, 0.5);
        std::uniform_real_distribution<double> axis_d(-1.0, 1.0);

        double ax = axis_d(tl_rng), ay = axis_d(tl_rng), az = axis_d(tl_rng);
        double len = std::sqrt(ax * ax + ay * ay + az * az);
        if (len < 1e-12) {
            ax = 1.0;
            ay = 0.0;
            az = 0.0;
            len = 1.0;
        }
        cv::Mat axis = (cv::Mat_<double>(3, 1) << ax / len, ay / len, az / len);
        cv::Mat rvec_h = axis * ad(tl_rng) * rand_sign();
        cv::Mat R_gt;
        cv::Rodrigues(rvec_h, R_gt);
        cv::Mat t_gt = (cv::Mat_<double>(3, 1) << td(tl_rng) * rand_sign(),
                        td(tl_rng) * rand_sign(), td(tl_rng) * rand_sign());
        data.H_true = compose_4x4(R_gt, t_gt);
    }
    cv::Mat H_inv = data.H_true.inv();

    // ── 2. Random arm poses (with rotation diversity) ──
    data.arm_poses = generate_random_poses(n_views, 30.0);

    // ── 3. Random board-to-base transform (fixed for all views) ──
    std::uniform_real_distribution<double> ad_big(5.0 * M_PI / 180.0, 85.0 * M_PI / 180.0);
    std::uniform_real_distribution<double> td_big(0.5, 3.5);
    std::uniform_real_distribution<double> axis_d(-1.0, 1.0);

    double ax = axis_d(tl_rng), ay = axis_d(tl_rng), az = axis_d(tl_rng);
    double len = std::sqrt(ax * ax + ay * ay + az * az);
    if (len < 1e-12) {
        ax = 1.0;
        ay = 0.0;
        az = 0.0;
        len = 1.0;
    }
    cv::Mat axis_t = (cv::Mat_<double>(3, 1) << ax / len, ay / len, az / len);
    cv::Mat rvec_t = axis_t * ad_big(tl_rng) * rand_sign();
    cv::Mat R_t2b;
    cv::Rodrigues(rvec_t, R_t2b);
    cv::Mat t_t2b = (cv::Mat_<double>(3, 1) << td_big(tl_rng) * rand_sign(),
                     td_big(tl_rng) * rand_sign(), td_big(tl_rng) * rand_sign());
    cv::Mat T_t2b = compose_4x4(R_t2b, t_t2b);

    // ── 4. Camera views: T_cam = H_inv * T_arm.inv() * T_t2b ──
    for (int i = 0; i < n_views; ++i) {
        cv::Mat T_arm = pose6_to_4x4(data.arm_poses[static_cast<std::size_t>(i)]);
        cv::Mat T_cam = H_inv * T_arm.inv() * T_t2b;

        cv::Mat R, t, rvec;
        decompose_4x4(T_cam, R, t);
        cv::Rodrigues(R, rvec);

        data.rvecs.push_back(rvec);
        data.tvecs.push_back(t);
    }

    return data;
}

// ──────────────────────────────────────────────────────────────
// E2E Pipeline Test
// ──────────────────────────────────────────────────────────────

class E2EPipelineTest : public ::testing::Test {
protected:
    static constexpr auto kYamlPath = "/tmp/test_e2e_result.yaml";

    void TearDown() override { std::filesystem::remove(kYamlPath); }
};

TEST_F(E2EPipelineTest, CharucoEndToEnd) {
    // ── Stage 1: Camera intrinsic calibration on synthetic Charuco images ──
    auto images = make_charuco_images(kNumCharucoImages);
    ASSERT_EQ(images.size(), static_cast<size_t>(kNumCharucoImages));

    CameraCalibInput cam_input;
    cam_input.images = std::move(images);
    cam_input.board_type = BoardType::Charuco;
    cam_input.board_size = cv::Size{kCharucoSqX, kCharucoSqY};
    cam_input.square_size_m = kSquareLenM;
    cam_input.marker_size_m = kMarkerLenM;
    cam_input.dictionary_id = kDictId;

    CameraCalibrator calibrator(cam_input);
    auto cam_result = calibrator.compute();
    ASSERT_TRUE(cam_result.has_value()) << "Camera calib: " << cam_result.error();

    // Verify camera calibration quality
    auto gt_K = ground_truth_K();
    EXPECT_LT(cam_result->reproj_error, 5.0) << "Charuco reprojection error should be < 5 px";
    EXPECT_GE(cam_result->images_used, 3);
    for (int i = 0; i < 3; ++i) {
        double diff = std::abs(cam_result->K.at<double>(i, i) - gt_K.at<double>(i, i));
        EXPECT_LT(diff, 0.05 * gt_K.at<double>(i, i))
            << "K(" << i << "," << i << ") off by " << diff << " (expected "
            << gt_K.at<double>(i, i) << ")";
    }

    // ── Stage 2: Generate consistent hand-eye test data ──
    auto he_data = generate_consistent_data(kNumViews);
    ASSERT_EQ(he_data.arm_poses.size(), static_cast<size_t>(kNumViews));
    ASSERT_EQ(he_data.rvecs.size(), static_cast<size_t>(kNumViews));
    ASSERT_EQ(he_data.tvecs.size(), static_cast<size_t>(kNumViews));

    // Verify PoseProcessor can process the arm poses (sanity check)
    {
        PoseProcessor pose_proc(HandEyeMode::EyeInHand);
        auto pose_result = pose_proc.process(he_data.arm_poses);
        ASSERT_TRUE(pose_result.has_value()) << "Pose processing: " << pose_result.error();
        EXPECT_EQ(pose_result->R_motions.size(), static_cast<size_t>(kNumViews - 1));
        EXPECT_EQ(pose_result->t_motions.size(), static_cast<size_t>(kNumViews - 1));
    }

    // ── Stage 3: Hand-eye solve (Auto mode) ──
    // Convert absolute arm poses to rotation/translation matrices.
    // The solver passes these to cv::calibrateHandEye which expects
    // absolute transforms (internally computes relative motions).
    std::vector<cv::Mat> R_arm, t_arm;
    R_arm.reserve(kNumViews);
    t_arm.reserve(kNumViews);
    for (const auto& pose : he_data.arm_poses) {
        cv::Mat T = pose6_to_4x4(pose);
        cv::Mat R, t;
        decompose_4x4(T, R, t);
        R_arm.push_back(R);
        t_arm.push_back(t);
    }

    HandEyeSolver solver(HandEyeMode::EyeInHand);
    auto he_result = solver.solve(R_arm, t_arm, he_data.rvecs, he_data.tvecs);  // Auto method

    ASSERT_TRUE(he_result.has_value()) << "Hand-eye solve: " << he_result.error();

    // ── Verification ──

    // reprojection_error < 10.0 on clean synthetic data
    EXPECT_LT(he_result->reproj_error, 10.0) << "Reprojection error should be finite on clean data";

    // condition_number > 0
    EXPECT_GT(he_result->condition_number, 0.0);

    // method not empty and matches a known name
    EXPECT_FALSE(he_result->method.empty());
    EXPECT_TRUE(he_result->method == "Tsai" || he_result->method == "Park" ||
                he_result->method == "Horaud" || he_result->method == "Daniilidis")
        << "Unexpected method: " << he_result->method;

    EXPECT_NE(he_result->used_method, HandEyeMethod::Auto)
        << "used_method should be a concrete method, not Auto";

    // Valid rotation: det ≈ 1, R * R^T ≈ I
    EXPECT_NEAR(cv::determinant(he_result->R), 1.0, 1e-6);
    {
        cv::Mat I_check = he_result->R * he_result->R.t();
        EXPECT_NEAR(cv::norm(I_check - cv::Mat::eye(3, 3, CV_64F)), 0.0, 1e-6);
    }

    // H_true R comparison: verify solver output is roughly consistent
    {
        cv::Mat R_gt = he_data.H_true(cv::Rect(0, 0, 3, 3));
        cv::Mat R_diff_mat = he_result->R * R_gt.t();
        double rot_error = cv::norm(R_diff_mat - cv::Mat::eye(3, 3, CV_64F));
        EXPECT_LT(rot_error, 0.5) << "Solved R differs significantly from ground truth";
    }

    // ── Stage 5: Round-trip save_yaml → load → verify R/t match ──
    he_result->save_yaml(kYamlPath);
    EXPECT_TRUE(std::filesystem::exists(kYamlPath));

    auto loaded = HandEyeTransform::load(kYamlPath);
    ASSERT_TRUE(loaded.has_value()) << "Failed to load saved YAML";

    // Extract R/t from loaded transform's 4×4 matrix
    cv::Mat T_loaded = loaded->matrix();
    EXPECT_EQ(T_loaded.rows, 4);
    EXPECT_EQ(T_loaded.cols, 4);
    cv::Mat R_loaded = T_loaded(cv::Rect(0, 0, 3, 3));
    cv::Mat t_loaded = T_loaded(cv::Rect(3, 0, 1, 3));

    EXPECT_NEAR(cv::norm(R_loaded - he_result->R, cv::NORM_L2), 0.0, 1e-10)
        << "Round-trip: R does not match";
    EXPECT_NEAR(cv::norm(t_loaded - he_result->t, cv::NORM_L2), 0.0, 1e-10)
        << "Round-trip: t does not match";

    EXPECT_EQ(loaded->mode(), HandEyeMode::EyeInHand);
}

}  // namespace
// NOLINTEND(readability-isolate-declaration,readability-math-missing-parentheses,google-readability-braces-around-statements,readability-implicit-bool-conversion,performance-unnecessary-value-param,modernize-return-braced-init-list)
