#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "realman_calibration/hand_eye.hpp"
#include "realman_calibration/pose_proc.hpp"
#include "synthetic_poses.h"

using namespace rm::calib;
using namespace rm::calib::test;

namespace {

// ──────────────────────────────────────────────────────────────
// constants
// ──────────────────────────────────────────────────────────────

constexpr int kNumMotions = 6;
constexpr int kBoardW      = 8;
constexpr int kBoardH      = 6;
constexpr double kSquareSize = 0.03;

constexpr double kMinAngleRad = 5.0 * M_PI / 180.0;
constexpr double kMaxAngleRad = 45.0 * M_PI / 180.0;
constexpr double kMinT       = 0.5;
constexpr double kMaxT       = 1.5;

// ──────────────────────────────────────────────────────────────
// bounded-pose generation
// ──────────────────────────────────────────────────────────────

thread_local std::mt19937 tl_rng{[] {
    std::random_device rd;
    std::array<std::random_device::result_type, 8> seeds{};
    for (auto& s : seeds) s = rd();
    std::seed_seq seq(seeds.begin(), seeds.end());
    return std::mt19937{seq};
}()};

double rand_sign() { return (tl_rng() % 2) ? 1.0 : -1.0; }

[[nodiscard]] auto bounded_random_poses(int num)
    -> std::vector<std::array<double, 6>>
{
    std::uniform_real_distribution<double> ad(kMinAngleRad, kMaxAngleRad);
    std::uniform_real_distribution<double> td(kMinT, kMaxT);
    std::uniform_real_distribution<double> axis_dist(-1.0, 1.0);

    std::vector<std::array<double, 6>> poses;
    poses.reserve(static_cast<std::size_t>(num));

    for (int i = 0; i < num; ++i) {
        double ax = axis_dist(tl_rng), ay = axis_dist(tl_rng), az = axis_dist(tl_rng);
        double len = std::sqrt(ax * ax + ay * ay + az * az);
        if (len < 1e-12) { ax = 1.0; ay = 0.0; az = 0.0; len = 1.0; }

        double angle = ad(tl_rng) * rand_sign();
        cv::Mat axis = (cv::Mat_<double>(3, 1) << ax / len, ay / len, az / len);
        cv::Mat rvec = axis * angle;
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        double sy = R.at<double>(2, 0);
        double pitch = std::asin(sy);
        double roll, yaw;
        if (std::abs(sy) < 0.99999) {
            roll  = std::atan2(-R.at<double>(2, 1), R.at<double>(2, 2));
            yaw   = std::atan2(-R.at<double>(1, 0), R.at<double>(0, 0));
        } else {
            roll  = std::atan2( R.at<double>(1, 2), R.at<double>(1, 1));
            yaw   = 0.0;
        }

        poses.push_back({
            td(tl_rng) * rand_sign(),
            td(tl_rng) * rand_sign(),
            td(tl_rng) * rand_sign(),
            roll, pitch, yaw
        });
    }
    return poses;
}

[[nodiscard]] auto pose6_to_4x4(const std::array<double, 6>& p) -> cv::Mat {
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

[[nodiscard]] auto compose_4x4(const cv::Mat& R, const cv::Mat& t) -> cv::Mat {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t.copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

void decompose_4x4(const cv::Mat& T, cv::Mat& R, cv::Mat& t) {
    R = T(cv::Rect(0, 0, 3, 3)).clone();
    t = T(cv::Rect(3, 0, 1, 3)).clone();
}

// ──────────────────────────────────────────────────────────────
// chessboard & camera
// ──────────────────────────────────────────────────────────────

[[nodiscard]] auto board_points() -> std::vector<cv::Point3f> {
    std::vector<cv::Point3f> pts;
    pts.reserve(static_cast<std::size_t>(kBoardH * kBoardW));
    for (int r = 0; r < kBoardH; ++r)
        for (int c = 0; c < kBoardW; ++c)
            pts.emplace_back(static_cast<float>(c) * kSquareSize,
                             static_cast<float>(r) * kSquareSize, 0.0f);
    return pts;
}

[[nodiscard]] auto camera_matrix() -> cv::Mat {
    return (cv::Mat_<double>(3, 3) << 800.0, 0.0, 320.0,
                                      0.0, 800.0, 240.0,
                                      0.0, 0.0,   1.0);
}

[[nodiscard]] auto zero_distortion() -> cv::Mat {
    return cv::Mat::zeros(5, 1, CV_64F);
}

// ──────────────────────────────────────────────────────────────
// data generation
// ──────────────────────────────────────────────────────────────

struct HandEyeTestData {
    std::vector<cv::Mat> R_tool;
    std::vector<cv::Mat> t_tool;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::Mat H_true;
};

[[nodiscard]] auto generate_test_data(HandEyeMode mode [[maybe_unused]],
                                       int n_views,
                                       double noise_sigma)
    -> HandEyeTestData
{
    HandEyeTestData data;

    // ── 1. Random H ──
    {
        std::uniform_real_distribution<double> ad(
            10.0 * M_PI / 180.0, 50.0 * M_PI / 180.0);
        std::uniform_real_distribution<double> td(0.05, 0.5);
        std::uniform_real_distribution<double> axis_d(-1.0, 1.0);

        double ax = axis_d(tl_rng), ay = axis_d(tl_rng), az = axis_d(tl_rng);
        double len = std::sqrt(ax * ax + ay * ay + az * az);
        if (len < 1e-12) { ax = 1.0; ay = 0.0; az = 0.0; len = 1.0; }
        cv::Mat axis = (cv::Mat_<double>(3, 1) << ax / len, ay / len, az / len);
        cv::Mat rvec_h = axis * ad(tl_rng) * rand_sign();
        cv::Mat R_gt;
        cv::Rodrigues(rvec_h, R_gt);
        cv::Mat t_gt = (cv::Mat_<double>(3, 1) <<
                        td(tl_rng) * rand_sign(),
                        td(tl_rng) * rand_sign(),
                        td(tl_rng) * rand_sign());
        data.H_true = compose_4x4(R_gt, t_gt);
    }
    cv::Mat H_inv = data.H_true.inv();

    // ── 2. Absolute arm poses ──
    auto arm_poses6 = bounded_random_poses(n_views);
    for (auto i = 0; i < n_views; ++i) {
        cv::Mat T_arm = pose6_to_4x4(
            arm_poses6[static_cast<std::size_t>(i)]);
        cv::Mat R, t;
        decompose_4x4(T_arm, R, t);
        data.R_tool.push_back(R);
        data.t_tool.push_back(t);
    }

    // ── 3. T_target2base ──
    std::uniform_real_distribution<double> ad_big(
        5.0 * M_PI / 180.0, 85.0 * M_PI / 180.0);
    std::uniform_real_distribution<double> td_big(0.5, 3.5);
    std::uniform_real_distribution<double> axis_d(-1.0, 1.0);

    double ax = axis_d(tl_rng), ay = axis_d(tl_rng), az = axis_d(tl_rng);
    double len = std::sqrt(ax * ax + ay * ay + az * az);
    if (len < 1e-12) { ax = 1.0; ay = 0.0; az = 0.0; len = 1.0; }
    cv::Mat axis_t = (cv::Mat_<double>(3, 1) << ax / len, ay / len, az / len);
    cv::Mat rvec_t = axis_t * ad_big(tl_rng) * rand_sign();
    cv::Mat R_t2b;
    cv::Rodrigues(rvec_t, R_t2b);
    cv::Mat t_t2b = (cv::Mat_<double>(3, 1) <<
                     td_big(tl_rng) * rand_sign(),
                     td_big(tl_rng) * rand_sign(),
                     td_big(tl_rng) * rand_sign());
    cv::Mat T_t2b = compose_4x4(R_t2b, t_t2b);

    // ── 4. Camera views ──
    const auto board_pts = board_points();
    const auto K         = camera_matrix();
    const auto dist      = zero_distortion();

    for (int i = 0; i < n_views; ++i) {
        cv::Mat T_arm = pose6_to_4x4(
            arm_poses6[static_cast<std::size_t>(i)]);
        cv::Mat T_cam = H_inv * T_arm.inv() * T_t2b;

        cv::Mat R, t, rvec;
        decompose_4x4(T_cam, R, t);
        cv::Rodrigues(R, rvec);

        if (noise_sigma > 0.0) {
            std::vector<cv::Point2f> img_pts;
            cv::projectPoints(board_pts, rvec, t, K, dist, img_pts);
            inject_noise(img_pts, noise_sigma);
            cv::solvePnP(board_pts, img_pts, K, dist, rvec, t);
        }

        data.rvecs.push_back(rvec);
        data.tvecs.push_back(t);
    }

    return data;
}

}  // anonymous namespace

// ──────────────────────────────────────────────────────────────
// Test 1 — EyeInHand: solver returns valid result on clean data
// ──────────────────────────────────────────────────────────────
TEST(HandEyeSolverTest, EyeInHandHappy) {
    auto data = generate_test_data(HandEyeMode::EyeInHand, kNumMotions, 0.0);

    // Verify that calibrateHandEye itself recovers ground truth
    // (sanity check that the generated data is self-consistent)
    {
        std::vector<cv::Mat> R_cam_R;
        for (const auto& rv : data.rvecs) {
            cv::Mat R; cv::Rodrigues(rv, R); R_cam_R.push_back(R);
        }
        cv::Mat Rd, td;
        cv::calibrateHandEye(data.R_tool, data.t_tool,
                             R_cam_R, data.tvecs,
                             Rd, td, cv::CALIB_HAND_EYE_TSAI);
        cv::Mat R_gt = data.H_true(cv::Rect(0, 0, 3, 3));
        cv::Mat t_gt = data.H_true(cv::Rect(3, 0, 1, 3));
        EXPECT_LT(cv::norm(Rd - R_gt, cv::NORM_L2), 1e-6)
            << "Reference calibrateHandEye should recover ground truth";
        EXPECT_LT(cv::norm(td - t_gt, cv::NORM_L2), 1e-6);
    }

    HandEyeSolver solver(HandEyeMode::EyeInHand);
    auto result = solver.solve(data.R_tool, data.t_tool,
                               data.rvecs, data.tvecs);

    ASSERT_TRUE(result.has_value())
        << "Solve failed: " << result.error();

    // Structural checks
    EXPECT_EQ(result->R.rows, 3);
    EXPECT_EQ(result->R.cols, 3);
    EXPECT_EQ(result->t.rows, 3);
    EXPECT_EQ(result->t.cols, 1);
    EXPECT_TRUE(result->method == "Tsai" || result->method == "Park" ||
                result->method == "Horaud" || result->method == "Daniilidis");
    EXPECT_NE(result->used_method, HandEyeMethod::Auto)
        << "used_method should be a concrete method, not Auto";
    EXPECT_GT(result->condition_number, 0.0)
        << "condition_number should be > 0 for well-conditioned data";
    EXPECT_EQ(result->mode, HandEyeMode::EyeInHand);

    // R should be a valid rotation matrix
    EXPECT_NEAR(cv::determinant(result->R), 1.0, 1e-6);
    cv::Mat should_be_I = result->R * result->R.t();
    EXPECT_NEAR(cv::norm(should_be_I - cv::Mat::eye(3, 3, CV_64F)),
                0.0, 1e-6);

    // Reprojection error should be finite on clean data
    EXPECT_GE(result->reproj_error, 0.0);
    EXPECT_LT(result->reproj_error, 1e3);
}

// ──────────────────────────────────────────────────────────────
// Test 2 — EyeToHand: solver returns valid result on clean data
// ──────────────────────────────────────────────────────────────
TEST(HandEyeSolverTest, EyeToHandHappy) {
    auto data = generate_test_data(HandEyeMode::EyeToHand, kNumMotions, 0.0);

    HandEyeSolver solver(HandEyeMode::EyeToHand);
    auto result = solver.solve(data.R_tool, data.t_tool,
                               data.rvecs, data.tvecs);

    ASSERT_TRUE(result.has_value())
        << "Solve failed: " << result.error();

    EXPECT_EQ(result->R.rows, 3);
    EXPECT_EQ(result->R.cols, 3);
    EXPECT_EQ(result->t.rows, 3);
    EXPECT_EQ(result->t.cols, 1);
    EXPECT_TRUE(result->method == "Tsai" || result->method == "Park" ||
                result->method == "Horaud" || result->method == "Daniilidis");
    EXPECT_NE(result->used_method, HandEyeMethod::Auto)
        << "used_method should be a concrete method, not Auto";
    EXPECT_GT(result->condition_number, 0.0);
    EXPECT_EQ(result->mode, HandEyeMode::EyeToHand);

    EXPECT_NEAR(cv::determinant(result->R), 1.0, 1e-6);
    cv::Mat should_be_I = result->R * result->R.t();
    EXPECT_NEAR(cv::norm(should_be_I - cv::Mat::eye(3, 3, CV_64F)),
                0.0, 1e-6);

    EXPECT_GE(result->reproj_error, 0.0);
    EXPECT_LT(result->reproj_error, 1e3);
}

// ──────────────────────────────────────────────────────────────
// Test 3 — fewer than 3 relative motions → error
// ──────────────────────────────────────────────────────────────
TEST(HandEyeSolverTest, InsufficientMotions) {
    auto arm_poses = generate_random_poses(2);

    PoseProcessor proc(HandEyeMode::EyeInHand);
    auto proc_result = proc.process(arm_poses);
    ASSERT_TRUE(proc_result.has_value());

    std::vector<cv::Mat> rvecs = {
        cv::Mat::zeros(3, 1, CV_64F)
    };
    std::vector<cv::Mat> tvecs = {
        cv::Mat::zeros(3, 1, CV_64F)
    };

    HandEyeSolver solver(HandEyeMode::EyeInHand);
    auto result = solver.solve(proc_result->R_motions, proc_result->t_motions,
                               rvecs, tvecs);

    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("at least 3"), std::string::npos)
        << "Error message: " << result.error();
}

// ──────────────────────────────────────────────────────────────
// Test 4 — noise tolerance: 0.5 px Gaussian noise, still solves
// ──────────────────────────────────────────────────────────────
TEST(HandEyeSolverTest, NoiseTolerance) {
    constexpr double kNoiseSigma = 0.5;

    auto data = generate_test_data(HandEyeMode::EyeInHand,
                                    kNumMotions, kNoiseSigma);

    HandEyeSolver solver(HandEyeMode::EyeInHand);
    auto result = solver.solve(data.R_tool, data.t_tool,
                               data.rvecs, data.tvecs);

    ASSERT_TRUE(result.has_value())
        << "Solve should succeed even with 0.5 px noise, but got error: "
        << result.error();

    EXPECT_GT(result->reproj_error, 0.0)
        << "Noisy data should produce a non-zero reprojection error";
    EXPECT_LT(result->reproj_error, 1e3)
        << "Reprojection error should be finite";

    EXPECT_NEAR(cv::determinant(result->R), 1.0, 1e-6)
        << "Noisy solve should still produce a valid rotation";
}
