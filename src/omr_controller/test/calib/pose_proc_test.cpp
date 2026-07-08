#include "omr_controller/calib/pose_proc.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include <array>

#include <opencv2/core.hpp>
// NOLINTBEGIN(google-readability-braces-around-statements)

using namespace omr_controller::calib;

namespace {

// ── helpers ──

/// Verify a 3×3 matrix is a valid rotation (det ≈ 1, R*R^T ≈ I).
void expect_valid_rotation(const cv::Mat& R, double tol = 1e-6) {
    ASSERT_EQ(R.rows, 3);
    ASSERT_EQ(R.cols, 3);
    EXPECT_NEAR(cv::determinant(R), 1.0, tol) << "det(R) should be ≈ 1";
    cv::Mat should_be_I = R * R.t();
    EXPECT_NEAR(cv::norm(should_be_I - cv::Mat::eye(3, 3, CV_64F)), 0.0, tol)
        << "R * R^T should be ≈ I";
}

// Three known arm poses: {tx, ty, tz, rx, ry, rz} in meters and radians (RPY).
constexpr std::array<double, 6> kPose0{0.4, 0.1, 0.3, 0.0, 0.0, 0.0};  // pure translation
constexpr std::array<double, 6> kPose1{0.5, 0.2, 0.3, 0.0, 0.0, 0.5};  // + yaw rotation
constexpr std::array<double, 6> kPose2{0.5, 0.3, 0.4, 0.2, 0.0, 0.0};  // + roll rotation

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────
// Test 1 — EyeInHand, 3 known poses
// ─────────────────────────────────────────────────────────────
TEST(PoseProcessorTest, EyeInHandThreePoses) {
    const std::array<double, 6> poses[] = {kPose0, kPose1, kPose2};

    PoseProcessor proc(HandEyeMode::EyeInHand);
    auto result = proc.process(poses);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->R_motions.size(), 2U);
    EXPECT_EQ(result->t_motions.size(), 2U);

    // Verify each R is a valid rotation matrix.
    for (const auto& R : result->R_motions) {
        expect_valid_rotation(R);
    }

    // Verify t vectors have 3 rows.
    for (const auto& t : result->t_motions) {
        EXPECT_EQ(t.rows, 3);
        EXPECT_EQ(t.cols, 1);
    }
}

// ─────────────────────────────────────────────────────────────
// Test 2 — EyeToHand, same 3 poses
// ─────────────────────────────────────────────────────────────
TEST(PoseProcessorTest, EyeToHandThreePoses) {
    const std::array<double, 6> poses[] = {kPose0, kPose1, kPose2};

    PoseProcessor proc_eih(HandEyeMode::EyeInHand);
    auto result_eih = proc_eih.process(poses);
    ASSERT_TRUE(result_eih.has_value());

    PoseProcessor proc_eth(HandEyeMode::EyeToHand);
    auto result_eth = proc_eth.process(poses);
    ASSERT_TRUE(result_eth.has_value());

    EXPECT_EQ(result_eth->R_motions.size(), 2U);
    EXPECT_EQ(result_eth->t_motions.size(), 2U);

    for (const auto& R : result_eth->R_motions) {
        expect_valid_rotation(R);
    }

    // Verify output differs from EyeInHand.
    bool any_R_differs = false;
    bool any_t_differs = false;
    const double kEps = 1e-10;
    for (size_t i = 0; i < result_eih->R_motions.size(); ++i) {
        if (cv::norm(result_eih->R_motions[i] - result_eth->R_motions[i]) > kEps)
            any_R_differs = true;
        if (cv::norm(result_eih->t_motions[i] - result_eth->t_motions[i]) > kEps)
            any_t_differs = true;
    }
    EXPECT_TRUE(any_R_differs || any_t_differs)
        << "EyeInHand and EyeToHand should produce different A matrices.";
}

// ─────────────────────────────────────────────────────────────
// Test 3 — insufficient poses (only 1)
// ─────────────────────────────────────────────────────────────
TEST(PoseProcessorTest, InsufficientPoses) {
    const std::array<double, 6> poses[] = {kPose0};

    PoseProcessor proc(HandEyeMode::EyeInHand);
    auto result = proc.process(poses);

    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("at least 2"), std::string::npos)
        << "Error message: " << result.error();
}

// ─────────────────────────────────────────────────────────────
// Test 4 — two identical poses → identity motion
// ─────────────────────────────────────────────────────────────
TEST(PoseProcessorTest, IdenticalPoses) {
    const std::array<double, 6> poses[] = {kPose0, kPose0};

    PoseProcessor proc(HandEyeMode::EyeInHand);
    auto result = proc.process(poses);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->R_motions.size(), 1U);
    ASSERT_EQ(result->t_motions.size(), 1U);

    // R should be identity.
    cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
    double R_diff = cv::norm(result->R_motions[0] - I, cv::NORM_L2);
    EXPECT_LT(R_diff, 1e-6) << "R should be identity for identical poses";

    // t should be zero vector.
    double t_norm = cv::norm(result->t_motions[0], cv::NORM_L2);
    EXPECT_LT(t_norm, 1e-6) << "t should be zero for identical poses";
}
// NOLINTEND(google-readability-braces-around-statements)
