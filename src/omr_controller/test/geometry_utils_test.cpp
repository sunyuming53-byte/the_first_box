#include <gtest/gtest.h>

#include <cmath>
#include <opencv2/core.hpp>

#include "omr_controller/geometry_utils.hpp"

namespace omr_controller {
namespace {

constexpr double kTol = 1e-9;

// ──── Identity 4×4 matrix → identity pose ────

TEST(HomogeneousToPoseTest, IdentityMatrix) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    auto pose = homogeneous_to_pose(T);

    // Position at origin
    EXPECT_NEAR(pose.position.x, 0.0, kTol);
    EXPECT_NEAR(pose.position.y, 0.0, kTol);
    EXPECT_NEAR(pose.position.z, 0.0, kTol);

    // Identity quaternion: (0, 0, 0, 1)
    EXPECT_NEAR(pose.orientation.x, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.y, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.z, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.w, 1.0, kTol);
}

// ──── Pure Z-rotation 90° → quaternion (0, 0, 0.707, 0.707) ────

TEST(HomogeneousToPoseTest, ZRotation90Deg) {
    double half_pi = M_PI / 2.0;
    double c = std::cos(half_pi);  // 0
    double s = std::sin(half_pi);  // 1

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    T.at<double>(0, 0) = c;
    T.at<double>(0, 1) = -s;
    T.at<double>(1, 0) = s;
    T.at<double>(1, 1) = c;

    // Also set a non-zero translation to verify it's preserved
    T.at<double>(0, 3) = 1.0;
    T.at<double>(1, 3) = 2.0;
    T.at<double>(2, 3) = 3.0;

    auto pose = homogeneous_to_pose(T);

    // Translation preserved
    EXPECT_NEAR(pose.position.x, 1.0, kTol);
    EXPECT_NEAR(pose.position.y, 2.0, kTol);
    EXPECT_NEAR(pose.position.z, 3.0, kTol);

    // Quaternion for 90° Z-rotation: (0, 0, sin(45°), cos(45°))
    double expected_z = std::sin(half_pi / 2.0);
    double expected_w = std::cos(half_pi / 2.0);

    EXPECT_NEAR(pose.orientation.x, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.y, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.z, expected_z, kTol);
    EXPECT_NEAR(pose.orientation.w, expected_w, kTol);
}

// ──── Translation only (identity rotation) ────

TEST(HomogeneousToPoseTest, TranslationOnly) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    T.at<double>(0, 3) = 1.5;
    T.at<double>(1, 3) = -2.5;
    T.at<double>(2, 3) = 0.75;

    auto pose = homogeneous_to_pose(T);

    EXPECT_NEAR(pose.position.x, 1.5, kTol);
    EXPECT_NEAR(pose.position.y, -2.5, kTol);
    EXPECT_NEAR(pose.position.z, 0.75, kTol);

    // Rotation is identity → identity quaternion
    EXPECT_NEAR(pose.orientation.x, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.y, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.z, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.w, 1.0, kTol);
}

// ──── Rotation about X-axis 90° ────

TEST(HomogeneousToPoseTest, XRotation90Deg) {
    double half_pi = M_PI / 2.0;
    double c = std::cos(half_pi);  // 0
    double s = std::sin(half_pi);  // 1

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    T.at<double>(1, 1) = c;
    T.at<double>(1, 2) = -s;
    T.at<double>(2, 1) = s;
    T.at<double>(2, 2) = c;

    T.at<double>(0, 3) = 5.0;
    T.at<double>(1, 3) = -3.0;
    T.at<double>(2, 3) = 7.0;

    auto pose = homogeneous_to_pose(T);

    // Translation
    EXPECT_NEAR(pose.position.x, 5.0, kTol);
    EXPECT_NEAR(pose.position.y, -3.0, kTol);
    EXPECT_NEAR(pose.position.z, 7.0, kTol);

    // Quaternion for 90° X-rotation: (sin(45°), 0, 0, cos(45°))
    double expected_x = std::sin(half_pi / 2.0);
    double expected_w = std::cos(half_pi / 2.0);

    EXPECT_NEAR(pose.orientation.x, expected_x, kTol);
    EXPECT_NEAR(pose.orientation.y, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.z, 0.0, kTol);
    EXPECT_NEAR(pose.orientation.w, expected_w, kTol);
}

}  // namespace
}  // namespace omr_controller
