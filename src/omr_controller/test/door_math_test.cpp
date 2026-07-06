#include <gtest/gtest.h>

#include <cmath>
#include <opencv2/core.hpp>

#include "omr_controller/door_math.hpp"

namespace omr_controller {
namespace {

// ──── Table 1 verification values (r=2.0, L=1.5, h=0.0) ────
// Each entry: theta_deg, phi_deg, Cx_expected, Cy_expected, Cz_expected

struct Table1Case {
    double theta_deg;
    double phi_deg;
    double cx;
    double cy;
    double cz;
};

// Using exact values from the paper (±0.001 tolerance)
constexpr Table1Case kTable1[] = {
    {0.0, 0.0, 2.0000, 0.0000, 0.0000},
    {0.0, 45.0, 2.0000, 1.0607, 0.4393},
    {0.0, 90.0, 2.0000, 1.5000, 1.5000},
    {0.0, 180.0, 2.0000, 0.0000, 3.0000},
    {30.0, 0.0, 1.7321, 1.0000, 0.0000},
    {30.0, 45.0, 1.2017, 1.9186, 0.4393},
    {30.0, 90.0, 0.9821, 2.2990, 1.5000},
    {30.0, 180.0, 1.7321, 1.0000, 3.0000},
    {60.0, 0.0, 1.0000, 1.7321, 0.0000},
    {60.0, 45.0, 0.0814, 2.2624, 0.4393},
    {60.0, 90.0, -0.2990, 2.4821, 1.5000},
    {60.0, 180.0, 1.0000, 1.7321, 3.0000},
    {90.0, 0.0, 0.0000, 2.0000, 0.0000},
    {90.0, 45.0, -1.0607, 2.0000, 0.4393},
    {90.0, 90.0, -1.5000, 2.0000, 1.5000},
    {90.0, 180.0, -0.0000, 2.0000, 3.0000},
};

constexpr double kR = 2.0;
constexpr double kL = 1.5;
constexpr double kH = 0.0;
constexpr double kTol = 0.001;
constexpr double kPi = M_PI;

double deg2rad(double deg) { return deg * kPi / 180.0; }

class Table1Test : public ::testing::TestWithParam<Table1Case> {};

TEST_P(Table1Test, ComputeCMatches) {
    const auto& tc = GetParam();
    double theta = deg2rad(tc.theta_deg);
    double phi = deg2rad(tc.phi_deg);

    cv::Mat C = computeC(theta, phi, kR, kL, kH);

    EXPECT_NEAR(C.at<double>(0, 0), tc.cx, kTol);
    EXPECT_NEAR(C.at<double>(1, 0), tc.cy, kTol);
    EXPECT_NEAR(C.at<double>(2, 0), tc.cz, kTol);
}

INSTANTIATE_TEST_SUITE_P(All16Cases, Table1Test, ::testing::ValuesIn(kTable1));

// ──── R_T orthonormality ────

TEST(RTTest, ColumnsAreOrthonormal) {
    // Test at a few nontrivial angles
    const double angles[][2] = {
        {0.3, 0.5}, {0.7, 1.2}, {1.0, 2.0}, {2.0, 1.5}, {0.1, 3.0}};

    for (const auto& [theta, phi] : angles) {
        cv::Mat R = computeRT(theta, phi);

        // Extract columns
        cv::Mat c0 = R.col(0);
        cv::Mat c1 = R.col(1);
        cv::Mat c2 = R.col(2);

        // Dot products: c_i · c_j = 0 for i≠j, = 1 for i=j
        EXPECT_NEAR(c0.dot(c1), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(c0.dot(c2), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(c1.dot(c2), 0.0, kTol) << "θ=" << theta << " φ=" << phi;

        EXPECT_NEAR(c0.dot(c0), 1.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(c1.dot(c1), 1.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(c2.dot(c2), 1.0, kTol) << "θ=" << theta << " φ=" << phi;
    }
}

TEST(RTTest, DeterminantIsOne) {
    // Test at a few nontrivial angles
    const double angles[][2] = {
        {0.3, 0.5}, {0.7, 1.2}, {1.0, 2.0}, {2.0, 1.5}, {0.1, 3.0}};

    for (const auto& [theta, phi] : angles) {
        cv::Mat R = computeRT(theta, phi);
        double det = cv::determinant(R);
        EXPECT_NEAR(det, 1.0, kTol) << "θ=" << theta << " φ=" << phi;
    }
}

TEST(RTTest, PhiZero_Columns_ez_u_ezCrossU) {
    // At φ=0, R_T = [e_z, u, e_z×u]
    const double theta_vals[] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, kPi};

    for (double theta : theta_vals) {
        cv::Mat R = computeRT(theta, 0.0);

        // Column 0 should be e_z = (0, 0, 1)
        EXPECT_NEAR(R.at<double>(0, 0), 0.0, kTol);
        EXPECT_NEAR(R.at<double>(1, 0), 0.0, kTol);
        EXPECT_NEAR(R.at<double>(2, 0), 1.0, kTol);

        // Column 1 should be u = (cosθ, sinθ, 0)
        double ct = std::cos(theta);
        double st = std::sin(theta);
        EXPECT_NEAR(R.at<double>(0, 1), ct, kTol);
        EXPECT_NEAR(R.at<double>(1, 1), st, kTol);
        EXPECT_NEAR(R.at<double>(2, 1), 0.0, kTol);

        // Column 2 should be e_z × u = (-sinθ, cosθ, 0)
        EXPECT_NEAR(R.at<double>(0, 2), -st, kTol);
        EXPECT_NEAR(R.at<double>(1, 2), ct, kTol);
        EXPECT_NEAR(R.at<double>(2, 2), 0.0, kTol);
    }
}

// ──── C special cases ────

TEST(CTest, PhiZero_Cz_is_h) {
    // φ=0 → C_z = h (any θ)
    const double theta_vals[] = {0.0, 0.5, 1.0, 2.0, kPi};
    for (double theta : theta_vals) {
        cv::Mat C = computeC(theta, 0.0, kR, kL, kH);
        EXPECT_NEAR(C.at<double>(2, 0), kH, kTol) << "θ=" << theta;
    }
}

TEST(CTest, PhiPi_Cz_is_h_plus_2L) {
    // φ=π → C_z = h + 2L
    const double theta_vals[] = {0.0, 0.5, 1.0, 2.0, kPi};
    for (double theta : theta_vals) {
        cv::Mat C = computeC(theta, kPi, kR, kL, kH);
        EXPECT_NEAR(C.at<double>(2, 0), kH + 2 * kL, kTol) << "θ=" << theta;
    }
}

TEST(CTest, ThetaZero_Cx_is_r) {
    // θ=0 → C_x = r (any φ)
    const double phi_vals[] = {0.0, 0.5, 1.0, 2.0, kPi};
    for (double phi : phi_vals) {
        cv::Mat C = computeC(0.0, phi, kR, kL, kH);
        EXPECT_NEAR(C.at<double>(0, 0), kR, kTol) << "φ=" << phi;
    }
}

// ──── world_T_target verification ────

TEST(WorldTTargetTest, OriginMapsToOrigin) {
    // world_T_target · (C, 1)^T = (0, 0, 0, 1)^T
    const double cases[][2] = {
        {0.0, 0.0}, {0.5, 0.3}, {1.0, 1.5}, {2.0, 3.0}, {0.0, kPi}};

    for (const auto& [theta, phi] : cases) {
        cv::Mat C = computeC(theta, phi, kR, kL, kH);
        cv::Mat T = computeWorldTTarget(theta, phi, kR, kL, kH);

        // Build homogeneous C
        cv::Mat C_hom = cv::Mat::ones(4, 1, CV_64F);
        C_hom.at<double>(0, 0) = C.at<double>(0, 0);
        C_hom.at<double>(1, 0) = C.at<double>(1, 0);
        C_hom.at<double>(2, 0) = C.at<double>(2, 0);

        cv::Mat result = T * C_hom;

        EXPECT_NEAR(result.at<double>(0, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(1, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(2, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(3, 0), 1.0, kTol) << "θ=" << theta << " φ=" << phi;
    }
}

// Additional verification: bottom endpoint B = C - L·x̂_T maps to (-L, 0, 0, 1)
TEST(WorldTTargetTest, BottomEndpointMapsToMinusL) {
    // B = C - L·x̂_T (world coords), should map to (-L, 0, 0) in target frame
    const double cases[][2] = {{0.0, 0.3}, {0.5, 0.7}, {1.0, 1.2}};

    for (const auto& [theta, phi] : cases) {
        cv::Mat C = computeC(theta, phi, kR, kL, kH);
        cv::Mat R = computeRT(theta, phi);
        cv::Mat T = computeWorldTTarget(theta, phi, kR, kL, kH);

        // x̂_T is R's first column
        cv::Mat xT = R.col(0);

        // B = C - L * x̂_T (world coords)
        cv::Mat B = C - kL * xT;

        cv::Mat B_hom = cv::Mat::ones(4, 1, CV_64F);
        B_hom.at<double>(0, 0) = B.at<double>(0, 0);
        B_hom.at<double>(1, 0) = B.at<double>(1, 0);
        B_hom.at<double>(2, 0) = B.at<double>(2, 0);

        cv::Mat result = T * B_hom;

        EXPECT_NEAR(result.at<double>(0, 0), -kL, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(1, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(2, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(3, 0), 1.0, kTol) << "θ=" << theta << " φ=" << phi;
    }
}

// Top endpoint T = C + L·x̂_T maps to (L, 0, 0, 1)
TEST(WorldTTargetTest, TopEndpointMapsToL) {
    const double cases[][2] = {{0.0, 0.3}, {0.5, 0.7}, {1.0, 1.2}};

    for (const auto& [theta, phi] : cases) {
        cv::Mat C = computeC(theta, phi, kR, kL, kH);
        cv::Mat R = computeRT(theta, phi);
        cv::Mat T = computeWorldTTarget(theta, phi, kR, kL, kH);

        cv::Mat xT = R.col(0);

        // T_top = C + L * x̂_T (world coords)
        cv::Mat T_top = C + kL * xT;

        cv::Mat T_hom = cv::Mat::ones(4, 1, CV_64F);
        T_hom.at<double>(0, 0) = T_top.at<double>(0, 0);
        T_hom.at<double>(1, 0) = T_top.at<double>(1, 0);
        T_hom.at<double>(2, 0) = T_top.at<double>(2, 0);

        cv::Mat result = T * T_hom;

        EXPECT_NEAR(result.at<double>(0, 0), kL, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(1, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(2, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(3, 0), 1.0, kTol) << "θ=" << theta << " φ=" << phi;
    }
}

// Point C + ŷ_T maps to (0, 1, 0, 1)
TEST(WorldTTargetTest, AlongYAxisMapsToYUnit) {
    const double cases[][2] = {{0.0, 0.3}, {0.5, 0.7}, {1.0, 1.2}};

    for (const auto& [theta, phi] : cases) {
        cv::Mat C = computeC(theta, phi, kR, kL, kH);
        cv::Mat R = computeRT(theta, phi);
        cv::Mat T = computeWorldTTarget(theta, phi, kR, kL, kH);

        cv::Mat yT = R.col(1);

        // P = C + ŷ_T (world coords)
        cv::Mat P = C + yT;

        cv::Mat P_hom = cv::Mat::ones(4, 1, CV_64F);
        P_hom.at<double>(0, 0) = P.at<double>(0, 0);
        P_hom.at<double>(1, 0) = P.at<double>(1, 0);
        P_hom.at<double>(2, 0) = P.at<double>(2, 0);

        cv::Mat result = T * P_hom;

        EXPECT_NEAR(result.at<double>(0, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(1, 0), 1.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(2, 0), 0.0, kTol) << "θ=" << theta << " φ=" << phi;
        EXPECT_NEAR(result.at<double>(3, 0), 1.0, kTol) << "θ=" << theta << " φ=" << phi;
    }
}

}  // namespace
}  // namespace omr_controller
