#pragma once

#include <cmath>

#include <opencv2/core.hpp>

namespace omr_controller {

/// Eq.12 — Center of the rotated line segment (3×1 CV_64F column vector).
///
/// @param theta  Angle of the rotating "door" rectangle about the z-axis [rad].
/// @param phi    Angle of the line segment about the top-end axis u [rad].
/// @param omega  Rotation angle of the lower gimbal frame about x̂_T [rad].
/// @param r      Radial distance of the bottom endpoint from the z-axis.
/// @param L      Half-length of the line segment.
/// @param h      Height of the bottom endpoint above the xy-plane.
///
/// C(θ,φ,ω) = ( r·cosθ - L·sinθ·sinφ - L·cosθ·cosφ·sinω,
///              r·sinθ + L·cosθ·sinφ - L·sinθ·cosφ·sinω,
///              h + L·(1 - cosφ·cosω) )^T
inline cv::Mat computeC(double theta, double phi, double omega,
                         double r, double L, double h) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double cp = std::cos(phi);
    double sp = std::sin(phi);
    double cw = std::cos(omega);
    double sw = std::sin(omega);

    auto C = cv::Mat_<double>(3, 1);
    C(0, 0) = r * ct - L * st * sp - L * ct * cp * sw;
    C(1, 0) = r * st + L * ct * sp - L * st * cp * sw;
    C(2, 0) = h + L * (1.0 - cp * cw);
    return C;
}

/// Eq.18-20 — Rotation matrix R_T that maps target-frame coordinates to world-frame
/// coordinates (3×3 CV_64F matrix, columns are x̂_T, ŷ_T, ẑ_T in world coords).
///
/// x̂_T = ( sinθ·sinφ + cosθ·cosφ·sinω,  -cosθ·sinφ + sinθ·cosφ·sinω,  cosφ·cosω )ᵀ
/// ŷ_T = ( cosθ·cosω,                    sinθ·cosω,                   -sinω       )ᵀ
/// ẑ_T = ( -cosθ·sinφ·sinω - sinθ·cosφ, cosθ·cosφ - sinθ·sinφ·sinω,  sinφ·cosω   )ᵀ
inline cv::Mat computeRT(double theta, double phi, double omega) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double cp = std::cos(phi);
    double sp = std::sin(phi);
    double cw = std::cos(omega);
    double sw = std::sin(omega);

    auto R = cv::Mat_<double>(3, 3);
    // Column 0: x̂_T
    R(0, 0) = st * sp + ct * cp * sw;
    R(1, 0) = -ct * sp + st * cp * sw;
    R(2, 0) = cp * cw;
    // Column 1: ŷ_T
    R(0, 1) = ct * cw;
    R(1, 1) = st * cw;
    R(2, 1) = -sw;
    // Column 2: ẑ_T (CORRECTED — tex Eq.20 sign errors fixed)
    R(0, 2) = ct * sp * sw - st * cp;    // = +cosθ·sinφ·sinω - sinθ·cosφ
    R(1, 2) = ct * cp + st * sp * sw;    // = cosθ·cosφ + sinθ·sinφ·sinω
    R(2, 2) = sp * cw;                   // = sinφ·cosω (unchanged)
    return R;
}

/// Eq.25 — Homogeneous transformation matrix that maps world-frame coordinates to
/// target-frame coordinates (4×4 CV_64F matrix).
///
/// world_T_target = [ R_T^T   -R_T^T·C ]
///                  [ 0 0 0   1        ]
///
/// Top-left 3×3 = R_T^T (transpose of computeRT).  Translation from Eq.23 closed-form:
///   tx = (h+L)·cosφ·cosω - L
///   ty = -r
///   tz = -h·sinφ·cosω + L·cosφ·sinω
///
/// Verifies: world_T_target · (C, 1)^T = (0, 0, 0, 1)^T
inline cv::Mat computeWorldTTarget(double theta, double phi, double omega,
                                    double r, double L, double h) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double cp = std::cos(phi);
    double sp = std::sin(phi);
    double cw = std::cos(omega);
    double sw = std::sin(omega);

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    // Top-left 3×3 block = R_T^T (rows of computeRT)
    T.at<double>(0, 0) = st * sp + ct * cp * sw;   // x̂_T_x
    T.at<double>(0, 1) = -ct * sp + st * cp * sw;  // x̂_T_y
    T.at<double>(0, 2) = cp * cw;                   // x̂_T_z
    T.at<double>(1, 0) = ct * cw;                   // ŷ_T_x
    T.at<double>(1, 1) = st * cw;                   // ŷ_T_y
    T.at<double>(1, 2) = -sw;                       // ŷ_T_z
    T.at<double>(2, 0) = ct * sp * sw - st * cp;    // ẑ_T_x (corrected)
    T.at<double>(2, 1) = ct * cp + st * sp * sw;    // ẑ_T_y (corrected)
    T.at<double>(2, 2) = sp * cw;                   // ẑ_T_z

    // Translation column t = -R_T^T · C (must match corrected R_T)
    cv::Mat C = (cv::Mat_<double>(3, 1) << r * ct - L * st * sp - L * ct * cp * sw,
                                             r * st + L * ct * sp - L * st * cp * sw,
                                             h + L * (1.0 - cp * cw));
    // R_T^T is already stored in T's 3×3 block
    cv::Mat RtC = T(cv::Rect(0, 0, 3, 3)) * C;
    T.at<double>(0, 3) = -RtC.at<double>(0, 0);
    T.at<double>(1, 3) = -RtC.at<double>(1, 0);
    T.at<double>(2, 3) = -RtC.at<double>(2, 0);

    return T;
}

}  // namespace omr_controller
