#pragma once

#include <cmath>

#include <opencv2/core.hpp>

namespace omr_controller {

/// Eq.6 — Center of the rotated line segment (3×1 CV_64F column vector).
///
/// @param theta  Angle of the rotating "door" rectangle about the z-axis [rad].
/// @param phi    Angle of the line segment about the top-end axis u [rad].
/// @param r      Radial distance of the bottom endpoint from the z-axis.
/// @param L      Half-length of the line segment.
/// @param h      Height of the bottom endpoint above the xy-plane.
///
/// C(θ,φ) = ( r·cosθ - L·sinθ·sinφ,
///            r·sinθ + L·cosθ·sinφ,
///            h + L·(1 - cosφ) )^T
inline cv::Mat computeC(double theta, double phi, double r, double L, double h) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double cp = std::cos(phi);
    double sp = std::sin(phi);

    auto C = cv::Mat_<double>(3, 1);
    C(0, 0) = r * ct - L * st * sp;
    C(1, 0) = r * st + L * ct * sp;
    C(2, 0) = h + L * (1.0 - cp);
    return C;
}

/// Eq.32 — Rotation matrix R_T that maps target-frame coordinates to world-frame
/// coordinates (3×3 CV_64F matrix, columns are x̂_T, ŷ_T, ẑ_T in world coords).
///
/// R_T = [ sinθ·sinφ     cosθ     -cosφ·sinθ ]
///       [ -cosθ·sinφ    sinθ     cosφ·cosθ  ]
///       [ cosφ           0        sinφ       ]
inline cv::Mat computeRT(double theta, double phi) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double cp = std::cos(phi);
    double sp = std::sin(phi);

    auto R = cv::Mat_<double>(3, 3);
    R(0, 0) = st * sp;
    R(0, 1) = ct;
    R(0, 2) = -cp * st;
    R(1, 0) = -ct * sp;
    R(1, 1) = st;
    R(1, 2) = cp * ct;
    R(2, 0) = cp;
    R(2, 1) = 0.0;
    R(2, 2) = sp;
    return R;
}

/// Eq.44 — Homogeneous transformation matrix that maps world-frame coordinates to
/// target-frame coordinates (4×4 CV_64F matrix).
///
/// world_T_target = [ R_T^T   -R_T^T·C ]
///                  [ 0 0 0   1        ]
///
/// Verifies: world_T_target · (C, 1)^T = (0, 0, 0, 1)^T
inline cv::Mat computeWorldTTarget(double theta, double phi, double r, double L, double h) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double cp = std::cos(phi);
    double sp = std::sin(phi);

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    // Top-left 3×3 block = R_T^T
    T.at<double>(0, 0) = st * sp;
    T.at<double>(0, 1) = -ct * sp;
    T.at<double>(0, 2) = cp;
    T.at<double>(1, 0) = ct;
    T.at<double>(1, 1) = st;
    T.at<double>(1, 2) = 0.0;
    T.at<double>(2, 0) = -cp * st;
    T.at<double>(2, 1) = cp * ct;
    T.at<double>(2, 2) = sp;

    // Translation column (explicit form from Eq.44)
    T.at<double>(0, 3) = L - (h + L) * cp;
    T.at<double>(1, 3) = -r;
    T.at<double>(2, 3) = -(h + L) * sp;

    return T;
}

}  // namespace omr_controller
