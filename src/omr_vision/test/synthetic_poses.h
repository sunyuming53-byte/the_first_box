#pragma once

#include <array>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace omr_vision::test {

/// Generate N random arm poses {tx,ty,tz,rx,ry,rz} in meters / radians.
///
/// Translation: uniform in [-0.3, 0.3]
/// Rotation:    uniform RPY in [-π, π]
///
/// Rotation diversity: at least 3 poses must have >30° spread in each axis
/// (max-min ≥ min_rotation_spread_deg in all of rx, ry, rz).  Regenerates
/// up to 100 times if the constraint is not met.
///
/// @throws std::runtime_error on persistent failure to meet diversity.
[[nodiscard]] auto generate_random_poses(int N, double min_rotation_spread_deg = 30.0)
    -> std::vector<std::array<double, 6>>;

/// Generate a random hand-eye (camera-to-flange) transform as a 4×4
/// homogeneous matrix (CV_64F).
///
/// Rotation:    random axis-angle via cv::Rodrigues (uniform axis, angle ∈ [0,π])
/// Translation: uniform in [-0.2, 0.2] meters on each axis
[[nodiscard]] auto generate_random_handeye_transform() -> cv::Mat;

/// Add zero-mean Gaussian noise with standard deviation `sigma` (pixels)
/// to every point in-place.
void inject_noise(std::vector<cv::Point2f>& points, double sigma);

}  // namespace omr_vision::test
