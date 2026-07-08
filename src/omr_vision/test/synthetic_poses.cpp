#include "synthetic_poses.h"

#include <cmath>

#include <random>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace omr_vision::test {

// ──────────────────────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────────────────────
namespace {

/// Thread-local Mersenne Twister seeded from random_device.
[[nodiscard]] auto rng() -> std::mt19937& {
    thread_local auto gen = [] {
        std::random_device rd;
        // seed_seq from multiple random_device values for good entropy
        std::array<std::random_device::result_type, 8> seeds{};
        for (auto& s : seeds) {
            s = rd();
        }
        std::seed_seq seq(seeds.begin(), seeds.end());
        return std::mt19937{seq};
    }();
    return gen;
}

/// Check rotation diversity: max-min ≥ min_spread_rad in all 3 axes (rx,ry,rz).
[[nodiscard]] auto has_rotation_spread(const std::vector<std::array<double, 6>>& poses,
                                       double min_spread_rad) -> bool {
    if (poses.size() < 3) return false;

    double min_rx = poses[0][3];
    double max_rx = poses[0][3];
    double min_ry = poses[0][4];
    double max_ry = poses[0][4];
    double min_rz = poses[0][5];
    double max_rz = poses[0][5];

    for (const auto& p : poses) {
        min_rx = std::min(min_rx, p[3]);
        max_rx = std::max(max_rx, p[3]);
        min_ry = std::min(min_ry, p[4]);
        max_ry = std::max(max_ry, p[4]);
        min_rz = std::min(min_rz, p[5]);
        max_rz = std::max(max_rz, p[5]);
    }

    return (max_rx - min_rx) >= min_spread_rad && (max_ry - min_ry) >= min_spread_rad &&
           (max_rz - min_rz) >= min_spread_rad;
}

}  // anonymous namespace

// ──────────────────────────────────────────────────────────────
// generate_random_poses
// ──────────────────────────────────────────────────────────────

auto generate_random_poses(int N, double min_rotation_spread_deg)
    -> std::vector<std::array<double, 6>> {
    if (N < 1) return {};

    constexpr double kTransLimit = 0.3;  // meters
    constexpr double kRotLimit = M_PI;   // radians
    const double kMinSpread = min_rotation_spread_deg * M_PI / 180.0;

    auto& gen = rng();
    std::uniform_real_distribution<double> trans_dist(-kTransLimit, kTransLimit);
    std::uniform_real_distribution<double> rot_dist(-kRotLimit, kRotLimit);

    constexpr int kMaxRetries = 100;

    for (int attempt = 0; attempt < kMaxRetries; ++attempt) {
        std::vector<std::array<double, 6>> poses;
        poses.reserve(static_cast<std::size_t>(N));

        for (int i = 0; i < N; ++i) {
            poses.push_back({
                trans_dist(gen),  // tx
                trans_dist(gen),  // ty
                trans_dist(gen),  // tz
                rot_dist(gen),    // rx
                rot_dist(gen),    // ry
                rot_dist(gen)     // rz
            });
        }

        // If N < 3 the diversity check is impossible; return what we have.
        if (N < 3) return poses;

        if (has_rotation_spread(poses, kMinSpread)) {
            return poses;
        }
    }

    throw std::runtime_error(
        "generate_random_poses: failed to meet rotation diversity "
        "constraint after " +
        std::to_string(kMaxRetries) + " attempts");
}

// ──────────────────────────────────────────────────────────────
// generate_random_handeye_transform
// ──────────────────────────────────────────────────────────────

auto generate_random_handeye_transform() -> cv::Mat {
    constexpr double kTransLimit = 0.2;  // meters

    auto& gen = rng();

    // ── random rotation via axis-angle ──
    // Draw a random direction uniformly on the unit sphere
    std::normal_distribution<double> normal{0.0, 1.0};
    double ax = normal(gen);
    double ay = normal(gen);
    double az = normal(gen);
    double len = std::sqrt((ax * ax) + (ay * ay) + (az * az));
    if (len < 1e-12) {
        ax = 1.0;
        ay = 0.0;
        az = 0.0;
        len = 1.0;
    }

    cv::Mat axis = (cv::Mat_<double>(3, 1) << ax / len, ay / len, az / len);

    // Random angle in [0, π]
    std::uniform_real_distribution<double> angle_dist(0.0, M_PI);
    double angle = angle_dist(gen);

    cv::Mat R;
    cv::Rodrigues(axis * angle, R);  // R is 3×3 CV_64F

    // ── random translation ──
    std::uniform_real_distribution<double> t_dist(-kTransLimit, kTransLimit);
    double tx = t_dist(gen);
    double ty = t_dist(gen);
    double tz = t_dist(gen);

    // ── compose 4×4 homogeneous matrix ──
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat roi = T(cv::Rect(0, 0, 3, 3));
    R.copyTo(roi);
    T.at<double>(0, 3) = tx;
    T.at<double>(1, 3) = ty;
    T.at<double>(2, 3) = tz;

    return T;
}

// ──────────────────────────────────────────────────────────────
// inject_noise
// ──────────────────────────────────────────────────────────────

void inject_noise(std::vector<cv::Point2f>& points, double sigma) {
    if (sigma <= 0.0 || points.empty()) return;

    auto& gen = rng();
    std::normal_distribution<double> noise{0.0, sigma};

    for (auto& pt : points) {
        pt.x += static_cast<float>(noise(gen));
        pt.y += static_cast<float>(noise(gen));
    }
}

}  // namespace omr_vision::test
