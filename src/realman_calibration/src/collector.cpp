#include "realman_calibration/collector.hpp"
#include "realman_vision/capture.hpp"
#include "realman_vision/camera/stream.hpp"
#include "realman/core/arm.hpp"
#include "realman/core/error.hpp"
#include "realman/motion/types.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include "realman_calibration/format_polyfill.hpp"
#include <iostream>
#include <span>
#include <thread>

namespace rm::calib {

// ──────────────────────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────────────────────
namespace {

constexpr auto kRotationThresholdRad = 30.0 * M_PI / 180.0;  // 30°

/// Check that saved poses span >30° across X, Y, Z rotation axes.
[[nodiscard]] auto rotation_diversity_ok(std::span<const std::array<double, 6>> poses) -> bool {
    if (poses.size() < 3) return false;                     // can't judge yet

    double min_rx = poses[0][3], max_rx = poses[0][3];
    double min_ry = poses[0][4], max_ry = poses[0][4];
    double min_rz = poses[0][5], max_rz = poses[0][5];

    for (auto i = 1uz; i < poses.size(); ++i) {            // intentional unsigned
        min_rx = std::min(min_rx, poses[i][3]); max_rx = std::max(max_rx, poses[i][3]);
        min_ry = std::min(min_ry, poses[i][4]); max_ry = std::max(max_ry, poses[i][4]);
        min_rz = std::min(min_rz, poses[i][5]); max_rz = std::max(max_rz, poses[i][5]);
    }

    return (max_rx - min_rx > kRotationThresholdRad) &&
           (max_ry - min_ry > kRotationThresholdRad) &&
           (max_rz - min_rz > kRotationThresholdRad);
}

} // anonymous namespace

// ──────────────────────────────────────────────────────────────
// CalibDataCollector::Impl
// ──────────────────────────────────────────────────────────────

class CalibDataCollector::Impl {
public:
    explicit Impl(const CalibDataConfig& cfg)
        : arm_cfg_{cfg.arm_ip, 8080, rm::ArmModel::RM_65},
          arm_{arm_cfg_},
          cam_{cfg.camera},
          capture_{rm::vision::CaptureConfig{cfg.output_dir, cfg.total_images, false, cfg.board_size}},
          board_size_{cfg.board_size},
          total_{cfg.total_images},
          output_dir_{cfg.output_dir}
    {}

    [[nodiscard]] auto run(std::vector<rm::JointPosition> waypoints) -> Result<CalibSession> {
        namespace fs = std::filesystem;
        fs::create_directories(output_dir_);

        std::vector<std::array<double, 6>> saved_poses;
        std::vector<cv::Point2f> corners;
        int saved_count = 0;

        const bool has_display = (std::getenv("DISPLAY") != nullptr);

        if (!waypoints.empty()) {
            // ========================================
            // Auto-collection mode
            // ========================================
            std::cout << std::format("Auto-collection: {} waypoints, target {} frames\n",
                                     waypoints.size(), total_);

            const auto window_name = std::format("Hand-Eye Calibration — {}", output_dir_.string());
            if (has_display) cv::namedWindow(window_name, cv::WINDOW_NORMAL);

            for (size_t idx = 0; idx < waypoints.size(); ++idx) {
                // Stop if we have enough frames with good diversity
                if (saved_count >= total_ && rotation_diversity_ok(saved_poses)) break;

                // Move arm to waypoint — skip on failure
                try {
                    arm_.moveJ(waypoints[idx], 50, true);
                } catch (const rm::ArmError& e) {
                    std::cerr << std::format("Waypoint {} unreachable, skipping: {}\n", idx, e.what());
                    continue;
                }

                // Allow arm to settle
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                // Grab camera frame
                auto frame_opt = cam_.next();
                if (!frame_opt.has_value()) {
                    std::cerr << std::format("Camera frame grab failed at waypoint {}\n", idx);
                    continue;
                }

                auto& frame = *frame_opt;
                cv::Mat gray;
                cv::cvtColor(frame.color, gray, cv::COLOR_BGR2GRAY);

                // Detect chessboard
                corners.clear();
                bool found = cv::findChessboardCorners(gray, board_size_, corners);
                if (found) {
                    cv::cornerSubPix(gray, corners, cv::Size(5, 5),
                                     cv::Size(-1, -1),
                                     cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
                                                      30, 0.1));
                }

                // Auto-capture on detection
                if (found) {
                    auto pose = arm_.toolPose();
                    rm::vision::ArmPose arm_pose{pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw};
                    capture_.save_with_pose(frame, saved_count, arm_pose);

                    saved_poses.push_back({pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw});
                    ++saved_count;

                    std::cout << std::format("Auto-captured {} / {} (waypoint {}, {} corners)\n",
                                             saved_count, total_, idx, corners.size());
                } else {
                    std::cout << std::format("No board at waypoint {}\n", idx);
                }

                // Visual feedback (if display available)
                if (has_display) {
                    auto display = capture_.draw_overlay(frame, corners, saved_count, total_);
                    auto pose = arm_.toolPose();
                    auto arm_text = std::format(
                        "Arm: x={:.3f} y={:.3f} z={:.3f} roll={:.1f} pitch={:.1f} yaw={:.1f}",
                        pose.x, pose.y, pose.z,
                        pose.roll * 180.0 / M_PI,
                        pose.pitch * 180.0 / M_PI,
                        pose.yaw * 180.0 / M_PI);
                    cv::putText(display, arm_text, cv::Point(20, 80),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(200, 200, 200), 1);

                    auto status_text = found
                        ? std::format("Auto: Board OK  corners={}", corners.size())
                        : std::format("Auto: Board NOT FOUND");
                    auto status_color = found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                    cv::putText(display, status_text, cv::Point(20, 110),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, status_color, 1);

                    cv::imshow(window_name, display);
                    cv::waitKey(1);  // brief refresh
                }
            }

            if (has_display) cv::destroyWindow(window_name);

            std::cout << std::format("Auto-collection done: {} frames saved, diversity {}\n",
                                     saved_count,
                                     rotation_diversity_ok(saved_poses) ? "OK" : "insufficient");

        } else {
            // ========================================
            // Interactive mode
            // ========================================
            const auto window_name = std::format("Hand-Eye Calibration — {}", output_dir_.string());
            if (has_display) cv::namedWindow(window_name, cv::WINDOW_NORMAL);

            while (true) {
                auto frame_opt = cam_.next();
                if (!frame_opt.has_value()) {
                    return Unexpected<std::string>("Camera stream ended unexpectedly");
                }

                auto& frame = *frame_opt;
                cv::Mat gray;
                cv::cvtColor(frame.color, gray, cv::COLOR_BGR2GRAY);

                // find chessboard corners
                corners.clear();
                bool found = cv::findChessboardCorners(gray, board_size_, corners);

                if (found) {
                    cv::cornerSubPix(gray, corners, cv::Size(5, 5),
                                     cv::Size(-1, -1),
                                     cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
                                                      30, 0.1));
                }

                int key = -1;
                if (has_display) {
                    // draw overlay
                    auto display = capture_.draw_overlay(frame, corners, saved_count, total_);

                    // arm pose status text
                    auto pose = arm_.toolPose();
                    auto arm_text = std::format(
                        "Arm: x={:.3f} y={:.3f} z={:.3f} roll={:.1f} pitch={:.1f} yaw={:.1f}",
                        pose.x, pose.y, pose.z,
                        pose.roll * 180.0 / M_PI,
                        pose.pitch * 180.0 / M_PI,
                        pose.yaw * 180.0 / M_PI);
                    cv::putText(display, arm_text, cv::Point(20, 80),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(200, 200, 200), 1);

                    // board detection status
                    auto status_text = found
                        ? std::format("Board: OK  corners={}", corners.size())
                        : std::format("Board: NOT FOUND");
                    auto status_color = found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
                    cv::putText(display, status_text, cv::Point(20, 110),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, status_color, 1);

                    cv::imshow(window_name, display);
                    key = static_cast<int>(cv::waitKey(30));
                } else {
                    // Headless: log status to stdout instead of display
                    auto pose = arm_.toolPose();
                    std::cout << std::format(
                        "Headless: saved={}/{} arm=({:.3f},{:.3f},{:.3f}) board={}\n",
                        saved_count, total_,
                        pose.x, pose.y, pose.z,
                        found ? "OK" : "NOT FOUND");
                    // Non-blocking wait to avoid busy-loop (stdin polling)
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                }

                // ── handle key input ──
                if (key == 's' || key == 'S') {
                    auto pose = arm_.toolPose();
                    rm::vision::ArmPose arm_pose{pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw};
                    capture_.save_with_pose(frame, saved_count, arm_pose);

                    saved_poses.push_back({pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw});
                    ++saved_count;

                    std::cout << std::format("Saved frame {} / {}\n", saved_count, total_);

                    // check if target reached
                    if (saved_count >= total_) {
                        if (rotation_diversity_ok(saved_poses)) {
                            std::cout << std::format(
                                "Target reached ({} frames). Rotation diversity OK.\n",
                                saved_count);
                            break;  // ← exit loop successfully
                        } else {
                            std::cout << std::format(
                                "WARNING: {} frames saved but rotation diversity insufficient.\n"
                                "  Need >30° range across ALL three axes (X/Y/Z).\n"
                                "  Continue capturing or quit.\n",
                                saved_count);
                            // continue capturing beyond target
                        }
                    }
                } else if (key == 'q' || key == 'Q' || key == 27) {
                    std::cout << std::format("Quit requested.  Saved {} frames.\n", saved_count);
                    break;
                }
            }

            if (has_display) cv::destroyWindow(window_name);
        }

        // clean shutdown
        arm_.stop();
        // CameraStream and FrameCapture destructors handle cleanup

        auto session = CalibSession{
            .dir = output_dir_,
            .arm_poses = std::move(saved_poses),
        };

        return session;
    }

private:
    rm::ArmConfig     arm_cfg_;
    rm::Arm           arm_;
    rm::vision::camera::CameraStream cam_;
    rm::vision::FrameCapture      capture_;
    cv::Size          board_size_;
    int               total_;
    std::filesystem::path output_dir_;
};

// ──────────────────────────────────────────────────────────────
// CalibDataCollector (PIMPL facade)
// ──────────────────────────────────────────────────────────────

CalibDataCollector::CalibDataCollector(const CalibDataConfig& cfg)
    : impl_{std::make_unique<Impl>(cfg)}
{}

CalibDataCollector::~CalibDataCollector() = default;

auto CalibDataCollector::run(std::vector<rm::JointPosition> waypoints) -> Result<CalibSession> {
    return impl_->run(std::move(waypoints));
}

} // namespace rm::calib
