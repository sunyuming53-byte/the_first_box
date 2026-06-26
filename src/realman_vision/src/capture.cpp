#include "realman_vision/capture.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <format>
#include <fstream>
#include <string>

namespace rm::vision {

// ──────────────────────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────────────────────
namespace {

[[nodiscard]] auto timestamp_ms() -> int64_t {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

} // anonymous namespace

// ──────────────────────────────────────────────────────────────
// FrameCapture
// ──────────────────────────────────────────────────────────────

FrameCapture::FrameCapture(const CaptureConfig& cfg) : cfg_{cfg} {
    namespace fs = std::filesystem;

    fs::create_directories(cfg_.output_dir / "images");

    if (cfg_.save_depth) {
        fs::create_directories(cfg_.output_dir / "depth");
    }
}

auto FrameCapture::draw_overlay(const camera::CameraFrame& frame,
                                const std::vector<cv::Point2f>& corners,
                                int saved_count,
                                int total_required) -> cv::Mat
{
    cv::Mat display = frame.color.clone();

    // ── board corners ──
    if (!corners.empty()) {
        for (const auto& pt : corners) {
            cv::circle(display, pt, 5, cv::Scalar(0, 255, 0), -1);   // green filled
        }
    } else {
        // no corners detected — show a red warning indicator
        cv::Point centre(display.cols / 2, display.rows / 2);
        cv::circle(display, centre, 24, cv::Scalar(0, 0, 255), 2);
        cv::line(display,
                 cv::Point(centre.x - 17, centre.y - 17),
                 cv::Point(centre.x + 17, centre.y + 17),
                 cv::Scalar(0, 0, 255), 2);
        cv::line(display,
                 cv::Point(centre.x + 17, centre.y - 17),
                 cv::Point(centre.x - 17, centre.y + 17),
                 cv::Scalar(0, 0, 255), 2);
    }

    // ── frame counter ──
    auto count_str = std::format("{}/{}", saved_count, total_required);
    cv::putText(display, count_str, cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 255), 2);

    // ── instructions ──
    cv::putText(display, "[S] Save  [Q] Quit",
                cv::Point(20, display.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 255, 255), 2);

    return display;
}

void FrameCapture::save(const camera::CameraFrame& frame, int index) {
    namespace fs = std::filesystem;

    // ── colour image ──
    auto img_dir = cfg_.output_dir / "images";
    fs::create_directories(img_dir);
    auto img_path = img_dir / std::format("{:05d}.jpg", index);
    cv::imwrite(img_path.string(), frame.color);

    // ── depth image (16-bit PNG) ──
    if (cfg_.save_depth && !frame.depth.empty()) {
        auto depth_dir = cfg_.output_dir / "depth";
        fs::create_directories(depth_dir);
        auto depth_path = depth_dir / std::format("{:05d}.png", index);
        cv::imwrite(depth_path.string(), frame.depth);
    }
}

void FrameCapture::save_with_pose(const camera::CameraFrame& frame,
                                   int index,
                                   const ArmPose& pose)
{
    save(frame, index);

    auto csv_path = cfg_.output_dir / "robot_poses.csv";
    bool write_header = !std::filesystem::exists(csv_path);

    auto file = std::ofstream(csv_path, std::ios::app);
    if (!file) return;

    if (write_header) {
        file << "timestamp,tx,ty,tz,rx,ry,rz\n";
    }

    file << std::format("{},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n",
                        timestamp_ms(),
                        pose.tx, pose.ty, pose.tz,
                        pose.rx, pose.ry, pose.rz);
}

} // namespace rm::vision
