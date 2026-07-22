#include "omr_vision/calibration/format_polyfill.hpp"

#include <cmath>
#include <cstdlib>

#include <array>
#include <iostream>
#include <string_view>

#include <omr_vision/camera/stream.hpp>
#include <omr_vision/detection/white_label.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace {

void drawCorners(cv::Mat& image, const std::array<cv::Point2f, 4>& corners, const cv::Scalar& color,
                 int thickness = 2) {
    for (int i = 0; i < 4; ++i) {
        cv::line(image, corners[static_cast<size_t>(i)], corners[static_cast<size_t>((i + 1) % 4)],
                 color, thickness);
        cv::circle(image, corners[static_cast<size_t>(i)], 4, cv::Scalar(0, 0, 255), -1);
    }
}

void drawPose(cv::Mat& image, const omr_vision::detection::WhiteLabelPose& pose,
              const omr_vision::camera::CameraIntrinsics& intrinsics,
              const omr_vision::detection::WhiteLabelConfig& cfg) {
    drawCorners(image, pose.detection.corners_img, cv::Scalar(0, 255, 0));

    cv::Mat dist = intrinsics.dist_coeff;
    if (dist.empty()) {
        dist = cv::Mat::zeros(1, 5, CV_64F);
    }
    const double axis_len = std::min(cfg.width_m, cfg.height_m) * 2.0;
    cv::drawFrameAxes(image, intrinsics.K, dist, pose.rvec, pose.tvec,
                      static_cast<float>(axis_len));

    const auto hud =
        std::format("t=({:.3f},{:.3f},{:.3f}) m  reproj={:.2f}px  content={:.2f}", pose.tvec[0],
                    pose.tvec[1], pose.tvec[2], pose.reproj_error_px, pose.detection.content_score);
    cv::putText(image, hud, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(0, 255, 255), 2);
}

}  // namespace

int main(int argc, char* argv[]) {
    omr_vision::detection::WhiteLabelConfig cfg;
    bool show_mask = true;
    int confirm_frames = 3;
    double confirm_shift_px = 50.0;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        auto need_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Error: " << name << " requires a value\n";
                std::exit(1);
            }
            return argv[++i];
        };
        if (arg == "--width-m") {
            try {
                cfg.width_m = std::stod(need_value("--width-m"));
            } catch (const std::exception&) {
                std::cerr << "Error: --width-m requires a numeric value\n";
                return 1;
            }
        } else if (arg == "--height-m") {
            try {
                cfg.height_m = std::stod(need_value("--height-m"));
            } catch (const std::exception&) {
                std::cerr << "Error: --height-m requires a numeric value\n";
                return 1;
            }
        } else if (arg == "--threshold") {
            try {
                cfg.binary_threshold = std::stoi(need_value("--threshold"));
            } catch (const std::exception&) {
                std::cerr << "Error: --threshold requires an integer value\n";
                return 1;
            }
        } else if (arg == "--confirm-frames") {
            try {
                confirm_frames = std::stoi(need_value("--confirm-frames"));
            } catch (const std::exception&) {
                std::cerr << "Error: --confirm-frames requires an integer value\n";
                return 1;
            }
        } else if (arg == "--confirm-shift-px") {
            try {
                confirm_shift_px = std::stod(need_value("--confirm-shift-px"));
            } catch (const std::exception&) {
                std::cerr << "Error: --confirm-shift-px requires a numeric value\n";
                return 1;
            }
        } else if (arg == "--no-mask") {
            show_mask = false;
        } else if (arg == "--pnp-method") {
            const std::string_view method = need_value("--pnp-method");
            if (method == "ippe") {
                cfg.pnp_method = omr_vision::detection::WhiteLabelPnpMethod::Ippe;
            } else if (method == "iterative") {
                cfg.pnp_method = omr_vision::detection::WhiteLabelPnpMethod::Iterative;
            } else {
                std::cerr << "Error: --pnp-method must be 'ippe' or 'iterative'\n";
                return 1;
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: detect_white_label [--width-m 0.020] [--height-m 0.0775] "
                         "[--threshold 180] [--confirm-frames 3] [--confirm-shift-px 50] "
                         "[--pnp-method ippe|iterative] [--no-mask]\n";
            return 0;
        }
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    omr_vision::camera::CameraConfig cam_cfg;
    cam_cfg.enable_depth = true;
    omr_vision::camera::CameraStream stream(cam_cfg);
    if (!stream) {
        std::cerr << "Error: failed to open RealSense camera\n";
        return 1;
    }

    omr_vision::detection::WhiteLabelTemporalConfirm confirmer(confirm_frames, confirm_shift_px);

    const char* method_name =
        (cfg.pnp_method == omr_vision::detection::WhiteLabelPnpMethod::Ippe) ? "ippe" : "iterative";
    std::cout << std::format(
        "detect_white_label running (label {:.1f}x{:.1f} mm, threshold={}, pnp={}, "
        "confirm={} frames). Press q to quit.\n",
        cfg.width_m * 1000.0, cfg.height_m * 1000.0, cfg.binary_threshold, method_name,
        confirm_frames);

    while (true) {
        auto frame = stream.next();
        if (!frame.has_value()) {
            std::cerr << "Error: camera stream ended\n";
            break;
        }

        cfg.focal_length_px = frame->color_intrinsics.fy();

        cv::Mat mask;
        auto raw =
            omr_vision::detection::detectWhiteLabel(frame->color, cfg, show_mask ? &mask : nullptr);
        auto confirmed = confirmer.update(raw);

        cv::Mat vis = frame->color.clone();
        if (confirmed.has_value()) {
            auto pose = omr_vision::detection::estimateWhiteLabelPoseFromDetection(
                *confirmed, frame->color_intrinsics, cfg);
            if (pose.has_value()) {
                drawPose(vis, *pose, frame->color_intrinsics, cfg);
                cv::putText(
                    vis,
                    std::format("confirmed {}/{}", confirmer.streak(), confirmer.requiredFrames()),
                    cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            } else {
                drawCorners(vis, confirmed->corners_img, cv::Scalar(0, 165, 255));
                cv::putText(vis, "confirmed but PnP failed", cv::Point(20, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
            }
        } else if (raw.has_value()) {
            drawCorners(vis, raw->corners_img, cv::Scalar(0, 255, 255), 1);
            cv::putText(
                vis,
                std::format("confirming {}/{}", confirmer.streak(), confirmer.requiredFrames()),
                cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        } else {
            cv::putText(vis, "label not detected", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("white_label_pnp", vis);
        if (show_mask && !mask.empty()) {
            cv::imshow("white_label_mask", mask);
        }
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }
    }

    return 0;
}
