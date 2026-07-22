#include "omr_vision/camera/stream.hpp"

#include <cstring>
#include <exception>
#include <iostream>

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>

#include "gst_streamer.hpp"

namespace omr_vision::camera {

// ──────────────────────────────────────────────────────────────
// CameraStream::Impl — PIMPL hiding librealsense2 from public API
// ──────────────────────────────────────────────────────────────
class CameraStream::Impl {
public:
    explicit Impl(const CameraConfig& cfg)
        : enable_depth_{cfg.enable_depth},
          capture_fps_{cfg.fps},
          streaming_fps_{cfg.streaming_fps} {
        // Validate streaming_fps: default to capture fps if invalid, cap to capture fps.
        // Note: streaming_fps must evenly divide capture_fps for accurate decimation
        // (integer division truncation; e.g. 30/20=1 → streams every frame instead of ~20fps).
        if (streaming_fps_ <= 0) streaming_fps_ = capture_fps_;
        if (streaming_fps_ > capture_fps_) streaming_fps_ = capture_fps_;

        if (cfg.streaming_enabled) {
            gst_streamer_ = std::make_unique<GstStreamer>(
                cfg.width, cfg.height, streaming_fps_, cfg.streaming_bitrate_kbps, cfg.streaming_url);
            gst_streamer_->start();
            std::cerr << "GStreamer streaming enabled → " << cfg.streaming_url << std::endl;
        }

        config_.enable_stream(RS2_STREAM_COLOR, cfg.width, cfg.height, RS2_FORMAT_BGR8, cfg.fps);
        if (enable_depth_) {
            config_.enable_stream(RS2_STREAM_DEPTH, cfg.width, cfg.height, RS2_FORMAT_Z16, cfg.fps);
        }

        try {
            profile_ = pipeline_.start(config_);
            has_frames_ = true;
        } catch (...) {
            has_frames_ = false;
        }

        if (enable_depth_ && has_frames_) {
            auto sensor = profile_.get_device().first<rs2::depth_sensor>();
            depth_scale_ = sensor.get_depth_scale();
        }
    }

    [[nodiscard]] auto next() -> std::optional<CameraFrame> {
        if (!has_frames_) return std::nullopt;

        while (true) {
            // ── wait for the next frameset ──
            rs2::frameset frames;
            try {
                frames = pipeline_.wait_for_frames();
            } catch (const rs2::error&) {
                has_frames_ = false;
                return std::nullopt;
            }

            // ── align depth to colour (only when depth is active) ──
            rs2::frameset aligned;
            if (enable_depth_) {
                aligned = align_.process(frames);
            } else {
                aligned = frames;
            }

            auto color_frame = aligned.get_color_frame();
            if (!color_frame) continue;  // should never happen, but be safe

            // ── build output ──
            CameraFrame out;
            out.frame_id = frame_counter_++;

            const int w = color_frame.get_width();
            const int h = color_frame.get_height();

            // colour (BGR, 8-bit)
            out.color = cv::Mat(h, w, CV_8UC3);
            std::memcpy(out.color.data, color_frame.get_data(),
                        static_cast<std::size_t>(w) * h * 3);

            // ── streaming push (decimated) ──
            if (gst_streamer_) {
                if (out.frame_id % (capture_fps_ / streaming_fps_) == 0) {
                    gst_streamer_->push(out.color);
                }
            }

            // ── color intrinsics (captured once from the first color frame) ──
            if (!color_intrinsics_captured_) {
                auto vsp = color_frame.get_profile().as<rs2::video_stream_profile>();
                auto i = vsp.get_intrinsics();
                color_intrinsics_.K =
                    (cv::Mat_<double>(3, 3) << i.fx, 0.0, i.ppx, 0.0, i.fy, i.ppy, 0.0, 0.0, 1.0);
                color_intrinsics_.dist_coeff = (cv::Mat_<double>(1, 5) << i.coeffs[0], i.coeffs[1],
                                                i.coeffs[2], i.coeffs[3], i.coeffs[4]);
                color_intrinsics_captured_ = true;
            }
            out.color_intrinsics = color_intrinsics_;

            // depth (16-bit mm, aligned)
            if (enable_depth_) {
                auto depth_frame = aligned.get_depth_frame();
                if (!depth_frame) continue;  // skip frames with no depth

                out.depth = cv::Mat(h, w, CV_16UC1);
                std::memcpy(out.depth.data, depth_frame.get_data(),
                            static_cast<std::size_t>(w) * h * 2);

                // convert raw units → millimetres
                const double scale_to_mm = depth_scale_ * 1000.0;
                if (std::abs(scale_to_mm - 1.0) > 1e-6) {
                    out.depth.convertTo(out.depth, CV_16UC1, scale_to_mm);
                }

                // ── depth intrinsics (captured once from the first valid depth frame) ──
                if (!depth_intrinsics_captured_) {
                    auto vsp = depth_frame.get_profile().as<rs2::video_stream_profile>();
                    auto i = vsp.get_intrinsics();
                    depth_intrinsics_.K = (cv::Mat_<double>(3, 3) << i.fx, 0.0, i.ppx, 0.0, i.fy,
                                           i.ppy, 0.0, 0.0, 1.0);
                    depth_intrinsics_.dist_coeff =
                        (cv::Mat_<double>(1, 5) << i.coeffs[0], i.coeffs[1], i.coeffs[2],
                         i.coeffs[3], i.coeffs[4]);
                    depth_intrinsics_captured_ = true;
                }
                out.depth_intrinsics = depth_intrinsics_;
            }

            return out;
        }
    }

    [[nodiscard]] auto color_intrinsics() const -> CameraIntrinsics { return color_intrinsics_; }

    [[nodiscard]] auto depth_intrinsics() const -> CameraIntrinsics { return depth_intrinsics_; }

    [[nodiscard]] auto active() const -> bool { return has_frames_; }

    ~Impl() {
        try {
            pipeline_.stop();
        } catch (...) {  // NOLINT(bugprone-empty-catch)
            // Destructor must not throw; silently ignore pipeline stop errors.
        }
    }

private:
    rs2::pipeline pipeline_;
    rs2::config config_;
    rs2::align align_{RS2_STREAM_COLOR};
    rs2::pipeline_profile profile_;
    bool enable_depth_{false};
    float depth_scale_{0.001F};
    bool has_frames_{false};
    std::unique_ptr<GstStreamer> gst_streamer_;
    int64_t frame_counter_{0};
    CameraIntrinsics color_intrinsics_;
    CameraIntrinsics depth_intrinsics_;
    bool color_intrinsics_captured_{false};
    bool depth_intrinsics_captured_{false};
    int capture_fps_{30};
    int streaming_fps_{15};
};

// ──────────────────────────────────────────────────────────────
// CameraStream — public API
// ──────────────────────────────────────────────────────────────

CameraStream::CameraStream(const CameraConfig& cfg) : impl_{std::make_unique<Impl>(cfg)} {}

CameraStream::~CameraStream() = default;

CameraStream::CameraStream(CameraStream&&) noexcept = default;
auto CameraStream::operator=(CameraStream&&) noexcept -> CameraStream& = default;

auto CameraStream::next() -> std::optional<CameraFrame> { return impl_->next(); }

auto CameraStream::color_intrinsics() const -> CameraIntrinsics {
    return impl_->color_intrinsics();
}

auto CameraStream::depth_intrinsics() const -> CameraIntrinsics {
    return impl_->depth_intrinsics();
}

CameraStream::operator bool() const { return impl_ != nullptr && impl_->active(); }

}  // namespace omr_vision::camera
