#pragma once

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

namespace omr_vision::camera {

class GstStreamer {
public:
    GstStreamer(int width, int height, int fps, int bitrate_kbps,
                std::string rtsp_url);
    ~GstStreamer();

    GstStreamer(const GstStreamer&) = delete;
    auto operator=(const GstStreamer&) -> GstStreamer& = delete;
    GstStreamer(GstStreamer&&) noexcept = delete;
    auto operator=(GstStreamer&&) noexcept -> GstStreamer& = delete;

    auto start() -> void;
    auto stop() -> void;
    auto push(const cv::Mat& frame) -> void;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace omr_vision::camera
