#pragma once

#include <omr_controller/types.hpp>
#include <omr_vision/camera/stream.hpp>
#include <omr_vision/camera/types.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

#include <array>
#include <memory>
#include <optional>
#include <vector>

namespace omr_controller {

class ICamera {
public:
    virtual ~ICamera() = default;

    virtual std::optional<cv::Mat> next() = 0;
    virtual omr_vision::camera::CameraIntrinsics depth_intrinsics() const = 0;
};

class CameraStreamAdapter : public ICamera {
public:
    explicit CameraStreamAdapter(const omr_vision::camera::CameraConfig& cfg);

    std::optional<cv::Mat> next() override;
    omr_vision::camera::CameraIntrinsics depth_intrinsics() const override;

private:
    std::unique_ptr<omr_vision::camera::CameraStream> stream_;
};

class VisionClient {
public:
    explicit VisionClient(std::unique_ptr<ICamera> camera);

    std::vector<DetectionResult> detect(const cv::Mat& frame);
    std::optional<std::vector<DetectionResult>> next_detection();

    void set_params(const std::array<int, 3>& lower_hsv,
                    const std::array<int, 3>& upper_hsv,
                    int aruco_dict,
                    double aruco_marker_size_m);

private:
    std::unique_ptr<ICamera> camera_;

    std::array<int, 3> color_lower_hsv_{0, 50, 50};
    std::array<int, 3> color_upper_hsv_{10, 255, 255};
    int aruco_dict_{cv::aruco::DICT_6X6_250};
    double aruco_marker_size_m_{0.05};
};

}  // namespace omr_controller
