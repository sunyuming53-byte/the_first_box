#include "omr_controller/clients/vision_client.hpp"

#include <opencv2/imgproc.hpp>

namespace omr_controller {

CameraStreamAdapter::CameraStreamAdapter(const omr_vision::camera::CameraConfig& cfg)
    : stream_(std::make_unique<omr_vision::camera::CameraStream>(cfg)) {}

std::optional<cv::Mat> CameraStreamAdapter::next() {
    auto frame = stream_->next();
    if (frame.has_value()) {
        return frame->color;
    }
    return std::nullopt;
}

omr_vision::camera::CameraIntrinsics CameraStreamAdapter::depth_intrinsics() const {
    return stream_->depth_intrinsics();
}

VisionClient::VisionClient(std::unique_ptr<ICamera> camera) : camera_(std::move(camera)) {}

std::vector<DetectionResult> VisionClient::detect(const cv::Mat& frame) {
    std::vector<DetectionResult> results;

    // Stage 1: Color thresholding
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(color_lower_hsv_[0], color_lower_hsv_[1], color_lower_hsv_[2]),
                cv::Scalar(color_upper_hsv_[0], color_upper_hsv_[1], color_upper_hsv_[2]), mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        if (contour.empty()) continue;
        cv::Rect bbox = cv::boundingRect(contour);
        DetectionResult r;
        r.label = "color";
        r.center = cv::Point2d(bbox.x + bbox.width / 2.0, bbox.y + bbox.height / 2.0);
        r.confidence = 1.0;
        results.push_back(r);
    }

    // Stage 2: Aruco marker detection
    auto dict = cv::aruco::getPredefinedDictionary(aruco_dict_);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dict, corners, ids);

    if (!ids.empty()) {
        auto intrinsics = camera_->depth_intrinsics();
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, static_cast<float>(aruco_marker_size_m_),
                                             intrinsics.K, intrinsics.dist_coeff, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); ++i) {
            DetectionResult r;
            r.label = "aruco_" + std::to_string(ids[i]);
            double cx = 0.0, cy = 0.0;
            for (const auto& pt : corners[i]) {
                cx += pt.x;
                cy += pt.y;
            }
            r.center = cv::Point2d(cx / 4.0, cy / 4.0);
            r.confidence = 1.0;
            results.push_back(r);
        }
    }

    return results;
}

std::optional<std::vector<DetectionResult>> VisionClient::next_detection() {
    auto frame = camera_->next();
    if (!frame.has_value()) {
        return std::nullopt;
    }
    return detect(frame.value());
}

void VisionClient::set_params(const std::array<int, 3>& lower_hsv,
                              const std::array<int, 3>& upper_hsv, int aruco_dict,
                              double aruco_marker_size_m) {
    color_lower_hsv_ = lower_hsv;
    color_upper_hsv_ = upper_hsv;
    aruco_dict_ = aruco_dict;
    aruco_marker_size_m_ = aruco_marker_size_m;
}

}  // namespace omr_controller
