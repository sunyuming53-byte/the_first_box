#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "omr_controller/clients/vision_client.hpp"
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace omr_controller {

class TopicCameraAdapter : public ICamera {
public:
    explicit TopicCameraAdapter(rclcpp::Node::SharedPtr node,
                                const std::string& image_topic = "/camera/color/image_raw",
                                const std::string& camera_info_topic = "/camera/color/camera_info");

    std::optional<cv::Mat> next() override;

    omr_vision::camera::CameraIntrinsics depth_intrinsics() const override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::optional<cv::Mat> latest_frame_;

    mutable std::mutex info_mutex_;
    omr_vision::camera::CameraIntrinsics intrinsics_;
    bool intrinsics_set_{false};
};

}  // namespace omr_controller
