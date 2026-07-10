#include "omr_controller/clients/topic_camera_adapter.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>

namespace omr_controller {

TopicCameraAdapter::TopicCameraAdapter(rclcpp::Node::SharedPtr node, const std::string& image_topic,
                                       const std::string& camera_info_topic)
    : node_(node) {
    auto img_qos = rclcpp::QoS(1).best_effort();
    sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        image_topic, img_qos, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard lock(mutex_);
            try {
                auto cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
                latest_frame_ = cv_ptr->image.clone();
            } catch (const cv_bridge::Exception&) {
                return;
            }
            cv_.notify_one();
        });

    info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::QoS(1).reliable(),
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            std::lock_guard lock(info_mutex_);
            intrinsics_.K = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3],
                             msg->k[4], msg->k[5], msg->k[6], msg->k[7], msg->k[8]);
            if (!msg->d.empty()) {
                intrinsics_.dist_coeff = cv::Mat(msg->d, true).reshape(1, 1);
            } else {
                intrinsics_.dist_coeff = cv::Mat::zeros(1, 5, CV_64F);
            }
            intrinsics_set_ = true;
        });
}

std::optional<cv::Mat> TopicCameraAdapter::next() {
    std::unique_lock lock(mutex_);
    cv_.wait(lock, [this] { return latest_frame_.has_value(); });
    auto frame = std::move(latest_frame_);
    latest_frame_.reset();
    return frame;
}

omr_vision::camera::CameraIntrinsics TopicCameraAdapter::depth_intrinsics() const {
    std::lock_guard lock(info_mutex_);
    if (!intrinsics_set_) {
        omr_vision::camera::CameraIntrinsics intr;
        intr.K = cv::Mat::eye(3, 3, CV_64F);
        intr.dist_coeff = cv::Mat::zeros(1, 5, CV_64F);
        return intr;
    }
    return intrinsics_;
}

}  // namespace omr_controller
