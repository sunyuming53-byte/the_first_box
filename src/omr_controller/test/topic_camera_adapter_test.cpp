#include "omr_controller/clients/topic_camera_adapter.hpp"

#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>

#include <future>
#include <memory>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;

sensor_msgs::msg::Image make_test_image(rclcpp::Time stamp, uint8_t r, uint8_t g, uint8_t b) {
    cv::Mat img(kImageHeight, kImageWidth, CV_8UC3, cv::Scalar(b, g, r));
    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = stamp;
    cv_img.encoding = "bgr8";
    cv_img.image = img;
    auto msg = *cv_img.toImageMsg();
    return msg;
}

sensor_msgs::msg::CameraInfo make_test_camera_info(rclcpp::Time stamp) {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.height = kImageHeight;
    info.width = kImageWidth;
    info.k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
    info.d = {0.1, -0.05, 0.0, 0.0, 0.0};
    return info;
}

}  // namespace

class TopicCameraAdapterTest : public ::testing::Test {
protected:
    void SetUp() override { node_ = std::make_shared<rclcpp::Node>("topic_camera_test"); }

    void TearDown() override {
        adapter_.reset();
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<omr_controller::TopicCameraAdapter> adapter_;
};

TEST_F(TopicCameraAdapterTest, NextReturnsFrameWhenPublished) {
    adapter_ = std::make_unique<omr_controller::TopicCameraAdapter>(node_);
    auto pub = node_->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw",
                                                                rclcpp::QoS(1).reliable());

    rclcpp::spin_some(node_);

    auto img_msg = make_test_image(node_->now(), 255, 0, 0);
    pub->publish(img_msg);
    rclcpp::spin_some(node_);

    auto frame = adapter_->next();
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->rows, kImageHeight);
    EXPECT_EQ(frame->cols, kImageWidth);
    EXPECT_EQ(frame->channels(), 3);
}

TEST_F(TopicCameraAdapterTest, NextBlocksUntilFrameAvailable) {
    adapter_ = std::make_unique<omr_controller::TopicCameraAdapter>(node_);
    auto pub = node_->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw",
                                                                rclcpp::QoS(1).reliable());
    rclcpp::spin_some(node_);

    auto future = std::async(std::launch::async, [this]() { return adapter_->next(); });

    auto status = future.wait_for(std::chrono::milliseconds(50));
    EXPECT_EQ(status, std::future_status::timeout)
        << "next() should block when no frame is available";

    auto img_msg = make_test_image(node_->now(), 0, 255, 0);
    pub->publish(img_msg);
    rclcpp::spin_some(node_);

    auto frame = future.get();
    ASSERT_TRUE(frame.has_value());
}

TEST_F(TopicCameraAdapterTest, SubsequentNextBlocksAfterFirstConsumed) {
    adapter_ = std::make_unique<omr_controller::TopicCameraAdapter>(node_);
    auto pub = node_->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw",
                                                                rclcpp::QoS(1).reliable());
    rclcpp::spin_some(node_);

    auto img_msg = make_test_image(node_->now(), 0, 0, 255);
    pub->publish(img_msg);
    rclcpp::spin_some(node_);

    auto frame1 = adapter_->next();
    ASSERT_TRUE(frame1.has_value());

    auto future = std::async(std::launch::async, [this]() { return adapter_->next(); });
    auto status = future.wait_for(std::chrono::milliseconds(50));
    EXPECT_EQ(status, std::future_status::timeout)
        << "next() should block after consuming the first frame";

    auto img_msg2 = make_test_image(node_->now(), 128, 128, 128);
    pub->publish(img_msg2);
    rclcpp::spin_some(node_);

    auto frame2 = future.get();
    ASSERT_TRUE(frame2.has_value());
}

TEST_F(TopicCameraAdapterTest, DepthIntrinsicsDefaultBeforeInfoReceived) {
    adapter_ = std::make_unique<omr_controller::TopicCameraAdapter>(node_);
    auto intr = adapter_->depth_intrinsics();

    EXPECT_EQ(intr.K.at<double>(0, 0), 1.0);
    EXPECT_EQ(intr.K.at<double>(1, 1), 1.0);
    EXPECT_EQ(intr.K.at<double>(0, 2), 0.0);
    EXPECT_EQ(intr.K.at<double>(1, 2), 0.0);
}

TEST_F(TopicCameraAdapterTest, DepthIntrinsicsUpdatedFromCameraInfo) {
    adapter_ = std::make_unique<omr_controller::TopicCameraAdapter>(node_);
    auto pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info",
                                                                     rclcpp::QoS(1).reliable());
    rclcpp::spin_some(node_);

    auto info = make_test_camera_info(node_->now());
    pub->publish(info);
    rclcpp::spin_some(node_);

    auto intr = adapter_->depth_intrinsics();
    EXPECT_NEAR(intr.K.at<double>(0, 0), 500.0, 1e-6);
    EXPECT_NEAR(intr.K.at<double>(0, 2), 320.0, 1e-6);
    EXPECT_NEAR(intr.dist_coeff.at<double>(0), 0.1, 1e-6);
}

TEST_F(TopicCameraAdapterTest, CustomTopicNamesWork) {
    adapter_ = std::make_unique<omr_controller::TopicCameraAdapter>(node_, "/custom/image",
                                                                    "/custom/info");
    auto img_pub = node_->create_publisher<sensor_msgs::msg::Image>("/custom/image",
                                                                    rclcpp::QoS(1).reliable());
    auto info_pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/custom/info", rclcpp::QoS(1).reliable());
    rclcpp::spin_some(node_);

    auto info = make_test_camera_info(node_->now());
    info_pub->publish(info);
    rclcpp::spin_some(node_);

    auto intr = adapter_->depth_intrinsics();
    EXPECT_NEAR(intr.K.at<double>(0, 0), 500.0, 1e-6);

    auto img_msg = make_test_image(node_->now(), 128, 64, 32);
    img_pub->publish(img_msg);
    rclcpp::spin_some(node_);

    auto frame = adapter_->next();
    ASSERT_TRUE(frame.has_value());
}
