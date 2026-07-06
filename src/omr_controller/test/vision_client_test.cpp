#include "omr_controller/clients/vision_client.hpp"

#include <gtest/gtest.h>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

using namespace omr_controller;

namespace {

class FakeCamera : public ICamera {
public:
    explicit FakeCamera(cv::Mat image) : image_(std::move(image)) {}

    std::optional<cv::Mat> next() override {
        if (returned_) return std::nullopt;
        returned_ = true;
        return image_.clone();
    }

    omr_vision::camera::CameraIntrinsics depth_intrinsics() const override {
        omr_vision::camera::CameraIntrinsics intr;
        intr.K = (cv::Mat_<double>(3, 3) << 500.0, 0.0, 100.0,
                                             0.0, 500.0, 100.0,
                                             0.0, 0.0, 1.0);
        intr.dist_coeff = cv::Mat::zeros(1, 5, CV_64F);
        return intr;
    }

private:
    cv::Mat image_;
    bool returned_{false};
};

cv::Mat makeRedBlobImage(int width, int height, cv::Point center, int radius) {
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::circle(img, center, radius, cv::Scalar(0, 0, 255), cv::FILLED);
    return img;
}

cv::Mat makeArucoImage(int side, int marker_id, cv::aruco::PredefinedDictionaryType dict_type) {
    auto dict = cv::aruco::getPredefinedDictionary(dict_type);
    cv::Mat gray;
    cv::aruco::drawMarker(dict, marker_id, side, gray, 1);
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}

}  // namespace

class VisionClientTest : public ::testing::Test {
protected:
    // Helper: build VisionClient with a FakeCamera holding the given image.
    auto makeClient(cv::Mat image) -> VisionClient {
        return VisionClient(std::make_unique<FakeCamera>(std::move(image)));
    }
};

TEST_F(VisionClientTest, DetectRedBlobFindsColorLabel) {
    constexpr cv::Point kExpectedCenter(100, 100);
    cv::Mat img = makeRedBlobImage(200, 200, kExpectedCenter, 20);
    VisionClient client = makeClient(img);

    auto results = client.detect(img);

    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].label, "color");
    EXPECT_NEAR(results[0].center.x, kExpectedCenter.x, 5.0);
    EXPECT_NEAR(results[0].center.y, kExpectedCenter.y, 5.0);
}

TEST_F(VisionClientTest, DetectEmptyImageReturnsEmpty) {
    cv::Mat black(200, 200, CV_8UC3, cv::Scalar(0, 0, 0));
    VisionClient client = makeClient(black);

    auto results = client.detect(black);

    EXPECT_TRUE(results.empty());
}

TEST_F(VisionClientTest, DetectArucoMarkerFindsCorrectId) {
    cv::Mat img = makeArucoImage(200, 5, cv::aruco::DICT_6X6_250);
    VisionClient client = makeClient(img);

    auto results = client.detect(img);

    // Should find at least one aruco result with the correct label.
    bool found = false;
    for (const auto& r : results) {
        if (r.label == "aruco_5") {
            found = true;
            // Center of a 200x200 marker with 1-bit border ≈ image center.
            EXPECT_NEAR(r.center.x, 100.0, 20.0);
            EXPECT_NEAR(r.center.y, 100.0, 20.0);
            break;
        }
    }
    EXPECT_TRUE(found) << "Expected detection with label 'aruco_5'";
}
