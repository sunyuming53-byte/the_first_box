#include "omr_vision/detection/white_label.hpp"

#include <cmath>
#include <gtest/gtest.h>

#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace {

omr_vision::camera::CameraIntrinsics makeIntrinsics(int width, int height) {
    omr_vision::camera::CameraIntrinsics intr;
    const double fx = 600.0;
    const double fy = 600.0;
    const double cx = width * 0.5;
    const double cy = height * 0.5;
    intr.K = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    intr.dist_coeff = cv::Mat::zeros(1, 5, CV_64F);
    return intr;
}

std::vector<cv::Point3f> labelObjectPoints(double width_m, double height_m) {
    const float hw = static_cast<float>(width_m * 0.5);
    const float hh = static_cast<float>(height_m * 0.5);
    return {{-hw, hh, 0.0F}, {hw, hh, 0.0F}, {hw, -hh, 0.0F}, {-hw, -hh, 0.0F}};
}

cv::Mat renderSyntheticLabel(const omr_vision::camera::CameraIntrinsics& intr, int width,
                             int height, const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                             double width_m, double height_m) {
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    const auto obj = labelObjectPoints(width_m, height_m);
    std::vector<cv::Point2f> img_pts;
    cv::projectPoints(obj, rvec, tvec, intr.K, intr.dist_coeff, img_pts);

    std::vector<cv::Point> poly;
    poly.reserve(4);
    for (const auto& p : img_pts) {
        poly.emplace_back(cv::Point(cvRound(p.x), cvRound(p.y)));
    }
    cv::fillConvexPoly(image, poly, cv::Scalar(255, 255, 255));
    return image;
}

}  // namespace

TEST(WhiteLabelTest, DetectsSyntheticRectangle) {
    constexpr int kWidth = 640;
    constexpr int kHeight = 480;
    auto intr = makeIntrinsics(kWidth, kHeight);

    omr_vision::detection::WhiteLabelConfig cfg;
    cfg.width_m = 0.020;
    cfg.height_m = 0.0775;
    cfg.binary_threshold = 127;
    cfg.min_area_px = 200.0;
    cfg.min_content_score = 0.35;
    cfg.max_reproj_error_px = 3.0;

    const cv::Vec3d rvec(0.1, -0.15, 0.05);
    const cv::Vec3d tvec(0.02, -0.01, 0.45);
    cv::Mat image =
        renderSyntheticLabel(intr, kWidth, kHeight, rvec, tvec, cfg.width_m, cfg.height_m);

    auto detection = omr_vision::detection::detectWhiteLabel(image, cfg);
    ASSERT_TRUE(detection.has_value());
    EXPECT_GT(detection->confidence, 0.5);
}

TEST(WhiteLabelTest, EstimatesPoseNearGroundTruth) {
    constexpr int kWidth = 640;
    constexpr int kHeight = 480;
    auto intr = makeIntrinsics(kWidth, kHeight);

    omr_vision::detection::WhiteLabelConfig cfg;
    cfg.width_m = 0.020;
    cfg.height_m = 0.0775;
    cfg.binary_threshold = 127;
    cfg.min_area_px = 200.0;
    cfg.min_content_score = 0.35;
    cfg.max_reproj_error_px = 3.0;
    cfg.pnp_method = omr_vision::detection::WhiteLabelPnpMethod::Ippe;

    const cv::Vec3d rvec_gt(0.08, -0.12, 0.03);
    const cv::Vec3d tvec_gt(0.01, 0.02, 0.40);
    cv::Mat image =
        renderSyntheticLabel(intr, kWidth, kHeight, rvec_gt, tvec_gt, cfg.width_m, cfg.height_m);

    auto pose = omr_vision::detection::estimateWhiteLabelPose(image, intr, cfg);
    ASSERT_TRUE(pose.has_value());
    EXPECT_LT(pose->reproj_error_px, 2.0);

    EXPECT_NEAR(pose->tvec[0], tvec_gt[0], 0.005);
    EXPECT_NEAR(pose->tvec[1], tvec_gt[1], 0.005);
    EXPECT_NEAR(pose->tvec[2], tvec_gt[2], 0.01);

    ASSERT_FALSE(pose->T_cam_label.empty());
    EXPECT_EQ(pose->T_cam_label.rows, 4);
    EXPECT_EQ(pose->T_cam_label.cols, 4);
    EXPECT_DOUBLE_EQ(pose->T_cam_label.at<double>(3, 3), 1.0);
}

TEST(WhiteLabelTest, IterativeMethodAlsoEstimatesPose) {
    constexpr int kWidth = 640;
    constexpr int kHeight = 480;
    auto intr = makeIntrinsics(kWidth, kHeight);

    omr_vision::detection::WhiteLabelConfig cfg;
    cfg.width_m = 0.020;
    cfg.height_m = 0.0775;
    cfg.binary_threshold = 127;
    cfg.min_area_px = 200.0;
    cfg.min_content_score = 0.35;
    cfg.max_reproj_error_px = 3.0;
    cfg.pnp_method = omr_vision::detection::WhiteLabelPnpMethod::Iterative;

    const cv::Vec3d rvec_gt(0.05, -0.08, 0.02);
    const cv::Vec3d tvec_gt(0.0, 0.0, 0.42);
    cv::Mat image =
        renderSyntheticLabel(intr, kWidth, kHeight, rvec_gt, tvec_gt, cfg.width_m, cfg.height_m);

    auto pose = omr_vision::detection::estimateWhiteLabelPose(image, intr, cfg);
    ASSERT_TRUE(pose.has_value());
    EXPECT_LT(pose->reproj_error_px, 2.0);
    EXPECT_NEAR(pose->tvec[2], tvec_gt[2], 0.03);
}

TEST(WhiteLabelTest, EmptyImageReturnsNullopt) {
    cv::Mat empty;
    auto detection = omr_vision::detection::detectWhiteLabel(empty);
    EXPECT_FALSE(detection.has_value());
}

TEST(WhiteLabelTest, NoWhiteBlobReturnsNullopt) {
    cv::Mat black = cv::Mat::zeros(480, 640, CV_8UC3);
    auto detection = omr_vision::detection::detectWhiteLabel(black);
    EXPECT_FALSE(detection.has_value());
}

TEST(WhiteLabelTemporalConfirmTest, RequiresConsecutiveNearbyFrames) {
    omr_vision::detection::WhiteLabelTemporalConfirm confirm(3, 50.0);

    omr_vision::detection::WhiteLabelDetection det;
    det.corners_img = {cv::Point2f(100, 100), cv::Point2f(120, 100), cv::Point2f(120, 180),
                       cv::Point2f(100, 180)};

    EXPECT_FALSE(confirm.update(det).has_value());
    EXPECT_EQ(confirm.streak(), 1);
    EXPECT_FALSE(confirm.update(det).has_value());
    EXPECT_EQ(confirm.streak(), 2);
    auto out = confirm.update(det);
    ASSERT_TRUE(out.has_value());
    EXPECT_GE(confirm.streak(), 3);
}

TEST(WhiteLabelTemporalConfirmTest, JumpResetsStreak) {
    omr_vision::detection::WhiteLabelTemporalConfirm confirm(3, 30.0);

    omr_vision::detection::WhiteLabelDetection a;
    a.corners_img = {cv::Point2f(100, 100), cv::Point2f(120, 100), cv::Point2f(120, 180),
                     cv::Point2f(100, 180)};
    omr_vision::detection::WhiteLabelDetection b = a;
    for (auto& p : b.corners_img) {
        p.x += 200.0F;
    }

    EXPECT_FALSE(confirm.update(a).has_value());
    EXPECT_FALSE(confirm.update(a).has_value());
    EXPECT_FALSE(confirm.update(b).has_value());  // jump → streak back to 1
    EXPECT_EQ(confirm.streak(), 1);
}

TEST(WhiteLabelTemporalConfirmTest, MissClearsStreak) {
    omr_vision::detection::WhiteLabelTemporalConfirm confirm(3, 50.0);
    omr_vision::detection::WhiteLabelDetection det;
    det.corners_img = {cv::Point2f(10, 10), cv::Point2f(20, 10), cv::Point2f(20, 40),
                       cv::Point2f(10, 40)};

    EXPECT_FALSE(confirm.update(det).has_value());
    EXPECT_FALSE(confirm.update(std::nullopt).has_value());
    EXPECT_EQ(confirm.streak(), 0);
}
