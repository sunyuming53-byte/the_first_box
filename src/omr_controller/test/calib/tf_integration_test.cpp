#include <cmath>
#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <string>

#include "omr_controller/calib/hand_eye.hpp"
#include "omr_controller/calib/transform.hpp"
#include "synthetic_poses.h"
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
// NOLINTBEGIN(readability-convert-member-functions-to-static,cppcoreguidelines-pro-bounds-constant-array-index)

using namespace omr_controller::calib;
using namespace omr_controller::calib::test;

namespace {

// ──────────────────────────────────────────────────────────────
// fixture — temp file lifecycle
// ──────────────────────────────────────────────────────────────

class TransformIntegrationTest : public ::testing::Test {
protected:
    static constexpr auto kTempFile = "/tmp/test_calib_tf.yaml";
    static constexpr auto kEmptyFile = "/tmp/test_empty_tf.yaml";
    static constexpr auto kCorruptedFile = "/tmp/test_corrupt_tf.yaml";
    static constexpr auto kEyeToHandFile = "/tmp/test_eye2hand_tf.yaml";

    void TearDown() override {
        std::filesystem::remove(kTempFile);
        std::filesystem::remove(kEmptyFile);
        std::filesystem::remove(kCorruptedFile);
        std::filesystem::remove(kEyeToHandFile);
    }

    [[nodiscard]] HandEyeResult make_known_result(HandEyeMode mode) const {
        HandEyeResult r;
        r.R = cv::Mat::eye(3, 3, CV_64F);
        r.t = (cv::Mat_<double>(3, 1) << 0.1, 0.2, 0.3);
        r.mode = mode;
        r.reproj_error = 0.01;
        r.method = "Tsai";
        r.condition_number = 42.0;
        return r;
    }
};

// ── Test 1 — Load valid ──
TEST_F(TransformIntegrationTest, LoadValidYAML) {
    auto result = make_known_result(HandEyeMode::EyeInHand);
    result.save_yaml(kTempFile);

    auto loaded = HandEyeTransform::load(kTempFile);
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(loaded->mode(), HandEyeMode::EyeInHand);
    EXPECT_FALSE(loaded->matrix().empty());
}

// ── Test 2 — Load nonexistent ──
TEST_F(TransformIntegrationTest, LoadNonexistentFile) {
    auto loaded = HandEyeTransform::load("/tmp/no_such_file_abc123.yaml");
    EXPECT_FALSE(loaded.has_value());
}

// ── Test 3 — Empty YAML ──
TEST_F(TransformIntegrationTest, LoadEmptyYAML) {
    {
        std::ofstream ofs(kEmptyFile);
        ofs << "%YAML:1.0\n---\n";
        ofs.close();
    }
    auto loaded = HandEyeTransform::load(kEmptyFile);
    EXPECT_FALSE(loaded.has_value());
}

// ── Test 4 — Corrupted YAML ──
TEST_F(TransformIntegrationTest, LoadCorruptedYAML) {
    {
        std::ofstream ofs(kCorruptedFile);
        ofs << "%YAML:1.0\n---\njunk: 42\n";
        ofs.close();
    }
    auto loaded = HandEyeTransform::load(kCorruptedFile);
    EXPECT_FALSE(loaded.has_value());
}

// ── Test 5 — PointCameraToBase EyeInHand ──
TEST_F(TransformIntegrationTest, PointCameraToBase_EyeInHand) {
    auto result = make_known_result(HandEyeMode::EyeInHand);
    result.save_yaml(kTempFile);
    auto loaded = HandEyeTransform::load(kTempFile);
    ASSERT_TRUE(loaded.has_value());

    const std::array<double, 6> arm_pose = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};

    auto pt = loaded->point_camera_to_base(1.0, 2.0, 3.0, arm_pose);

    EXPECT_NEAR(pt[0], 2.1, 1e-6);
    EXPECT_NEAR(pt[1], 4.2, 1e-6);
    EXPECT_NEAR(pt[2], 6.3, 1e-6);
}

// ── Test 6 — PoseCameraToBase EyeInHand ──
TEST_F(TransformIntegrationTest, PoseCameraToBase_EyeInHand) {
    auto result = make_known_result(HandEyeMode::EyeInHand);
    result.save_yaml(kTempFile);
    auto loaded = HandEyeTransform::load(kTempFile);
    ASSERT_TRUE(loaded.has_value());

    const std::array<double, 6> arm_pose = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};

    auto pose = loaded->pose_camera_to_base(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, arm_pose);

    for (int i = 0; i < 6; ++i) {
        EXPECT_TRUE(std::isfinite(pose[i])) << "pose[" << i << "] = " << pose[i];
    }

    EXPECT_NEAR(pose[0], 2.1, 1e-6);
    EXPECT_NEAR(pose[1], 4.2, 1e-6);
    EXPECT_NEAR(pose[2], 6.3, 1e-6);
    EXPECT_NEAR(pose[3], 0.0, 1e-6);
    EXPECT_NEAR(pose[4], 0.0, 1e-6);
    EXPECT_NEAR(pose[5], 0.0, 1e-6);
}

// ── Test 7 — EyeToHand differs from EyeInHand ──
TEST_F(TransformIntegrationTest, EyeToHandDiffersFromEyeInHand) {
    auto result_eih = make_known_result(HandEyeMode::EyeInHand);
    auto result_eth = make_known_result(HandEyeMode::EyeToHand);
    result_eih.save_yaml(kTempFile);
    result_eth.save_yaml(kEyeToHandFile);

    auto eih = HandEyeTransform::load(kTempFile);
    auto eth = HandEyeTransform::load(kEyeToHandFile);
    ASSERT_TRUE(eih.has_value());
    ASSERT_TRUE(eth.has_value());

    const std::array<double, 6> arm_pose = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};

    auto pt_eih = eih->point_camera_to_base(5.0, 6.0, 7.0, arm_pose);
    auto pt_eth = eth->point_camera_to_base(5.0, 6.0, 7.0);

    double max_diff = 0.0;
    for (int i = 0; i < 3; ++i) {
        max_diff = std::max(max_diff, std::abs(pt_eih[i] - pt_eth[i]));
    }
    EXPECT_GT(max_diff, 1e-6) << "EyeInHand and EyeToHand must differ with non-zero arm_pose";
}

}  // namespace
// NOLINTEND(readability-convert-member-functions-to-static,cppcoreguidelines-pro-bounds-constant-array-index)
