#pragma once

#include <array>
#include <optional>

#include <omr_vision/camera/types.hpp>
#include <opencv2/core.hpp>

namespace omr_vision::detection {

/// PnP backend for coplanar label corners (OpenCV solvePnP flags).
enum class WhiteLabelPnpMethod {
    Ippe,       ///< Plane-specialized (default; best for flat tags).
    Iterative,  ///< Generic LM iteration (previous default).
};

struct WhiteLabelConfig {
    double width_m{0.020};    // short side of label (m)
    double height_m{0.0775};  // long side of label (m)

    // Legacy fixed threshold (also used as one of multi-threshold seeds).
    int binary_threshold{180};

    // Detection / filtering (does not affect PnP solver choice).
    double min_area_px{400.0};
    double aspect_ratio_tol{0.30};    // → aspect in [target*(1-tol), target*(1+tol)]
    double min_rectangularity{0.60};  // contourArea / minAreaRect area
    double min_content_score{0.45};   // warp-based white-label score in [0,1]
    bool use_clahe{true};
    bool use_otsu{true};
    bool use_multi_threshold{true};
    /// If >0, reject candidates whose long-side pixel length is outside the
    /// range implied by height_m at z_min_m..z_max_m (needs focal_length_px).
    double focal_length_px{0.0};
    double z_min_m{0.12};  // working distance lower bound for scale prior
    double z_max_m{2.00};
    int warp_short_px{40};
    int warp_long_px{155};

    // PnP-only (unchanged by detection improvements).
    double max_reproj_error_px{5.0};
    WhiteLabelPnpMethod pnp_method{WhiteLabelPnpMethod::Ippe};
};

struct WhiteLabelDetection {
    /// Image corners in order: top-left, top-right, bottom-right, bottom-left.
    std::array<cv::Point2f, 4> corners_img{};
    cv::RotatedRect rect{};
    double confidence{0.0};
    double content_score{0.0};
};

struct WhiteLabelPose {
    cv::Vec3d rvec{};
    cv::Vec3d tvec{};     // meters, camera frame
    cv::Mat T_cam_label;  // 4x4 CV_64F
    double reproj_error_px{0.0};
    WhiteLabelDetection detection{};
};

/// Require the same label (by center proximity) for N consecutive frames before accepting.
class WhiteLabelTemporalConfirm {
public:
    explicit WhiteLabelTemporalConfirm(int required_frames = 3, double max_center_shift_px = 50.0);

    /// Feed one frame's raw detection. Returns confirmed detection only after a streak.
    [[nodiscard]] auto update(const std::optional<WhiteLabelDetection>& detection)
        -> std::optional<WhiteLabelDetection>;

    void reset();

    [[nodiscard]] int streak() const { return streak_; }
    [[nodiscard]] int requiredFrames() const { return required_frames_; }

private:
    int required_frames_;
    double max_center_shift_px_;
    int streak_{0};
    cv::Point2d last_center_{0.0, 0.0};
};

/// Detect a white rectangular label in a BGR image (no pose).
/// If debug_mask != nullptr, writes the combined binary mask used for contours.
[[nodiscard]] auto detectWhiteLabel(const cv::Mat& bgr, const WhiteLabelConfig& cfg = {},
                                    cv::Mat* debug_mask = nullptr)
    -> std::optional<WhiteLabelDetection>;

/// Run PnP on an already-detected label (detection-stage output).
[[nodiscard]] auto estimateWhiteLabelPoseFromDetection(
    const WhiteLabelDetection& detection, const camera::CameraIntrinsics& color_intrinsics,
    const WhiteLabelConfig& cfg = {}) -> std::optional<WhiteLabelPose>;

/// Detect the label and estimate its pose in the camera frame via PnP.
[[nodiscard]] auto estimateWhiteLabelPose(const cv::Mat& bgr,
                                          const camera::CameraIntrinsics& color_intrinsics,
                                          const WhiteLabelConfig& cfg = {})
    -> std::optional<WhiteLabelPose>;

}  // namespace omr_vision::detection
