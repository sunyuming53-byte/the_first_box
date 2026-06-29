/// calib_node — ROS2 node wrapping the realman_calibration pipeline.
///
/// Exposes:
///   ~/run         (Trigger)   Run full pipeline from a pre-collected session.
///   ~/calibrate   (Trigger)   Camera intrinsic calibration only.
///   ~/hand_eye    (Trigger)   Hand-eye solve only (requires poses + camera result).
///   ~/stop        (Trigger)   Emergency stop the arm.
///
/// All paths and board parameters are configured via ROS2 params.

#include <realman_calibration/camera_calib.hpp>
#include <realman_calibration/collector.hpp>
#include <realman_calibration/hand_eye.hpp>
#include <realman_calibration/pose_proc.hpp>

#include <realman/arm.hpp>
#include <realman/types.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <filesystem>
#include <format>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <string>

namespace rm::calib {

class CalibNode : public rclcpp::Node {
public:
    explicit CalibNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("calib_node", options)
    {
        // ── ROS2 parameters ────────────────────────────────────────────
        declare_parameter("arm_ip", "192.168.1.18");
        declare_parameter("session_dir", "data/calib_session");
        declare_parameter("mode", "in_hand");
        declare_parameter("board_w", 11);
        declare_parameter("board_h", 8);
        declare_parameter("square_size_m", 0.030);

        // Parse hand-eye mode
        auto mode_str = get_parameter("mode").as_string();
        if (mode_str == "to_hand") {
            mode_ = HandEyeMode::EyeToHand;
        } else {
            mode_ = HandEyeMode::EyeInHand;
        }

        // Build config from params
        cfg_.arm_ip        = get_parameter("arm_ip").as_string();
        cfg_.output_dir    = get_parameter("session_dir").as_string();
        cfg_.board_size    = cv::Size(get_parameter("board_w").as_int(),
                                       get_parameter("board_h").as_int());
        cfg_.square_size_m = static_cast<float>(get_parameter("square_size_m").as_double());

        RCLCPP_INFO(get_logger(),
            "CalibNode ready — arm=%s mode=%s board=%dx%d sq=%.3fm session=%s",
            cfg_.arm_ip.c_str(), mode_str.c_str(),
            cfg_.board_size.width, cfg_.board_size.height,
            cfg_.square_size_m, cfg_.output_dir.string().c_str());

        setupServices();
    }

private:
    void setupServices() {
        using Trigger = std_srvs::srv::Trigger;

        run_svc_ = create_service<Trigger>(
            "~/run",
            [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
                res->success = true;
                std::string msg;
                {
                    std::lock_guard lock{mtx_};
                    msg = runPipeline();
                }
                res->message = msg;
            });

        cam_calib_svc_ = create_service<Trigger>(
            "~/calibrate",
            [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
                res->success = true;
                std::string msg;
                {
                    std::lock_guard lock{mtx_};
                    msg = runCameraCalib();
                }
                res->message = msg;
            });

        hand_eye_svc_ = create_service<Trigger>(
            "~/hand_eye",
            [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
                res->success = true;
                std::string msg;
                {
                    std::lock_guard lock{mtx_};
                    msg = runHandEye();
                }
                res->message = msg;
            });

        stop_svc_ = create_service<Trigger>(
            "~/stop",
            [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
                std::lock_guard lock{mtx_};
                try {
                    arm_.stop();
                    res->success = true;
                    res->message = "arm stopped";
                } catch (const rm::ArmError& e) {
                    res->success = false;
                    res->message = std::format("stop failed: {}", e.what());
                }
            });
    }

    // ── Pipeline stages ────────────────────────────────────────────────

    /// Load images from session directory and run camera calibration.
    auto loadSessionImages() -> std::vector<cv::Mat> {
        std::vector<cv::Mat> images;
        for (const auto& entry : std::filesystem::directory_iterator(cfg_.output_dir)) {
            auto ext = entry.path().extension();
            if (ext == ".jpg" || ext == ".JPG") {
                cv::Mat img = cv::imread(entry.path().string(), cv::IMREAD_GRAYSCALE);
                if (!img.empty()) images.push_back(std::move(img));
            }
        }
        return images;
    }

    /// Stage 2: Camera intrinsic calibration only.
    auto runCameraCalib() -> std::string {
        auto images = loadSessionImages();
        if (images.empty())
            return std::format("no images in {}", cfg_.output_dir.string());

        CameraCalibInput input;
        input.images       = std::move(images);
        input.board_size   = cfg_.board_size;
        input.square_size_m = cfg_.square_size_m;

        CameraCalibrator calibrator(input);
        auto result = calibrator.compute();
        if (!result)
            return std::format("camera calibration failed: {}", result.error());

        // Persist for hand-eye stage
        last_cam_rvecs_ = result->rvecs;
        last_cam_tvecs_ = result->tvecs;

        return std::format("camera OK — reproj {:.4f} px, {} images",
                           result->reproj_error, result->images_used);
    }

    /// Stage 3+4: Pose processing + hand-eye solve (requires camera result first).
    auto runHandEye() -> std::string {
        if (last_cam_rvecs_.empty())
            return "run ~/calibrate first — no camera result cached";
        if (last_arm_poses_.empty())
            return "no arm poses cached — run ~/run (full pipeline) first";

        PoseProcessor pose_proc(mode_);
        auto pose_result = pose_proc.process(last_arm_poses_);
        if (!pose_result)
            return std::format("pose processing failed: {}", pose_result.error());

        HandEyeSolver solver(mode_);
        auto he = solver.solve(pose_result->R_motions, pose_result->t_motions,
                               last_cam_rvecs_, last_cam_tvecs_);
        if (!he)
            return std::format("hand-eye failed: {}", he.error());

        auto result_path = cfg_.output_dir / "calibration_result.yaml";
        he->save_yaml(result_path);

        return std::format("hand-eye OK — method={} reproj={:.4f} px → {}",
                           he->method, he->reproj_error, result_path.string());
    }

    /// Full pipeline: collect → camera calib → poses → hand-eye.
    auto runPipeline() -> std::string {
        // Stage 1: Collect
        RCLCPP_INFO(get_logger(), "Stage 1: collecting calibration data...");
        CalibDataCollector collector(cfg_);
        auto session = collector.run();
        if (!session)
            return std::format("collect failed: {}", session.error());

        // Store arm poses for hand-eye stage
        last_arm_poses_ = session->arm_poses;

        // Stage 2: Camera calibration
        RCLCPP_INFO(get_logger(), "Stage 2: camera intrinsic calibration...");
        auto cam_msg = runCameraCalib();
        if (cam_msg.find("failed") != std::string::npos) return cam_msg;

        // Stage 3: Pose processing
        RCLCPP_INFO(get_logger(), "Stage 3: processing arm poses...");

        // Stage 4: Hand-eye solve
        RCLCPP_INFO(get_logger(), "Stage 4: solving hand-eye...");
        auto he_msg = runHandEye();

        return std::format("pipeline done — {} | {}", cam_msg, he_msg);
    }

    // ── State ──────────────────────────────────────────────────────────
    rm::Arm             arm_{rm::ArmConfig{.arm_ip = "192.168.1.18"}};
    CalibDataConfig     cfg_;
    HandEyeMode         mode_{HandEyeMode::EyeInHand};
    std::mutex          mtx_;

    // Cached results for staged execution
    std::vector<cv::Mat>              last_cam_rvecs_;
    std::vector<cv::Mat>              last_cam_tvecs_;
    std::vector<std::array<double, 6>> last_arm_poses_;

    // ROS2 services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_svc_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cam_calib_svc_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hand_eye_svc_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_svc_;
};

}  // namespace rm::calib

// ── Entry point ──────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm::calib::CalibNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
