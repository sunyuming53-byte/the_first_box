#include "omr_vision/calibration/format_polyfill.hpp"

#include <filesystem>
#include <iostream>
#include <string_view>

#include <omr_controller/calib/collector.hpp>
#include <omr_controller/calib/hand_eye.hpp>
#include <omr_controller/calib/pose_proc.hpp>
#include <omr_controller/clients/vision_client.hpp>
#include <omr_vision/calibration/camera_calib.hpp>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char* argv[]) {  // NOLINT(bugprone-exception-escape)
    omr_controller::calib::CalibDataConfig cfg;
    std::string mode_str;

    // Parse CLI arguments
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    for (int i = 1; i < argc; i += 2) {
        std::string_view arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            cfg.arm_ip = argv[i + 1];
        } else if (arg == "--output" && i + 1 < argc) {
            cfg.output_dir = argv[i + 1];
        } else if (arg == "--count" && i + 1 < argc) {
            cfg.total_images = std::stoi(argv[i + 1]);
        } else if (arg == "--board-w" && i + 1 < argc) {
            cfg.board_size.width = std::stoi(argv[i + 1]);
        } else if (arg == "--board-h" && i + 1 < argc) {
            cfg.board_size.height = std::stoi(argv[i + 1]);
        } else if (arg == "--square-size" && i + 1 < argc) {
            cfg.square_size_m = std::stof(argv[i + 1]);
        } else if (arg == "--mode" && i + 1 < argc) {
            mode_str = argv[i + 1];
        }
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    if (mode_str.empty()) {
        std::cerr << "Error: --mode <in_hand|to_hand> is required\n";
        return 1;
    }

    omr_controller::calib::HandEyeMode mode = omr_controller::calib::HandEyeMode::EyeInHand;
    if (mode_str == "in_hand") {
        mode = omr_controller::calib::HandEyeMode::EyeInHand;
    } else if (mode_str == "to_hand") {
        mode = omr_controller::calib::HandEyeMode::EyeToHand;
    } else {
        std::cerr << std::format("Error: unknown mode '{}' (expected in_hand or to_hand)\n",
                                 mode_str);
        return 1;
    }

    // ── Stage 1: Data collection ────────────────────────────────────────
    std::cout << std::format("Stage {}: Starting data collection...\n", 1);

    omr_controller::calib::CalibDataCollector collector(
        cfg,
        std::make_unique<omr_controller::CameraStreamAdapter>(omr_vision::camera::CameraConfig{}));
    auto session_result = collector.run();

    if (!session_result) {
        std::cerr << std::format("Stage 1 failed: {}\n", session_result.error());
        return 1;
    }

    auto& session = *session_result;
    std::cout << std::format("Stage {}: Done. {} images saved to {}\n", 1, cfg.total_images,
                             session.dir.string());

    // ── Stage 2: Camera intrinsic calibration ────────────────────────────
    std::cout << std::format("Stage {}: Running camera intrinsic calibration...\n", 2);

    std::vector<cv::Mat> images;
    for (const auto& entry : std::filesystem::directory_iterator(session.dir)) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".JPG") {
            cv::Mat img = cv::imread(entry.path().string(), cv::IMREAD_GRAYSCALE);
            if (!img.empty()) {
                images.push_back(std::move(img));
            }
        }
    }

    if (images.empty()) {
        std::cerr << std::format("Stage 2 failed: no images found in {}\n", session.dir.string());
        return 1;
    }

    omr_vision::calibration::CameraCalibInput cam_input;
    cam_input.images = std::move(images);
    cam_input.board_size = cfg.board_size;
    cam_input.square_size_m = cfg.square_size_m;

    omr_vision::calibration::CameraCalibrator calibrator(cam_input);
    auto cam_result = calibrator.compute();

    if (!cam_result) {
        std::cerr << std::format("Stage 2 failed: {}\n", cam_result.error());
        return 1;
    }

    std::cout << std::format("Stage {}: Done. Reprojection error: {:.4F} px, images used: {}\n", 2,
                             cam_result->reproj_error, cam_result->images_used);

    // ── Stage 3: Pose processing ─────────────────────────────────────────
    std::cout << std::format("Stage {}: Computing relative arm motions...\n", 3);

    omr_controller::calib::PoseProcessor pose_proc(mode);
    auto pose_result = pose_proc.process(session.arm_poses);

    if (!pose_result) {
        std::cerr << std::format("Stage 3 failed: {}\n", pose_result.error());
        return 1;
    }

    std::cout << std::format("Stage {}: Done. {} motion pairs computed.\n", 3,
                             pose_result->R_motions.size());

    // ── Stage 4: Hand-eye solve ──────────────────────────────────────────
    std::cout << std::format("Stage {}: Solving hand-eye calibration...\n", 4);

    omr_controller::calib::HandEyeSolver solver(mode);
    auto he_result = solver.solve(pose_result->R_motions, pose_result->t_motions, cam_result->rvecs,
                                  cam_result->tvecs);

    if (!he_result) {
        std::cerr << std::format("Stage 4 failed: {}\n", he_result.error());
        return 1;
    }

    he_result->save_yaml(cfg.output_dir / "calibration_result.yaml");

    std::cout << std::format("Stage {}: Done. Method: {}\n", 4, he_result->method);
    std::cout << "\nCalibration result:\n";
    std::cout << "Rotation matrix R:\n" << he_result->R << "\n";
    std::cout << "Translation vector t:\n" << he_result->t << "\n";
    std::cout << std::format("Reprojection error: {:.4F} px\n", he_result->reproj_error);
    std::cout << std::format("\nResult saved to {}/calibration_result.yaml\n",
                             cfg.output_dir.string());

    return 0;
}
