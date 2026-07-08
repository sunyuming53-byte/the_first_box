#include "omr_vision/calibration/format_polyfill.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <omr_controller/calib/hand_eye.hpp>
#include <omr_controller/calib/pose_proc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

namespace {

void print_usage() {
    std::cout
        << R"(Usage: compute_hand_eye --rvecs <path> --tvecs <path> --poses <path> --mode <in_hand|to_hand> [--method <method>] [--output <path>]

Compute the hand-eye calibration transform from pre-collected camera and arm data.

Options:
  --rvecs PATH    YAML file with camera rotation vectors (from calibrate_camera output)
  --tvecs PATH    YAML file with camera translation vectors
  --poses PATH    CSV file with arm end-effector poses (tx,ty,tz,rx,ry,rz per row)
  --mode MODE     Hand-eye mode: in_hand (camera on end-effector) or to_hand (camera fixed)
  --method METHOD  Solve method: auto (try all, pick best), tsai, park, horaud, daniilidis [default: auto]
  --output PATH   Output YAML file path [default: calibration_result.yaml]
  --help          Show this help message
)";
}

/// Load a sequence of cv::Mat from a YAML file key (OpenCV FileStorage format).
[[nodiscard]] auto load_mats_from_yaml(const std::string& path, const std::string& key)
    -> std::vector<cv::Mat> {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error(std::format("Cannot open file: {}", path));
    }

    cv::FileNode node = fs[key];
    if (node.isNone()) {
        throw std::runtime_error(std::format("Key '{}' not found in {}", key, path));
    }

    std::vector<cv::Mat> mats;
    if (node.isSeq()) {
        for (const auto& item : node) {
            cv::Mat m;
            item >> m;
            if (!m.empty()) mats.push_back(std::move(m));
        }
    } else {
        cv::Mat m;
        node >> m;
        if (!m.empty()) mats.push_back(std::move(m));
    }

    fs.release();
    return mats;
}

/// Load arm poses from CSV: skip header line, parse 6 doubles per row.
[[nodiscard]] auto load_poses_from_csv(const std::string& path)
    -> std::vector<std::array<double, 6>> {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error(std::format("Cannot open CSV: {}", path));
    }

    std::vector<std::array<double, 6>> poses;
    std::string line;

    // Skip header line
    if (!std::getline(file, line)) {
        throw std::runtime_error(std::format("Empty CSV: {}", path));
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::array<double, 6> pose{};
        std::istringstream ss(line);
        std::string token;
        // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
        for (int j = 0; j < 6 && std::getline(ss, token, ','); ++j) {
            pose[j] = std::stod(token);
        }
        // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
        poses.push_back(pose);
    }

    if (poses.empty()) {
        throw std::runtime_error(std::format("No pose data in CSV: {}", path));
    }

    return poses;
}

/// Parse method string to enum.
[[nodiscard]] auto parse_method(const std::string& s) -> omr_controller::calib::HandEyeMethod {
    if (s == "auto") return omr_controller::calib::HandEyeMethod::Auto;
    if (s == "tsai") return omr_controller::calib::HandEyeMethod::Tsai;
    if (s == "park") return omr_controller::calib::HandEyeMethod::Park;
    if (s == "horaud") return omr_controller::calib::HandEyeMethod::Horaud;
    if (s == "daniilidis") return omr_controller::calib::HandEyeMethod::Daniilidis;
    throw std::runtime_error(std::format("Unknown method: '{}'", s));
}

}  // anonymous namespace

int main(int argc, char* argv[]) {
    std::string rvecs_path;
    std::string tvecs_path;
    std::string poses_path;
    std::string mode_str;
    std::string method_str{"auto"};
    std::string output_path{"calibration_result.yaml"};

    // Parse CLI arguments
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage();
            return 0;
        }
        if (arg == "--rvecs" && i + 1 < argc) {
            rvecs_path = argv[++i];
        } else if (arg == "--tvecs" && i + 1 < argc) {
            tvecs_path = argv[++i];
        } else if (arg == "--poses" && i + 1 < argc) {
            poses_path = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            mode_str = argv[++i];
        } else if (arg == "--method" && i + 1 < argc) {
            method_str = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else {
            std::cerr << std::format("Unknown argument: {}\n\n", arg);
            print_usage();
            return 1;
        }
    }

    // Validate required arguments
    if (rvecs_path.empty() || tvecs_path.empty() || poses_path.empty() || mode_str.empty()) {
        std::cerr << "Error: --rvecs, --tvecs, --poses, and --mode are required\n\n";
        print_usage();
        return 1;
    }

    try {
        // ── Parse mode ──
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

        // ── Load camera data from YAML ──
        auto rvecs = load_mats_from_yaml(rvecs_path, "rvecs");
        auto tvecs = load_mats_from_yaml(tvecs_path, "tvecs");

        if (rvecs.empty() || tvecs.empty()) {
            std::cerr << "Error: empty rvecs or tvecs from YAML files\n";
            return 1;
        }
        std::cout << std::format("Loaded {} camera poses from YAML\n", rvecs.size());

        // ── Load arm poses from CSV ──
        auto poses = load_poses_from_csv(poses_path);
        std::cout << std::format("Loaded {} arm poses from CSV\n", poses.size());

        // ── Stage 3: Pose processing (arm poses → relative motions) ──
        std::cout << std::format("Computing relative arm motions (mode: {})...\n", mode_str);

        omr_controller::calib::PoseProcessor pose_proc(mode);
        auto pose_result = pose_proc.process(poses);

        if (!pose_result) {
            std::cerr << std::format("Pose processing failed: {}\n", pose_result.error());
            return 1;
        }
        std::cout << std::format("Computed {} motion pairs\n", pose_result->R_motions.size());

        // ── Stage 4: Hand-eye solve ──
        auto method = parse_method(method_str);
        std::cout << std::format("Solving hand-eye (method: {})...\n", method_str);

        omr_controller::calib::HandEyeSolver solver(mode);
        auto he_result =
            solver.solve(pose_result->R_motions, pose_result->t_motions, rvecs, tvecs, method);

        if (!he_result) {
            std::cerr << std::format("Hand-eye solve failed: {}\n", he_result.error());
            return 1;
        }

        // ── Save result to YAML ──
        he_result->save_yaml(output_path);

        // ── Print summary ──
        std::cout << "\n";
        std::cout << "Hand-Eye Calibration Result\n";
        std::cout << "===========================\n";
        std::cout << std::format("Mode: {}\n", mode_str);
        std::cout << std::format("Method: {}\n", he_result->method);
        std::cout << "Rotation matrix R:\n" << he_result->R << "\n";
        std::cout << "Translation vector t:\n" << he_result->t << "\n";
        std::cout << std::format("Reprojection error: {:.4F}\n", he_result->reproj_error);
        std::cout << std::format("Condition number: {:.4F}\n", he_result->condition_number);
        std::cout << std::format("Result saved to {}\n", output_path);

        return 0;

    } catch (const std::exception& e) {
        std::cerr << std::format("Error: {}\n", e.what());
        return 1;
    }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
