#include <realman_calibration/pose_proc.hpp>

#include <array>
#include <filesystem>
#include "realman_calibration/format_polyfill.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

int main(int argc, char* argv[]) {
    std::filesystem::path poses_path;
    std::string mode_str;

    // Parse CLI arguments
    for (int i = 1; i < argc; i += 2) {
        std::string_view arg = argv[i];
        if (arg == "--poses" && i + 1 < argc) {
            poses_path = argv[i + 1];
        } else if (arg == "--mode" && i + 1 < argc) {
            mode_str = argv[i + 1];
        }
    }

    if (poses_path.empty()) {
        std::cerr << "Error: --poses <path> is required\n";
        return 1;
    }
    if (mode_str.empty()) {
        std::cerr << "Error: --mode <in_hand|to_hand> is required\n";
        return 1;
    }

    rm::calib::HandEyeMode mode;
    if (mode_str == "in_hand") {
        mode = rm::calib::HandEyeMode::EyeInHand;
    } else if (mode_str == "to_hand") {
        mode = rm::calib::HandEyeMode::EyeToHand;
    } else {
        std::cerr << std::format("Error: unknown mode '{}' (expected in_hand or to_hand)\n", mode_str);
        return 1;
    }

    // Read CSV: skip header row, parse 6 doubles per line
    std::ifstream file(poses_path);
    if (!file.is_open()) {
        std::cerr << std::format("Error: cannot open {}\n", poses_path.string());
        return 1;
    }

    std::vector<std::array<double, 6>> poses;
    std::string line;
    bool first_line = true;

    while (std::getline(file, line)) {
        if (first_line) {
            first_line = false;
            continue;  // skip header
        }
        if (line.empty()) continue;

        std::array<double, 6> pose{};
        std::istringstream ss(line);
        std::string token;
        for (int j = 0; j < 6 && std::getline(ss, token, ','); ++j) {
            pose[j] = std::stod(token);
        }
        poses.push_back(pose);
    }

    if (poses.empty()) {
        std::cerr << std::format("Error: no pose data found in {}\n", poses_path.string());
        return 1;
    }

    rm::calib::PoseProcessor processor(mode);
    auto result = processor.process(poses);

    if (!result) {
        std::cerr << std::format("Error: {}\n", result.error());
        return 1;
    }

    std::cout << std::format("Motion pairs computed: {}\n", result->R_motions.size());
    return 0;
}
