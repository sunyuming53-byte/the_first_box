#include "realman_calibration/format_polyfill.hpp"

#include <iostream>
#include <string_view>

#include <realman_calibration/collector.hpp>

int main(int argc, char* argv[]) {
    rm::calib::CalibDataConfig cfg;

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
        }
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    rm::calib::CalibDataCollector collector(cfg);

    auto result = collector.run();
    if (!result) {
        std::cerr << std::format("Error: {}\n", result.error());
        return 1;
    }

    std::cout << std::format("Session path: {}\n", result->dir.string());
    return 0;
}
