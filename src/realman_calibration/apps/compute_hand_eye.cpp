#include "realman_calibration/format_polyfill.hpp"
#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
    // Parse CLI arguments for validation only
    std::string_view mode;
    std::string_view output{"calibration_result.yaml"};

    for (int i = 1; i < argc; i += 2) {
        std::string_view arg = argv[i];
        if (arg == "--mode" && i + 1 < argc) {
            mode = argv[i + 1];
        } else if (arg == "--output" && i + 1 < argc) {
            output = argv[i + 1];
        }
    }

    if (mode.empty()) {
        std::cerr << "Error: --mode <in_hand|to_hand> is required\n";
        return 1;
    }

    std::cout << std::format(
        "This tool requires pre-computed data from stages 2 and 3. "
        "Use run_pipeline instead.\n");
    return 0;
}
