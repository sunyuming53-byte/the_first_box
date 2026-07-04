#include "realman_calibration/format_polyfill.hpp"

#include <filesystem>
#include <iostream>
#include <string_view>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <realman_calibration/camera_calib.hpp>

int main(int argc, char* argv[]) {
    std::filesystem::path input_dir;
    cv::Size board_size{11, 8};
    float square_size_m{0.030F};

    // Parse CLI arguments
    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    for (int i = 1; i < argc; i += 2) {
        std::string_view arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_dir = argv[i + 1];
        } else if (arg == "--board-w" && i + 1 < argc) {
            board_size.width = std::stoi(argv[i + 1]);
        } else if (arg == "--board-h" && i + 1 < argc) {
            board_size.height = std::stoi(argv[i + 1]);
        } else if (arg == "--square-size" && i + 1 < argc) {
            square_size_m = std::stof(argv[i + 1]);
        }
    }

    if (input_dir.empty()) {
        std::cerr << "Error: --input <path> is required\n";
        return 1;
    }

    // Load all .jpg images as grayscale
    std::vector<cv::Mat> images;
    for (const auto& entry : std::filesystem::directory_iterator(input_dir)) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".JPG") {
            cv::Mat img = cv::imread(entry.path().string(), cv::IMREAD_GRAYSCALE);
            if (!img.empty()) {
                images.push_back(std::move(img));
            }
        }
        // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    if (images.empty()) {
        std::cerr << std::format("Error: no .jpg images found in {}\n", input_dir.string());
        return 1;
    }

    rm::calib::CameraCalibInput input;
    input.images = std::move(images);
    input.board_size = board_size;
    input.square_size_m = square_size_m;

    rm::calib::CameraCalibrator calibrator(input);
    auto result = calibrator.compute();

    if (!result) {
        std::cerr << std::format("Error: {}\n", result.error());
        return 1;
    }

    std::cout << std::format("Camera matrix K:\n");
    std::cout << result->K << "\n\n";
    std::cout << std::format("Distortion coefficients:\n");
    std::cout << result->dist << "\n\n";
    std::cout << std::format("Reprojection error: {:.4F} px\n", result->reproj_error);
    std::cout << std::format("Images used: {}\n", result->images_used);

    return 0;
}
