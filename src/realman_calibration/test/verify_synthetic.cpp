/// Quick smoke-test: verify chessboard detection works on synthetic images.
/// Builds and runs as part of the test infrastructure.

#include "synthetic_board.h"

#include <opencv2/calib3d.hpp>
#include <cstdio>
#include <cstdlib>

int main() {
  constexpr int kCount = 5;
  const cv::Size kBoardSize(9, 6);
  const float kSquareM = 0.030f;
  const cv::Size kImageSize(640, 480);

  // ── synthetic camera intrinsics (640×480, fov ~60°) ──
  cv::Mat K = (cv::Mat_<double>(3, 3) << 500.0, 0.0, 320.0,
                                           0.0, 500.0, 240.0,
                                           0.0, 0.0, 1.0);
  cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);

  auto images = rm::calib::test::generate_chessboard_images(
      kCount, kBoardSize, kSquareM, K, dist, kImageSize);

  if (static_cast<int>(images.size()) != kCount) {
    std::fprintf(stderr, "FAIL: expected %d images, got %zu\n", kCount,
                 images.size());
    return 1;
  }

  int detected = 0;
  for (int i = 0; i < kCount; ++i) {
    if (images[i].type() != CV_8UC1) {
      std::fprintf(stderr, "FAIL: image %d is not CV_8UC1 (type=%d)\n", i,
                   images[i].type());
      return 1;
    }

    std::vector<cv::Point2f> corners;
    bool found =
        cv::findChessboardCorners(images[i], kBoardSize, corners);
    if (found) {
      ++detected;
    } else {
      std::fprintf(stderr, "WARN: chessboard not detected on image %d\n", i);
    }
  }

  if (detected < kCount) {
    std::fprintf(stderr, "FAIL: only %d/%d images passed chessboard detection\n",
                 detected, kCount);
    return 1;
  }

  std::printf("PASS: all %d images passed chessboard detection\n", detected);
  return 0;
}
