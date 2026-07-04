#include "realman_vision/camera/types.hpp"

namespace rm::vision::camera {

auto CameraFrame::point_3d(int u, int v) const -> std::array<double, 3> {
    // depth is in mm (CV_16UC1), convert to meters
    auto z = static_cast<double>(depth.at<uint16_t>(v, u)) / 1000.0;

    auto x = (static_cast<double>(u) - depth_intrinsics.cx()) / depth_intrinsics.fx() * z;
    auto y = (static_cast<double>(v) - depth_intrinsics.cy()) / depth_intrinsics.fy() * z;

    return {x, y, z};
}

}  // namespace rm::vision::camera
