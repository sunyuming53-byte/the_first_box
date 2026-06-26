#pragma once

#include "realman_vision/camera/types.hpp"
#include <memory>
#include <optional>

namespace rm::vision::camera {

class CameraStream {
public:
    explicit CameraStream(const CameraConfig& cfg);
    ~CameraStream();

    CameraStream(const CameraStream&) = delete;
    auto operator=(const CameraStream&) -> CameraStream& = delete;
    CameraStream(CameraStream&&) noexcept;
    auto operator=(CameraStream&&) noexcept -> CameraStream&;

    // Block until an aligned RGB-D frame pair arrives.
    [[nodiscard]] auto next() -> std::optional<CameraFrame>;

    // Depth sensor intrinsics (invariant across frames).
    [[nodiscard]] auto depth_intrinsics() const -> CameraIntrinsics;

    // True while the pipeline has frames to deliver.
    [[nodiscard]] explicit operator bool() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rm::vision::camera
