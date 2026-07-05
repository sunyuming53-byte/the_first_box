# omr_vision

Intel RealSense camera capture and streaming for the RealMan pipeline.

**Dependencies:** OpenCV, librealsense2

## What it does

- Captures RGB and depth frames from Intel RealSense D400 series cameras
- Provides a `CameraStream` abstraction for frame polling
- Defines `CameraConfig` and common types (`Frame`, `Intrinsics`)

## Quick start

```cpp
#include "omr_vision/capture.hpp"

auto capture = rm_vision::Capture::create();
auto frame = capture->grab();  // blocks until next frame
```

## Build

This package is a dependency of `realman_calibration`. Build the full workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build
```
