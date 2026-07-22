# omr_vision

Intel RealSense camera capture, calibration helpers, and white-label PnP for the
OMRobot pipeline.

**Dependencies:** OpenCV, librealsense2

## What it does

- Captures RGB and depth frames from Intel RealSense D400 series cameras
- Provides a `CameraStream` abstraction for frame polling (`color_intrinsics` /
  `depth_intrinsics`)
- Camera / Charuco intrinsic calibration utilities
- White rectangular label detection + PnP (`detection/white_label.hpp`) for
  popped-out door-handle tags (default 20×77.5 mm)

## White label PnP (live D435)

With a RealSense connected and a white 20×77.5 mm label visible:

```bash
# after colcon build --packages-select omr_vision
source install/setup.bash
ros2 run omr_vision detect_white_label
# optional: --width-m 0.020 --height-m 0.0775 --threshold 200
```

Press `q` to quit. The window draws label corners, frame axes, and camera-frame
`tvec` (meters).

## Quick start (capture)

```cpp
#include "omr_vision/camera/stream.hpp"

omr_vision::camera::CameraStream stream({});
auto frame = stream.next();  // optional aligned RGB-D
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omr_vision --cmake-args -DBUILD_TESTING=ON
```
