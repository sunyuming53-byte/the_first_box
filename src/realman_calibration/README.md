# realman_calibration

Hand-eye calibration pipeline for RealMan robot arms.

**Dependencies:** `realman_vision`, `realman_driver`, OpenCV, librealsense2

## What it does

- **Camera calibration** — compute intrinsics from a chessboard/Charuco board
- **Pose collection** — record robot TCP poses + camera board detections
- **Hand-eye calibration** — solve AX=XB for camera-to-flange transform
- **Coordinate transformation** — project camera-frame points to robot base frame

## Pipeline executables

| Executable | Purpose |
|---|---|
| `calibrate_camera` | Compute camera intrinsics from calibration images |
| `collect_data` | Record synchronized arm poses + camera frames |
| `process_poses` | Extract board poses from collected frames |
| `compute_hand_eye` | Solve hand-eye transform (Tsai / Park / Daniilidis) |
| `run_pipeline` | End-to-end calibration (all steps) |
| `calib_node` | ROS2 node for interactive calibration workflow |

## Quick start

```bash
# 1. Calibrate camera (print a Charuco board first)
ros2 run realman_calibration calibrate_camera --config config/board.yaml

# 2. Collect calibration data (arm + camera synchronized)
ros2 run realman_calibration collect_data

# 3. Compute hand-eye transform
ros2 run realman_calibration compute_hand_eye

# Or run the full pipeline at once:
ros2 run realman_calibration run_pipeline
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build
```
