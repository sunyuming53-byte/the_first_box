# Visual Perception Pipeline Index

Mapping of documentation to pipeline use cases for RealMan robotic arm visual grasping.

## Use Case → Document Map

| Pipeline Stage | What You Need | Primary Document | Supporting |
|---|---|---|---|
| **Hardware Setup** | D435 camera, arm model, wiring | [01-yolov8-recognition.md](./01-yolov8-recognition.md) §1 | [02-hand-eye-calibration.md](./02-hand-eye-calibration.md) §4.2 |
| **Environment Setup** | ROS Noetic, Python 3.8, CUDA, D435 driver | [01-yolov8-recognition.md](./01-yolov8-recognition.md) §2 | — |
| **Visual Detection** | YOLOV8 model loading + RGB-D inference | [01-yolov8-recognition.md](./01-yolov8-recognition.md) §6 | — |
| **ROS Integration** | Topic publish/subscribe, launch files | [01-yolov8-recognition.md](./01-yolov8-recognition.md) §4-5 | — |
| **Hand-Eye Calibration** | Calibration board, data collection, OpenCV solve | [02-hand-eye-calibration.md](./02-hand-eye-calibration.md) §4 | — |
| **Coordinate Transform** | Chain homogeneous transformation (3D point) | [03-coordinate-transformation.md](./03-coordinate-transformation.md) §2 | — |
| **Coordinate Transform** | Chain homogeneous transformation (6D pose) | [03-coordinate-transformation.md](./03-coordinate-transformation.md) §3 | — |
| **Grasping Execution** | Multi-segment trajectory planning | [01-yolov8-recognition.md](./01-yolov8-recognition.md) §7 | — |
| **Debugging** | Calibration failure, coordinate mismatch | [02-hand-eye-calibration.md](./02-hand-eye-calibration.md) §6 | — |

## Critical Integration Points

### 1. Camera ↔ Arm

```
D435 (USB)
  ↓ RGB-D frames
YOLOV8 (Python)
  ↓ ObjectInfo {class, x, y, z} on /object_pose
convert() (numpy)
  ↓ H_cam_ee * H_ee_base * p_cam
rm::Arm (C++) / RM API
  ↓ moveJ / moveL / gripper
RealMan Robot Arm
```

### 2. Hand-Eye Calibration Result Is the Linchpin

```
手眼标定 (one-time offline)
  ↓
rotation_matrix (3×3) + translation_vector (3×1)
  ↓ fed into
convert() in every online grasping cycle
```

If hand-eye calibration is wrong, **every subsequent grasp will miss**.

### 3. ROS1 → ROS2 Migration Map

| ROS1 Component | ROS2 Equivalent |
|---|---|
| `vi_grab/` package | New ROS2 package (port Python scripts) |
| `vi_msgs/ObjectInfo.msg` | New ROS2 `.msg` definition |
| `/rm_driver/Arm_Current_State` topic | `rm::Arm` C++ API or `rm_driver` ROS2 topic |
| `rospy.wait_for_message()` | ROS2 subscription / service call |
| `catkin build` | `colcon build --packages-select vi_grab` |
| `roslaunch vi_grab vi_grab_demo.launch` | ROS2 launch file |

### 4. Known ROS2 Package to Extend

Current workspace has:
```
src/realman_driver/   # rm::Arm PIMPL + rm::ArmNode
```

To add visual grasping, create alongside:
```
src/vi_grab/          # Ported from ROS1 vi_grab
  ├── vi_grab/
  │   ├── camera_node.py       # D435 + YOLOV8 detection
  │   ├── grasp_node.py        # Coordinate transform + motion planning
  │   └── object_info.py       # ROS2 message bridge
  ├── launch/
  │   └── vi_grab_demo.launch.py
  └── package.xml
```

### 5. Math Dependencies (Pure Python, No ROS)

```python
# Only these 3 imports are needed for the coordinate transform core
import numpy as np
from scipy.spatial.transform import Rotation
import cv2  # for camera calibration only
```

## Source References

- YOLOV8 Demo: https://develop.realman-robotics.com/symbiosis/demo/YOLOV8VisualRecognition/
- YOLOV8 Code: https://github.com/RealManRobot/YOLOv8-Visual-Recognition
- Hand-Eye Calibration Guide: https://develop.realman-robotics.com/AI/developerGuide/hand/
- Hand-Eye Code: https://github.com/RealManRobot/hand_eye_calibration
