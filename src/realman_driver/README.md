# RealMan Driver

C++17 / ROS2 Humble library for RealMan robot arm control.

## Quick Start

```cpp
#include "realman/arm.hpp"

int main() {
    rm::ArmConfig cfg;
    cfg.ip = "192.168.1.18";
    cfg.model = rm::ArmModel::RM_65;

    rm::Arm arm(cfg);
    arm.moveJ(rm::JointPosition({0.5, 0, 0, 0, 0, 0}), 30);

    auto state = arm.state();
    std::cout << "Joint 1: " << state.joint_position.radians[0] << " rad\n";
}
```

## Build

The SDK is included as a git submodule at `third_party/RM_API2`. From the workspace root:

```bash
git clone --recurse-submodules <this-repo>
cd realman
source /opt/ros/humble/setup.bash
colcon build
```

To build the package standalone:

```bash
cd realman_driver
mkdir build && cd build
cmake .. -DREALMAN_SDK=../../third_party/RM_API2/C
make -j$(nproc)
```

## Architecture

```
  User's rclcpp::Node
  ├── subscriber (external signal)
  ├── timer       (periodic control)
  └── arm.moveJ() (arm control)
        ↓
  rm::Arm (PIMPL, non-Node)
        ↓ C API
  libapi_c.so (RM_API2)
        ↓ TCP
  RealMan Robot Arm
```

## Two Ways to Use

**1. As Library** — full control, embed Arm in your own Node:
```cpp
rm::Arm arm(cfg);
arm.moveJ(target);
```

**2. As Standalone Node** — services over ROS2:
```bash
ros2 run realman_driver arm_node
ros2 service call /arm_node/stop std_srvs/srv/Trigger {}
```

## Supported Arm Models

RM65, RM75, ECO65, ECO63, RML63, RML63-III, GEN72, GEN72-II

## License

MIT
