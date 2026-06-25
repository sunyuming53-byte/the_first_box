# RealMan Robot Arm — ROS2 Workspace

ROS2 Humble workspace for controlling [RealMan](https://www.realman-robot.com/) robot arms
(RM65, RM75, ECO65, RML63, GEN72, etc.) via the official RM_API2 SDK.

## Prerequisites

- **Ubuntu 22.04** (Jammy)
- **ROS2 Humble** — [install guide](https://docs.ros.org/en/humble/Installation.html)
- **Git** + **SSH key** registered with GitHub (the SDK submodule uses SSH)

Check your ROS2 setup:

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

## Quick Start

```bash
# 1. Clone with submodule
git clone --recurse-submodules git@github.com:<your-org>/realman.git
cd realman

# 2. Build
source /opt/ros/humble/setup.bash
colcon build

# 3. Source and run an example
source install/setup.bash
ros2 run realman_driver hello_arm   # (when example executables are enabled)
```

## Project Structure

```
realman/                        # ROS2 workspace root
├── src/
│   └── realman_driver/          # ROS2 package — C++ arm driver library
│       ├── include/realman/    # Public headers
│       │   ├── arm.hpp         #   rm::Arm — main control interface
│       │   ├── types.hpp       #   JointPosition, CartesianPose, ArmConfig, etc.
│       │   ├── error.hpp       #   rm::ArmError exception
│       │   └── arm_node.hpp    #   rm::ArmNode — ROS2 node wrapper
│       ├── src/                # Implementation
│       │   ├── arm.cpp         #   PIMPL + worker thread + RM_API2 calls
│       │   ├── error.cpp       #   Error formatting
│       │   └── arm_node.cpp    #   Node with stop service
│       ├── examples/           # Usage examples
│       │   ├── hello_arm.cpp
│       │   ├── external_trigger.cpp
│       │   └── arm_node.cpp
│       ├── CMakeLists.txt
│       └── package.xml
├── third_party/
│   └── RM_API2/                # Git submodule — official RealMan SDK
│       └── C/                  # C SDK (headers + shared libs)
├── docs/                       # Project documentation
├── knowledge-base/             # Notes & references
├── build/                      # colcon build output (git-ignored)
├── install/                    # colcon install output (git-ignored)
└── log/                        # colcon build logs (git-ignored)
```

## Architecture

```
  Your rclcpp::Node
  ├── subscriber   (external trigger)
  ├── timer        (periodic control loop)
  └── arm.moveJ()  (arm control)
        │
        ▼
  rm::Arm (PIMPL, non-ROS)
        │  C API
        ▼
  libapi_c.so (RM_API2 SDK)
        │  TCP
        ▼
  RealMan Robot Arm
```

Two usage modes:

1. **As a library** — embed `rm::Arm` in your own ROS2 node for full control
2. **As a standalone node** — run `arm_node` and call its ROS2 services

## Building

A standard `colcon build` is all you need. The CMakeLists.txt auto-discovers
the SDK at `third_party/RM_API2/C`.

```bash
source /opt/ros/humble/setup.bash
colcon build
```

If you placed the SDK elsewhere, override with:

```bash
colcon build --cmake-args -DREALMAN_SDK=/custom/path/to/RM_API2/C
```

## Updating the SDK

The SDK is pinned as a git submodule. To update to the latest upstream:

```bash
cd third_party/RM_API2
git fetch
git checkout <desired-tag-or-branch>
cd ../..
git add third_party/RM_API2
git commit -m "chore: update RM_API2 submodule to <version>"
```

## Supported Arm Models

RM65 · RM75 · ECO65 · ECO63 · RML63 · RML63-III · GEN72 · GEN72-II

## License

MIT — see [LICENSE](LICENSE) 
