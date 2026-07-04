# RealMan Robot Arm — ROS2 Workspace

ROS2 Humble workspace for controlling [RealMan](https://www.realman-robot.com/) robot arms
(RM65, RM75, ECO65, ECO63, RML63, RML63-III, GEN72, GEN72-II) via the official RM_API2 SDK.

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
ros2 run realman_driver arm_node
ros2 run realman_driver gripper_test
```

## Project Structure

```
realman/                              # ROS2 workspace root
├── src/
│   ├── realman_vision/               # Shared lib — RealSense D435 capture
│   │   ├── include/realman_vision/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── realman_driver/               # Shared lib — arm control via RM_API2 SDK
│   │   ├── include/realman/          # Public headers (subdirectories by domain)
│   │   │   ├── core/                 #   Arm, ArmConfig, ArmState, ArmError
│   │   │   ├── motion/               #   JointPosition, CartesianPose, SpeedRatio
│   │   │   ├── gripper/              #   Gripper types
│   │   │   ├── node/                 #   ArmNode — rclcpp::Node wrapper
│   │   │   └── hal/                  #   Hardware abstraction types
│   │   ├── src/                      # Implementation (subdirectories)
│   │   │   ├── core/                 #   arm.cpp, arm_impl.hpp (private), error.cpp
│   │   │   ├── motion/               #   move functions
│   │   │   ├── gripper/              #   gripper functions
│   │   │   ├── state/                #   pollState, cached state
│   │   │   └── node/                 #   ArmNode implementation
│   │   ├── examples/                 # Executables
│   │   │   ├── hello_arm.cpp         #   → movej_test
│   │   │   ├── arm_node.cpp          #   → arm_node (ROS2 standalone node)
│   │   │   ├── gripper_test.cpp      #   → gripper_test
│   │   │   ├── joint_test.cpp        #   → joint_test
│   │   │   ├── movel_test.cpp        #   → movel_test
│   │   │   └── external_trigger.cpp  #   commented out in CMakeLists.txt
│   │   ├── test/                     # ament_cmake_gtest (BUILD_TESTING gate)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── realman_calibration/          # Shared lib + executables — hand-eye calibration
│   │   ├── include/realman_calibration/
│   │   ├── src/
│   │   ├── apps/                     # Pipeline executables + calib_node
│   │   ├── config/                   # Board config YAML
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── realman_hardware/             # ros2_control plugin
│   │   ├── include/
│   │   ├── src/
│   │   ├── plugins.xml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── realman_bringup/              # Launch + config only (no compiled code)
│       ├── launch/
│       ├── config/
│       ├── CMakeLists.txt
│       └── package.xml
├── cmake/
│   └── RealManSDKConfig.cmake        # CMake find module for libapi_c.so
├── scripts/                          # Deployment scripts
│   ├── deploy-remote                 #   Sync + restart services on robot
│   ├── sync-remote                   #   Rsync install/ to runtime container
│   ├── ssh-remote                    #   SSH into runtime (port 2022)
│   ├── generate-compile-commands.sh  #   Merge clangd compile_commands.json
│   ├── entrypoint-dev.sh             #   Develop container entrypoint
│   ├── entrypoint-runtime.sh         #   Runtime container entrypoint
│   └── supervisord.conf              #   Supervisor config (auto-starts arm_node + calib_node)
├── .devcontainer/
│   └── devcontainer.json             # VS Code dev container config
├── Dockerfile                        # Multi-stage (develop + runtime)
├── docker-compose.yml
├── .env.example                      # Proxy + ROS_DOMAIN_ID config
├── third_party/
│   └── RM_API2/                      # Git submodule — official RealMan C SDK
│       └── C/
│           ├── include/              #   rm_interface.h, etc.
│           └── linux/                #   libapi_c.so (versioned: linux_x86_c_vv1.1.5/)
├── docs/                             # Project documentation
└── knowledge-base/                   # Notes & references
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

The SDK is discovered at `/opt/realman-sdk` (or `$REALMAN_SDK`) via
`find_package(RealManSDK REQUIRED)` backed by `cmake/RealManSDKConfig.cmake`.
The Docker build copies SDK files from the submodule to this location.

```bash
source /opt/ros/humble/setup.bash
colcon build
```

Override SDK path:
```bash
colcon build --cmake-args -DREALMAN_SDK=/custom/path
```

### Build order (automatic with colcon)

`realman_vision` → `realman_driver` → `realman_calibration` / `realman_hardware` → `realman_bringup`

`colcon build` resolves this automatically, but it matters when building packages individually or adding cross-package dependencies.

### clangd IntelliSense

For IDE support, build with compile_commands and merge across packages:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
generate-compile-commands.sh
```

This produces `build/compile_commands.json` at the workspace root, which clangd
uses for go-to-definition, diagnostics, and completions. The dev container runs
this automatically on creation.

## Testing

Tests use `ament_cmake_gtest` and are gated behind `BUILD_TESTING`:

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test
```

Test binaries need the SDK library on `LD_LIBRARY_PATH` — the CMake config
handles this via `APPEND_ENV`.

## Docker & Dev Container

The project uses a multi-stage Dockerfile and VS Code dev container.

### Develop container (GUI + build tools)

```bash
# Via docker compose (recommended):
docker compose up develop

# Or manual build:
docker build . --target realman-develop -t realman:develop
docker run -it --network host --device /dev \
    -v $(pwd):/ws -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY \
    realman:develop
```

Or open in VS Code → "Reopen in Container" (uses `.devcontainer/devcontainer.json`).

### Runtime container (robot MiniPC — headless)

```bash
docker compose up runtime -d
```

The runtime container runs supervisor with auto-starting `arm_node` + `calib_node`,
plus an SSH server on port 2022 for receiving built artifacts.

### Deploy to robot

From the develop container:

```bash
sync-remote <robot-ip>      # rsync install/ to runtime container (SSH port 2022)
deploy-remote <robot-ip>    # sync + supervisor restart
ssh-remote <robot-ip>       # SSH into runtime
```

### Proxy config

If behind a local proxy (Clash, v2ray, etc.), copy `.env.example` to `.env` and
customize. `docker compose` reads proxy variables from `.env`.

## Updating the SDK

The SDK is pinned as a git submodule at `third_party/RM_API2`. To update:

```bash
cd third_party/RM_API2
git fetch
git checkout <desired-tag-or-branch>
cd ../..
git add third_party/RM_API2
git commit -m "chore: update RM_API2 submodule to <version>"
```

The C SDK `.so` is versioned at `third_party/RM_API2/C/linux/linux_x86_c_vv1.1.5/libapi_c.so`.
If the versioned path changes, update the `COPY` commands in the Dockerfile.

## Supported Arm Models

RM65 · RM75 · ECO65 · ECO63 · RML63 · RML63-III · GEN72 · GEN72-II

## License

MIT — see [LICENSE](LICENSE)
