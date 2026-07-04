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

# 3. Source and run examples
source install/setup.bash
ros2 run realman_arm movej_test
ros2 run realman_arm gripper_test
```

## Project Structure

```
realman/                              # ROS2 workspace root
├── src/
│   ├── realman_vision/               # ament_cmake — RealSense D435 capture (no ROS deps)
│   │   ├── include/realman_vision/
│   │   │   ├── camera/               #   CameraStream, CameraConfig
│   │   │   └── capture.hpp           #   Capture abstraction
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── realman_arm/                  # Plain CMake — arm control via RM_API2 SDK (ZERO ROS deps)
│   │   ├── include/realman/          # Public headers (subdirectories by domain)
│   │   │   ├── core/                 #   Arm (PIMPL), ArmConfig, ArmState, ArmError
│   │   │   ├── motion/               #   JointPosition, CartesianPose, SpeedRatio
│   │   │   ├── gripper/              #   GripperState, GripperAction
│   │   │   └── hal/                  #   Hardware abstraction types
│   │   ├── src/                      # Implementation (subdirectories)
│   │   │   ├── core/                 #   arm_facade.cpp, arm_impl.hpp (private), connection.cpp, error.cpp
│   │   │   ├── motion/               #   motion.cpp (moveJ, moveL, moveC, moveJ_P)
│   │   │   ├── gripper/              #   gripper.cpp
│   │   │   └── state/                #   state.cpp (pollState, cached state)
│   │   ├── examples/                 # Executables
│   │   │   ├── hello_arm.cpp         #   → movej_test
│   │   │   ├── gripper_test.cpp      #   → gripper_test
│   │   │   ├── joint_test.cpp        #   → joint_test
│   │   │   ├── movel_test.cpp        #   → movel_test
│   │   │   └── external_trigger.cpp  #   commented out in CMakeLists.txt
│   │   ├── cmake/                    #   RealManSDKConfig.cmake, FindRealManSDK.cmake
│   │   ├── test/                     #   ament_cmake_gtest (BUILD_TESTING gate)
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── realman_calibration/          # ament_cmake — hand-eye calibration
│   │   ├── include/realman_calibration/
│   │   ├── src/
│   │   ├── apps/                     # Pipeline executables + calib_node
│   │   ├── config/                   # Board config YAML
│   │   ├── launch/
│   │   ├── test/                     # Comprehensive test suite
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── realman_hardware/             # ament_cmake — ros2_control plugin
│   │   ├── include/
│   │   ├── src/
│   │   ├── plugins.xml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── realman_bringup/              # ament_cmake — launch + config only (no compiled code)
│       ├── launch/
│       ├── config/
│       ├── CMakeLists.txt
│       └── package.xml
├── cmake/                            # Shared CMake modules (clang_tidy.cmake, etc.)
├── scripts/                          # Deployment scripts
│   ├── deploy-remote                 #   Sync + restart services on robot
│   ├── sync-remote                   #   Rsync install/ to runtime container
│   ├── ssh-remote                    #   SSH into runtime (port 2022)
│   ├── generate-compile-commands.sh  #   Merge clangd compile_commands.json
│   ├── entrypoint-dev.sh             #   Develop container entrypoint
│   ├── entrypoint-runtime.sh         #   Runtime container entrypoint
│   └── supervisord.conf              #   Supervisor config (auto-starts controller_manager + calib_node)
├── .devcontainer/
│   └── devcontainer.json             # VS Code dev container config
├── Dockerfile                        # Multi-stage (develop + runtime)
├── docker-compose.yml
├── .env.example                      # Proxy + ROS_DOMAIN_ID config
├── .clang-format                     # Google-based, 4-space indent, 100col
├── .clang-tidy                       # C++23 target, GCC 11.4 toolchain
├── third_party/
│   └── RM_API2/                      # Git submodule — official RealMan C SDK
│       └── C/
│           ├── include/              #   rm_interface.h, etc.
│           └── linux/                #   libapi_c.so (versioned: linux_x86_c_vv1.1.5/)
└── docs/                             # Project documentation
```

## Architecture

```
  Your code (ROS2 node, ros2_control, or plain C++)
  └── arm.moveJ()     (arm control)
        │
        ▼
  rm::Arm             (PIMPL, non-ROS — single worker thread + command queue)
        │  C API
        ▼
  libapi_c.so         (RM_API2 SDK)
        │  TCP
        ▼
  RealMan Robot Arm
```

`rm::Arm` is a plain C++ class (not an `rclcpp::Node`) with **zero ROS dependency**.
It can be used in any context — embedded in your own ROS2 node, linked into a
`ros2_control` hardware interface, or used standalone outside ROS2.

## Building

The SDK is discovered at `/opt/realman-sdk` (or `$REALMAN_SDK`) via
`find_package(RealManSDK REQUIRED)` backed by `src/realman_arm/cmake/RealManSDKConfig.cmake`.
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

`realman_vision` → `realman_arm` → `realman_calibration` / `realman_hardware` → `realman_bringup`

`colcon build` resolves this automatically, but it matters when building packages individually or adding cross-package dependencies.

### clangd IntelliSense

For IDE support, build with compile_commands and merge across packages:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
generate-compile-commands.sh
```

This produces `build/compile_commands.json` at the workspace root, which clangd
uses for go-to-definition, diagnostics, and completions. The `.clangd` config
suppresses ROS2-header false positives and disables `UnusedIncludes`. The dev
container runs this automatically on creation.

## Testing

Tests use `ament_cmake_gtest` and are gated behind `BUILD_TESTING`:

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test
```

Test binaries need the SDK library on `LD_LIBRARY_PATH` — the CMake config
handles this via `APPEND_ENV`.

`realman_calibration` has the most comprehensive test suite: camera calibration,
pose processing, hand-eye solvers, TF integration, and synthetic data generators.

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

The runtime container runs supervisor with auto-starting `controller_manager` +
`calib_node`, plus an SSH server on port 2022 for receiving built artifacts.

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
