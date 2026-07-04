# AGENTS.md — RealMan ROS2 Workspace

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build
```

Override SDK path:
```bash
colcon build --cmake-args -DREALMAN_SDK=/opt/realman-sdk
```

The SDK is expected at `/opt/realman-sdk` (or `$REALMAN_SDK` env var),
with `include/` and `lib/libapi_c.so` underneath. Discovery happens via
`find_package(RealManSDK REQUIRED)` backed by `cmake/RealManSDKConfig.cmake`.
The Dockerfile copies the SDK from the submodule to `/opt/realman-sdk/` at
build time.

**Build order matters**: `realman_vision` and `realman_driver` must be built
before `realman_calibration` (which depends on both), then `realman_hardware`
(depends on `realman_driver`), then `realman_bringup` (launch/config only).
`colcon build` handles this automatically.

**IntelliSense**: clangd requires a merged `compile_commands.json` at the
workspace root. Build with `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`
and then run `generate-compile-commands.sh` to merge per-package files into
`build/compile_commands.json`. The dev container does this automatically in
`postCreateCommand`.

## Workspace packages

| Package | Type | Purpose |
|---|---|---|
| `realman_vision` | Shared lib | RealSense D435 capture (OpenCV + librealsense2) |
| `realman_driver` | Shared lib | Arm control via RM_API2 C SDK |
| `realman_calibration` | Shared lib + executables | Hand-eye calibration pipeline |
| `realman_hardware` | Plugin | ros2_control hardware interface |
| `realman_bringup` | Launch/config | System-level launch files |

Build order: `vision` → `driver` → `calibration` / `hardware` → `bringup`.

## Architecture

```
rm::Arm   (PIMPL facade, non-ROS)
  └─ Arm::Impl   (single worker thread + command queue)
       └─ libapi_c.so   (C SDK, TCP to the arm)
```

- `rm::Arm` is NOT a `rclcpp::Node`. It is a plain C++ class usable in any context.
- `rm::ArmNode` wraps `rm::Arm` in a `rclcpp::Node` with ROS2 services.
- All arm commands (motion, gripper, stop) are serialized through one worker thread
  with a `std::queue<std::function<void()>>`. The caller-facing API blocks via
  `std::condition_variable` when `blocking=true`.

### Source layout (realman_driver)

Public headers at `include/realman/` organized by subsystem:
```
include/realman/
├── core/          # Arm, ArmConfig, ArmState, ArmError
├── motion/        # JointPosition, CartesianPose, SpeedRatio typedefs
├── gripper/       # Gripper types
├── node/          # ArmNode (rclcpp::Node wrapper)
└── hal/           # Hardware abstraction types
```

Implementation at `src/` organized in subdirectories:
```
src/
├── core/          # arm.cpp, arm_impl.hpp (private), error.cpp
├── motion/        # move functions
├── gripper/       # gripper functions
├── state/         # pollState and cached state
└── node/          # ArmNode implementation
```

`CMakeLists.txt` uses `GLOB_RECURSE` with `CONFIGURE_DEPENDS` across these
subdirs. Adding a `.cpp` to any of `src/{core,motion,gripper,state,node}/`
will auto-trigger reconfiguration after the next edit.

## Unit conversion: radians vs degrees (CRITICAL)

The public API (`JointPosition`) uses **radians**. The C SDK uses **degrees**.
Conversion happens inside `Arm::Impl`:

- `moveJ()`: converts radians → degrees before calling `rm_movej()`
- `pollState()`: converts degrees → radians when reading joint positions

When adding new joint-angle API calls, match this convention or the arm will
receive wrong values.

## V1 stubs (unimplemented)

These methods throw `rm::ArmError("not implemented in V1")`:

- `moveJ_CANFD` / `moveP_CANFD`
- `getWorkFrames` / `setWorkFrame`
- `enableForceControl` / `disableForceControl`

If a user asks about these, they need implementation — don't assume they work.

## Example executables

`movej_test` (hello_arm.cpp), `arm_node`, `gripper_test`, `joint_test`,
`movel_test` are built. `external_trigger` is commented out — uncomment in
`CMakeLists.txt` to build it.  Do NOT modify `package.xml` to declare these
as dependencies unless they are actually built.

Run examples: `ros2 run realman_driver <executable>` (after `source install/setup.bash`).

## C++ standard

The project uses **C++23**. The ROS2 Humble base image ships GCC 11 which
has partial C++23 support — avoid features that require GCC 12+ (e.g.,
`std::expected`, `std::ranges::to`).

## SDK submodule

```bash
git submodule update --init --recursive   # after clone
```

The submodule URL is `git@github.com:RealManRobot/RM_API2.git` (SSH). If a user
lacks SSH keys, they'll need to switch to HTTPS in `.gitmodules`.

The C SDK `.so` is versioned at `third_party/RM_API2/C/linux/linux_x86_c_vv1.1.5/libapi_c.so`.
The cmake config discovers it via glob on `linux/linux_x86_c_vv*/libapi_c.so`.
When updating the submodule, check if the versioned path changed.

## ROS2 package

- Package name: `realman_driver`
- Depends on: `rclcpp`, `std_srvs`, `tf2_ros`, `geometry_msgs`, `sensor_msgs`
- The package exports a shared library (`librealman_driver.so`), not an executable

## Testing

Tests are gated by `BUILD_TESTING` (`ament_cmake_gtest`). Build with:
```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test
```

Test binaries need `LD_LIBRARY_PATH` pointing to the SDK lib — CMakeLists.txt
uses `APPEND_ENV` for this.

There is no CI pipeline, no `.clang-format`, and no pre-commit hooks. Do not
add formatting or linting config unless explicitly requested.

## Docker & deployment

Two-container architecture:

- **Develop** (`realman-develop` stage): GUI, build tools, workspace mounted at `/ws`
- **Runtime** (`realman-runtime` stage): headless, runs on robot MiniPC with
  supervisor auto-starting `arm_node` + `calib_node`

Deploy workflow (from develop container):
```bash
sync-remote <robot-ip>      # rsync install/ to runtime container over SSH port 2022
deploy-remote <robot-ip>    # sync + supervisor restart
ssh-remote <robot-ip>       # SSH into runtime
```

The runtime container expects built artifacts at `/ws/install`. Supervisor config
sources both `/opt/ros/humble/setup.bash` and `/ws/install/setup.bash` before
launching nodes.

Proxy config for apt/pip behind a local proxy: copy `.env.example` to `.env` and
customize. `docker compose` reads the proxy vars from `.env`.

## Dependencies not in workspace

`realman_calibration` depends on `realman_vision` — this **is** a workspace
package (`src/realman_vision/`), not an external dependency. It must be built
before the calibration package.
