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
`find_package(RealManSDK REQUIRED)` backed by `src/realman_arm/cmake/RealManSDKConfig.cmake`.
The Dockerfile copies the SDK from the submodule to `/opt/realman-sdk/` at
build time.

**Build order matters**: `realman_vision` (ament_cmake, no ROS deps) →
`realman_arm` (plain CMake, no ROS deps) → `realman_calibration` (depends on
both arm + vision) / `realman_hardware` (depends on arm) → `realman_bringup`
(launch/config only). `colcon build` handles this automatically.

### clangd IntelliSense

Build with compile_commands and merge across packages:

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
generate-compile-commands.sh
```

The `.clangd` config suppresses ROS2-header diagnostics (false positives from
system includes). `UnusedIncludes` is disabled.

## Workspace packages

| Package | Build system | Purpose |
|---|---|---|
| `realman_vision` | ament_cmake | RealSense D435 capture (OpenCV + librealsense2). No ROS deps. |
| `realman_arm` | **Plain CMake** | Arm control via RM_API2 C SDK. **Zero ROS dependency.** Deployable anywhere. |
| `realman_calibration` | ament_cmake | Hand-eye calibration pipeline. Depends on both arm + vision. |
| `realman_hardware` | ament_cmake | ros2_control hardware interface plugin. Depends on arm. |
| `realman_bringup` | ament_cmake | Launch files + config only. No compiled code. |

**Critical**: `realman_arm` is a **plain CMake project** (NOT `ament_cmake`).
It has no ROS dependencies and can be used outside ROS2. Do NOT add
`ament_cmake`, `rclcpp`, or any ROS dependency to it.

## Architecture

```
rm::Arm   (PIMPL facade, non-ROS, non-copyable, movable)
  └─ Arm::Impl   (private, single worker thread + command queue)
       └─ libapi_c.so   (C SDK, TCP to the arm)
```

- `rm::Arm` is NOT a `rclcpp::Node`. It is a plain C++ class usable in any context.
- All arm commands (motion, gripper, stop) are serialized through **one worker thread**
  with `std::queue<std::function<void()>>`. The caller-facing API blocks via
  `std::condition_variable` when `blocking=true`.
- **Lazy connection**: `rm_init()` runs at construction; actual TCP connection to
  the arm is deferred to the first command via `ensureConnected()`. This allows
  constructing `Arm` without hardware present.
- There is NO `ArmNode` class and NO `arm_node` executable. The runtime supervisor
  starts `ros2_control_node` (via `controller_manager`) and `calib_node`.

### Source layout (realman_arm)

```
src/realman_arm/
├── include/realman/
│   ├── core/          # Arm (PIMPL), ArmConfig, ArmState, ArmError
│   ├── motion/        # JointPosition, CartesianPose, SpeedRatio typedefs
│   ├── gripper/       # GripperState, GripperAction typedefs
│   └── hal/           # Hardware abstraction types
├── src/
│   ├── core/          # arm_facade.cpp, arm_impl.hpp (private), connection.cpp, error.cpp
│   ├── motion/        # motion.cpp (moveJ, moveL, moveC, moveJ_P)
│   ├── gripper/       # gripper.cpp
│   └── state/         # state.cpp (pollState, cached state)
├── examples/          # hello_arm.cpp → movej_test, gripper_test.cpp, etc.
├── cmake/             # RealManSDKConfig.cmake, FindRealManSDK.cmake
└── test/
```

`CMakeLists.txt` uses `GLOB_RECURSE` with `CONFIGURE_DEPENDS` across these
subdirs. Adding a `.cpp` to any of `src/{core,motion,gripper,state}/`
auto-triggers reconfiguration after the next edit.

## Unit conversion: radians vs degrees (CRITICAL)

The public API (`JointPosition`) uses **radians**. The C SDK uses **degrees**.
Conversion happens inside `Arm::Impl`:

- `motion.cpp` (moveJ, moveL, etc.): converts radians → degrees before calling SDK
- `state.cpp` (pollState): converts degrees → radians when reading joint positions

When adding new joint-angle API calls, match this convention or the arm will
receive wrong values. Look at `motion.cpp` and `state.cpp` for the exact
conversion pattern.

## V1 stubs (unimplemented)

These throw `rm::ArmError("not implemented in V1")`:

- `moveJ_CANFD` / `moveP_CANFD`
- `getWorkFrames` / `setWorkFrame`
- `enableForceControl` / `disableForceControl`

If a user asks about these, they need implementation — don't assume they work.

## Example executables

Built: `movej_test` (hello_arm.cpp), `gripper_test`, `joint_test`, `movel_test`.
`external_trigger.cpp` is commented out in CMakeLists.txt — uncomment to build it.
Do NOT modify `package.xml` to declare these as dependencies.

Run: `ros2 run realman_arm <executable>` (after `source install/setup.bash`).

## C++ standard

All packages use **C++23**. The ROS2 Humble base image ships GCC 11.4 which
has partial C++23 support — avoid features that require GCC 12+ (e.g.,
`std::expected`, `std::ranges::to`).

The calibration package includes polyfills (`expected_polyfill.hpp`,
`format_polyfill.hpp`) for this reason.

## cmake/ directory (workspace root)

Shared CMake modules. `clang_tidy.cmake` is included by packages via:
```cmake
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/clang_tidy.cmake OPTIONAL)
```
This is opt-in — pass `-DCLANG_TIDY=ON` to colcon build.

## Formatting & linting

The workspace root has **`.clang-format`** (Google-based, 4-space indent, 100col
limit, `BreakBeforeBraces: Attach`, `PointerAlignment: Left`, includes grouped:
project → system → stdlib → ROS2 → other) and **`.clang-tidy`** (C++23 target,
GCC 11.4 toolchain, with naming conventions: `CamelCase` classes, `camelBack`
functions/methods, `UPPER_CASE` constants/enums, `lower_case` namespaces).

`clang-tidy` checks are disabled by default — pass `-DCLANG_TIDY=ON` to enable.

There is no CI pipeline and no pre-commit hooks.

## SDK submodule

```bash
git submodule update --init --recursive   # after clone
```

The submodule URL is `git@github.com:RealManRobot/RM_API2.git` (SSH). If a user
lacks SSH keys, switch to HTTPS in `.gitmodules`.

The C SDK `.so` is versioned at `third_party/RM_API2/C/linux/linux_x86_c_vv1.1.5/libapi_c.so`.
The cmake config discovers it via glob on `linux/linux_x86_c_vv*/libapi_c.so`.
When updating the submodule, check if the versioned path changed.

## Testing

Tests are gated by `BUILD_TESTING` (`ament_cmake_gtest`). Build with:
```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test
```

Test binaries need `LD_LIBRARY_PATH` pointing to the SDK lib — CMake config
handles this via `APPEND_ENV`.

`realman_calibration` has the most comprehensive test suite (camera calibration,
pose processing, hand-eye, TF integration, synthetic data generators).

## Docker & deployment

Two-container architecture:

- **Develop** (`realman-develop`): GUI, build tools, workspace mounted at `/ws`
- **Runtime** (`realman-runtime`): Headless, runs on robot MiniPC with
  supervisor auto-starting `controller_manager` + `calib_node`

Deploy workflow (from develop container):
```bash
sync-remote <robot-ip>      # rsync install/ to runtime container over SSH port 2022
deploy-remote <robot-ip>    # sync + supervisor restart
ssh-remote <robot-ip>       # SSH into runtime
```

The runtime container expects built artifacts at `/ws/install`. Supervisor
config at `scripts/supervisord.conf` sources both `/opt/ros/humble/setup.bash`
and `/ws/install/setup.bash` before launching nodes.

Proxy config for apt/pip behind a local proxy: copy `.env.example` to `.env` and
customize. `docker compose` reads proxy vars from `.env`.

## Dependencies not in workspace

`realman_calibration` depends on both `realman_vision` and `realman_arm` — these
**are** workspace packages (`src/realman_vision/`, `src/realman_arm/`), not
external dependencies. They must be built before the calibration package.

## CodeGraph

This workspace is indexed with CodeGraph (`.codegraph/`). Use `codegraph_*`
tools for structural queries — symbol lookup, callers/callees, impact analysis.
The index lags file writes by ~500ms; don't query immediately after editing.
