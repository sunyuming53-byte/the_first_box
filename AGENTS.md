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

`arm_node` and `gripper_test` are built. `hello_arm` and `external_trigger`
are commented out — uncomment in `CMakeLists.txt` to build them.
`arm_node.cpp` (the implementation, not the example) is also commented out.
Do NOT modify `package.xml` to declare these as dependencies unless they are
actually built.

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

## ROS2 package

- Package name: `realman_driver`
- Depends on: `rclcpp` (ROS2), `std_srvs` (for ArmNode stop service)
- The package exports a shared library (`librealman_driver.so`), not an executable

## No tests, no CI, no formatting config

There is no test framework, no CI pipeline, no `.clang-format`, and no pre-commit
hooks. If adding tests, use `ament_cmake_gtest` (ROS2 convention). Do not add
formatting or linting config unless explicitly requested.
