# AGENTS.md — RealMan ROS2 Workspace

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build
```

Override SDK path: `colcon build --cmake-args -DREALMAN_SDK=/path/to/RM_API2/C`

The CMakeLists.txt auto-discovers `libapi_c.so` via `file(GLOB ...)` inside
`third_party/RM_API2/C/linux/`. If the SDK version changes and the glob path
breaks, update the glob pattern in `CMakeLists.txt`.

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

## Examples are disabled in CMakeLists.txt

The example executables (`hello_arm`, `external_trigger`, `arm_node`) are commented
out. `arm_node.cpp` (the implementation) is also commented out. Uncomment in
`CMakeLists.txt` to build them. Do NOT modify `package.xml` to declare these as
dependencies unless they are actually built.

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
