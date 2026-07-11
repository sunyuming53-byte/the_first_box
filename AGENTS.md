# AGENTS.md — RealMan ROS2 Workspace

## Pre-push gate

**Code MUST pass the full Docker-based build + test + lint cycle before it is pushed.**

CI uses `docker run --rm` (see `.github/workflows/ci.yml`). The Docker image
contains the **exact** toolchain versions (clang-format, OpenCV, PCL, GCC) that
CI uses. Local tool versions differ — local-only checks are NOT valid.

Verify locally with:

```bash
# 1. Rebuild image if dependencies changed
docker compose build develop

# 2. Build and test (match CI exactly)
docker run --rm --user root -v $(pwd):/ws realman:develop bash -c '
  set -euo pipefail
  set +u; source /opt/ros/humble/setup.bash; set -u
  colcon build --symlink-install
  colcon build --cmake-args -DBUILD_TESTING=ON
  colcon test --return-code-on-test-failure
'

# 3. clang-format (blocking — MUST use Docker, NOT local clang-format)
#    Check only changed files against base branch
CHANGED=$(git diff --name-only --diff-filter=ACMRT origin/main...HEAD -- '**.cpp' '**.hpp' '**.h' | grep -v third_party/ || true)
if [ -n "$CHANGED" ]; then
  echo "$CHANGED" | sed 's|^|/ws/|' | \
    docker run --rm -i -v $(pwd):/ws realman:develop bash -c '
      xargs -r clang-format --dry-run --Werror
    '
fi
```

> In Docker, tests need `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (the default
> `rmw_fastrtps_cpp` requires shared memory not available in containers).
> The Dockerfile pre-sets this.

No commit may be pushed if the Docker build, test, or clang-format check fails. If a test
genuinely cannot run in Docker (e.g., requires live robot hardware), it
must be skipped explicitly with a documented reason — never simply
commented out or disabled without explanation.

**NEVER run build, test, or format checks on the host machine.** The Docker
image is the single source of truth for toolchain versions. Host GCC,
clang-format, OpenCV, and PCL versions differ and will produce false
positives or false negatives. If you cannot run Docker, do not push.

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
`find_package(RealManSDK REQUIRED)` backed by `src/omr_hardware/third_party/realman_arm/cmake/RealManSDKConfig.cmake`.
The Dockerfile copies the SDK from the submodule to `/opt/realman-sdk/` at
build time.

**Build order matters**: `omr_vision` (ament_cmake, no ROS deps) →
`omr_hardware` (ament_cmake, embeds submodules) →
`omr_controller` (depends on omr_hardware + omr_vision) →
`omr_lio` (ament_cmake, LiDAR SLAM + Nav2) →
`omr_bringup` (launch/config only, depends on all).
`colcon build` handles this automatically.

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
| `omr_vision` | ament_cmake | RealSense D435 capture (OpenCV + librealsense2). No ROS deps. |
| `omr_hardware` | ament_cmake | ros2_control hardware interface plugins (ArmSystem, DaisHardware, M65BaseHardware). Embeds realman_arm, dais_motor, m65_chassis as submodules. |
| `omr_controller` | ament_cmake | Behavior tree-based task orchestrator (BT.CPP v4). Clients: Arm, Gripper, Motor, Base, Vision. Note: `motor_client` and `base_client` are created but NOT registered on BT blackboard — no BT action node can currently command them. Door trajectory math model with MoveIt2 collision-aware planning. Hand-eye calibration pipeline. |
| `rm65_moveit_config` | ament_cmake | MoveIt2 config for RM65 (SRDF, KDL kinematics, OMPL, geometric collision primitives). No compiled code. |
| `omr_lio` | ament_cmake | S-FAST_LIO LiDAR-IMU SLAM + Nav2 navigation. LioNode (ESKF + ikd-Tree), EstopperNode (0.3m emergency stop), InspectionSequencer (YAML waypoints with task event handshake). Waypoint recording tools. Depends on Livox Mid-360 + built-in IMU. |
| `omr_bringup` | ament_cmake | Launch files + config + URDF only. No compiled code. |

**Hardware interface plugins** (all in `omr_hardware/plugins.xml`):
- `ArmSystem` — wraps `rm::Arm` (RealMan arm via TCP)
- `DaisHardware` — wraps `dais::Motor` (D-AIS motor via Modbus RTU)
- `M65BaseHardware` — wraps `m65::Chassis` (M65 mobile base via serial)

**Submodules** (`src/omr_hardware/third_party/`):

| Path | Repo | Purpose |
|---|---|---|
| `realman_arm/` | ssh://git@github.com/ChiefTechLabs/realman_arm.git | `rm::Arm` — pure C++ arm control (zero ROS deps) |
| `dais_motor/` | ssh://git@github.com/ChiefTechLabs/dais_motor.git | `dais::Motor` — pure C++ Modbus RTU driver (zero ROS deps) |
| `m65_chassis/` | ssh://git@github.com/ChiefTechLabs/m65_chassis.git | `m65::Chassis` — pure C++ serial chassis driver (zero ROS deps) |

**All submodules are plain CMake projects (NOT `ament_cmake`) with zero ROS
dependencies.** Do NOT add `ament_cmake`, `rclcpp`, or any ROS dependency to them.

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
  starts `ros2_control_node` (via `controller_manager`).
- `dais::Motor` and `m65::Chassis` follow the same pattern: pure C++, zero ROS,
  wrapped by their respective hardware interface plugins in `omr_hardware`.

### Source layout (realman_arm)

```
src/omr_hardware/third_party/realman_arm/
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

## C++ standard

All packages use **C++23**. The ROS2 Humble base image ships GCC 11.4 which
has partial C++23 support — avoid features that require GCC 12+ (e.g.,
`std::expected`, `std::ranges::to`).

The former `realman_calibration` package included polyfills (`expected_polyfill.hpp`,
`format_polyfill.hpp`) for this reason; these remain in the migrated packages.

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

CI runs `clang-format --dry-run --Werror` (blocking, diff-only) and clang-tidy
(blocking, diff-only — checks only changed files against compile_commands.json).
No pre-commit hooks configured.

**IMPORTANT — Always run clang-format inside Docker.** The Docker image uses
clang-format 19; local host versions (14 on Ubuntu 22.04) apply different
formatting rules. A file that passes local `clang-format --dry-run` may fail
in CI. See Pre-push gate above for the exact command.

## Submodules

```bash
git submodule update --init --recursive   # after clone
```

All submodules use SSH URLs. If SSH keys are unavailable, temporarily switch
to HTTPS in `.gitmodules`.

**RealMan C SDK** is nested: `realman_arm/third_party/RM_API2/` (SSH:
`git@github.com:RealManRobot/RM_API2.git`). The `.so` is versioned at
`third_party/RM_API2/C/linux/linux_x86_c_vv1.1.5/libapi_c.so`. The cmake
config discovers it via glob on `linux/linux_x86_c_vv*/libapi_c.so`.
When updating the submodule, check if the versioned path changed and update
`COPY` commands in the Dockerfile.

## Testing

Tests are gated by `BUILD_TESTING` (`ament_cmake_gtest`). Build with:
```bash
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test
```

To run tests for a single package:
```bash
colcon test --packages-select omr_controller --event-handlers console_direct+
```

Test binaries need `LD_LIBRARY_PATH` pointing to the SDK lib — CMake config
handles this via `APPEND_ENV`.

`omr_controller` and `omr_vision` share what was formerly `realman_calibration`'s
comprehensive test suite (camera calibration, pose processing, hand-eye, TF
integration, synthetic data generators), alongside `omr_controller`'s own 22
test files (~20 test binaries) covering door math, collision, BT factory,
clients, geometry utils, and orchestrator.

### Docker test gotcha

The default `rmw_fastrtps_cpp` middleware requires shared memory (not available
in Docker). Tests in Docker fail with obscure errors unless you set:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
The Dockerfile pre-configures this, but if you see `RMW` errors during local
Docker testing, check this first.

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

Proxy config for apt/pip behind a local proxy: copy `.env.example` to `.env` and
customize. `docker compose` reads proxy vars from `.env`.

## Dependencies not in workspace

**When adding a new ROS2 `<depend>` in any `package.xml`**: also add the
corresponding `ros-humble-*` apt package to the `RUN apt-get install` blocks in
the Dockerfile (both `realman-base-dev` and `realman-base` stages). The
Dockerfile uses explicit `apt-get install` instead of `rosdep` because
`rosdep update` fails in GitHub Actions CI (DNS cannot resolve
`raw.githubusercontent.com` from Docker build containers).

## CodeGraph

This workspace is indexed with CodeGraph (`.codegraph/`). Use `codegraph_*`
tools for structural queries — symbol lookup, callers/callees, impact analysis.
The index lags file writes by ~500ms; don't query immediately after editing.

## Git worktrees

This repo uses git worktrees for parallel feature branches:
```
main workspace:   /home/ubuntu/.ws/pipeline             (main)
gimbal feature:   /home/ubuntu/.ws/pipeline-gimbal      (feat/gimbal-kinematics)
m65 feature:      /home/ubuntu/.ws/pipeline-m65-chassis  (feat/m65-chassis)
```

**When working in a worktree, ensure file operations target the correct
workspace path.** A session in `pipeline-gimbal` accidentally writing to
`pipeline` is a known failure mode — check `pwd` before editing.

The `.worktrees/` directory (bare repo metadata) is git-ignored.
Worktree paths above are conventions — verify with `git worktree list`.
`.env` is also git-ignored (developer-specific proxy/ROS config).
