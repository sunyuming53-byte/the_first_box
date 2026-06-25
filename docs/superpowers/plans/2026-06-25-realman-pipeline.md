# RealMan Pipeline — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reusable C++ library (`realman-pipeline`) that wraps RM_API2 into a ROS2-friendly `rm::Arm` class with external signal support.

**Architecture:** `rm::Arm` (PIMPL, non-Node) wraps RM_API2 C calls in a worker thread with a poll loop. Internal `rclcpp::Node` publishes state topics. `rm::ArmNode` (rclcpp::Node) holds `Arm` and exposes services. All deps: C++17, RM_API2 headers+lib, rclcpp.

**Tech Stack:** C++17, CMake, RM_API2 (libRM_Service.so, rm_interface.h), ROS2 Humble (rclcpp), std::thread + std::mutex

**Dependency note:** RM_API2 ships inside `ros2_rm_robot/rm_driver/`. Plan assumes `$REALMAN_SDK` env var or CMake cache variable pointing to the SDK root containing `include/rm_interface.h` and `lib/libRM_Service.so`.

---

## File Structure Map

| File | Responsibility |
|------|---------------|
| `include/realman/types.hpp` | All data structs, enums, ArmConfig |
| `include/realman/error.hpp` | `rm::ArmError` exception class |
| `include/realman/arm.hpp` | `rm::Arm` public API + PIMPL |
| `include/realman/arm_node.hpp` | `rm::ArmNode` (rclcpp::Node) |
| `src/arm.cpp` | Arm::Impl — worker thread, C API calls, ROS2 node |
| `src/arm_node.cpp` | ArmNode implementation, service callbacks |
| `src/error.cpp` | ArmError implementation |
| `CMakeLists.txt` | Build: shared lib `realman-pipeline` + examples |
| `examples/hello_arm.cpp` | Minimal: connect, moveJ, disconnect |
| `examples/external_trigger.cpp` | External ROS2 topic triggers arm motion |
| `examples/arm_node.cpp` | Spin ArmNode, arm controllable via service |

---

### Task 1: Project Scaffold

**Files:**
- Create: `CMakeLists.txt`

- [ ] **Step 1: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(realman-pipeline VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# RealMan SDK path — set via -DREALMAN_SDK=/path/to/sdk or env
if(NOT DEFINED REALMAN_SDK)
    set(REALMAN_SDK "$ENV{REALMAN_SDK}")
endif()
if(NOT REALMAN_SDK)
    message(FATAL_ERROR "REALMAN_SDK not set. Pass -DREALMAN_SDK=/path/to/RM_API2")
endif()

# RealMan SDK
add_library(rm_service SHARED IMPORTED)
set_target_properties(rm_service PROPERTIES
    IMPORTED_LOCATION "${REALMAN_SDK}/lib/libRM_Service.so"
    INTERFACE_INCLUDE_DIRECTORIES "${REALMAN_SDK}/include"
)

# Pipeline library
add_library(realman-pipeline SHARED
    src/arm.cpp
    src/arm_node.cpp
    src/error.cpp
)
target_include_directories(realman-pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(realman-pipeline PUBLIC
    rclcpp::rclcpp
    rm_service
    pthread
)
ament_target_dependencies(realman-pipeline rclcpp)

# Examples
add_executable(hello_arm examples/hello_arm.cpp)
target_link_libraries(hello_arm realman-pipeline)

add_executable(external_trigger examples/external_trigger.cpp)
target_link_libraries(external_trigger realman-pipeline)

add_executable(arm_node examples/arm_node.cpp)
target_link_libraries(arm_node realman-pipeline)

install(TARGETS realman-pipeline
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
```

- [ ] **Step 2: Create package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>realman-pipeline</name>
  <version>0.1.0</version>
  <description>ROS2 C++ pipeline library for RealMan robot arms</description>
  <maintainer email="dev@realman-pipeline">RealMan Pipeline Team</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Verify CMake configures**

```bash
cd /home/ubuntu/.ws/realman/realman-pipeline
mkdir -p build && cd build
REALMAN_SDK=/path/to/your/RM_API2 cmake .. 2>&1
```
Expected: cmake succeeds without errors

---

### Task 2: Data Types

**Files:**
- Create: `include/realman/types.hpp`

- [ ] **Step 1: Write types.hpp**

```cpp
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <array>

namespace rm {

enum class ArmModel {
    RM_65,
    RM_75,
    ECO65,
    ECO63,
    RML_63,
    RML_63_III,
    GEN_72,
    GEN_72_II,
};

struct JointPosition {
    std::vector<double> radians;
    explicit JointPosition(std::vector<double> r) : radians(std::move(r)) {}
    JointPosition() = default;
};

struct CartesianPose {
    double x{0}, y{0}, z{0};           // meters
    double roll{0}, pitch{0}, yaw{0};  // radians (Euler RPY)
};

using SpeedRatio = uint8_t;  // 0–100

struct ArmState {
    JointPosition                 joint_position;
    CartesianPose                 tool_pose;
    std::array<double, 6>         joint_current{};
    std::array<double, 6>         joint_temperature{};
    bool                          is_moving{false};
    int                           error_code{0};
    std::string                   error_message;
};

struct ArmConfig {
    std::string ip{"192.168.1.18"};
    int         tcp_port{8080};
    ArmModel    model{ArmModel::RM_65};
    int         dof{6};

    // UDP 主动上报
    std::string udp_ip{"192.168.1.10"};
    int         udp_port{8089};
    int         udp_cycle{5};
    int         udp_force_coordinate{0};
    bool        udp_joint_speed{true};
    bool        udp_arm_current_status{false};
    bool        udp_lift_state{false};
    bool        udp_expand_state{false};
    bool        udp_hand{false};
    bool        udp_aloha{false};

    // 轨迹跟随
    int         trajectory_mode{0};  // 0=透传 1=拟合 2=滤波
    int         radio{0};
};

} // namespace rm
```

- [ ] **Step 2: Verify compilation**

```bash
echo '#include "realman/types.hpp"\nint main() { rm::JointPosition j; return 0; }' > /tmp/test_types.cpp
g++ -std=c++17 -I include -c /tmp/test_types.cpp -o /tmp/test_types.o
```
Expected: compiles without errors

---

### Task 3: Error Handling

**Files:**
- Create: `include/realman/error.hpp`
- Create: `src/error.cpp`

- [ ] **Step 1: Write error.hpp**

```cpp
#pragma once
#include <stdexcept>
#include <string>

namespace rm {

class ArmError : public std::runtime_error {
public:
    explicit ArmError(int code, const std::string& message)
        : std::runtime_error(message), code_(code) {}

    int code() const noexcept { return code_; }

private:
    int code_;
};

} // namespace rm
```

- [ ] **Step 2: Write error.cpp**

```cpp
#include "realman/error.hpp"
// ArmError is fully inline in header; cpp exists for future non-inline additions.
```

- [ ] **Step 3: Verify compilation**

```bash
echo '#include "realman/error.hpp"\nint main() { rm::ArmError e(0x1001, "joint error"); return e.code(); }' > /tmp/test_error.cpp
g++ -std=c++17 -I include -c src/error.cpp /tmp/test_error.cpp -o /tmp/test_error 2>&1
```
Expected: compiles without errors

---

### Task 4: Arm Header

**Files:**
- Create: `include/realman/arm.hpp`

- [ ] **Step 1: Write arm.hpp declaration**

```cpp
#pragma once
#include "realman/types.hpp"
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <array>

namespace rm {

class Arm {
public:
    using MotionCallback = std::function<void(bool success)>;

    explicit Arm(const ArmConfig& config);
    ~Arm();

    // Non-copyable, movable
    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;
    Arm(Arm&&) = default;
    Arm& operator=(Arm&&) = default;

    // ── Motion ──
    void moveJ(const JointPosition& target, SpeedRatio speed = 50,
               bool blocking = true, int trajectory_connect = 0);
    void moveJ_P(const CartesianPose& target, SpeedRatio speed = 50,
                 bool blocking = true, int trajectory_connect = 0);
    void moveL(const CartesianPose& target, SpeedRatio speed = 50,
               bool blocking = true, int trajectory_connect = 0);
    void moveC(const CartesianPose& mid, const CartesianPose& end,
               SpeedRatio speed = 50, int loop = 1, bool blocking = true);
    void stop();

    // ── CANFD ──
    void moveJ_CANFD(const JointPosition& target, int mode = 0);
    void moveP_CANFD(const CartesianPose& target, int mode = 0);

    // ── State (read cached, non-blocking) ──
    JointPosition jointPosition() const;
    CartesianPose  toolPose() const;
    ArmState       state() const;

    // ── Frames ──
    std::vector<std::string> getWorkFrames();
    void setWorkFrame(const std::string& name);

    // ── Gripper ──
    void gripper(int position, SpeedRatio speed = 50, bool blocking = true);

    // ── Force control (requires 6-axis force sensor) ──
    void enableForceControl(const std::array<double, 6>& params);
    void disableForceControl();

    // ── Callback ──
    void onMotionComplete(MotionCallback cb);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace rm
```

- [ ] **Step 2: Verify compilation**

```bash
echo '#include "realman/arm.hpp"\nint main() { return 0; }' > /tmp/test_arm_header.cpp
g++ -std=c++17 -I include -c /tmp/test_arm_header.cpp -o /tmp/test_arm_header.o 2>&1
```
Expected: compiles without errors (PIMPL means no RM_API2 headers needed for this test)

---

### Task 5: Arm Implementation — Worker Thread + C API + ROS2 Node

**Files:**
- Create: `src/arm.cpp`

This is the core of the library. The `Impl` class runs RM_API2 calls in a worker thread.

- [ ] **Step 1: Write arm.cpp — Impl structure**

```cpp
#include "realman/arm.hpp"
#include "realman/error.hpp"
#include <rm_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <cstring>

namespace rm {

// ============================================================
// Wire C return codes → ArmError
// ============================================================
static void check(int ret, const char* operation) {
    if (ret != 0) {
        throw ArmError(ret, std::string(operation) + " failed (code " + std::to_string(ret) + ")");
    }
}

// ============================================================
// Command types for the worker thread queue
// ============================================================
enum class CmdType { MoveJ, MoveJP, MoveL, MoveC, Stop, Gripper,
                     ForceOn, ForceOff, MoveJ_CANFD, MoveP_CANFD };

struct Cmd {
    CmdType type;
    union {
        struct { float j[7]; int dof; int speed; int traj_connect; } movej;
        struct { float x, y, z, roll, pitch, yaw; int speed; int traj_connect; } movep;
        struct { float x, y, z, roll, pitch, yaw; int speed; int traj_connect; } movel;
        struct { float mx,my,mz,mroll,mpitch,myaw, ex,ey,ez,eroll,epitch,eyaw; int speed; int loop; } movec;
        struct { int position; int speed; } gripper;
        struct { float p[6]; } force_params;
        struct { float j[7]; int mode; } canfd;
    };
    bool blocking{true};
};

// ============================================================
// Impl
// ============================================================
class Arm::Impl {
public:
    explicit Impl(const ArmConfig& config);
    ~Impl();

    // Public API (called from user thread)
    void moveJ(const JointPosition& target, SpeedRatio speed, bool blocking, int traj_connect);
    void moveJP(const CartesianPose& target, SpeedRatio speed, bool blocking, int traj_connect);
    void moveL(const CartesianPose& target, SpeedRatio speed, bool blocking, int traj_connect);
    void moveC(const CartesianPose& mid, const CartesianPose& end, SpeedRatio speed, int loop, bool blocking);
    void stop();
    void gripper(int position, SpeedRatio speed, bool blocking);
    void enableForce(const std::array<double,6>& p);
    void disableForce();

    JointPosition jointPosition() const;
    CartesianPose  toolPose() const;
    ArmState       state() const;

    std::vector<std::string> getWorkFrames();
    void setWorkFrame(const std::string& name);

    void onMotionComplete(Arm::MotionCallback cb);

private:
    void workerLoop();
    void executeCmd(const Cmd& cmd);
    void pollState();

    // RM_API2
    rm_robot_handle* handle_{nullptr};

    // Worker thread
    std::thread worker_;
    std::atomic<bool> running_{true};
    std::mutex cmd_mutex_;
    std::condition_variable cmd_cv_;
    std::queue<Cmd> cmd_queue_;

    // State cache (protected by state_mutex_)
    mutable std::mutex state_mutex_;
    JointPosition joint_pos_;
    CartesianPose  tool_pose_;
    ArmState       arm_state_;

    // Motion done signaling
    std::mutex done_mutex_;
    std::condition_variable done_cv_;
    std::atomic<bool> motion_done_{true};
    std::atomic<bool> last_motion_ok_{true};

    // Callback
    std::mutex cb_mutex_;
    Arm::MotionCallback motion_cb_;
};
```

- [ ] **Step 2: Write arm.cpp — Constructor**

```cpp
Arm::Impl::Impl(const ArmConfig& config) {
    // Initialize RM SDK
    rm_set_log_call_back(nullptr, 3);
    int ret = rm_init(RM_TRIPLE_MODE_E);
    check(ret, "rm_init");

    // Map ArmModel → C API arm_type enum
    int c_model = 65; // default
    switch (config.model) {
        case ArmModel::RM_65:  c_model = 65; break;
        case ArmModel::RM_75:  c_model = 75; break;
        case ArmModel::ECO65:  c_model = 651; break;
        case ArmModel::ECO63:  c_model = 634; break;
        case ArmModel::RML_63: case ArmModel::RML_63_III: c_model = 632; break;
        case ArmModel::GEN_72: case ArmModel::GEN_72_II: c_model = 72; break;
    }

    // Connect
    char ip_cstr[32];
    std::strncpy(ip_cstr, config.ip.c_str(), sizeof(ip_cstr)-1);
    ip_cstr[sizeof(ip_cstr)-1] = '\0';
    handle_ = rm_create_robot_arm(ip_cstr, config.tcp_port);
    if (!handle_ || handle_->id == -1) {
        if (handle_) rm_delete_robot_arm(handle_);
        throw ArmError(-1, "Failed to connect to arm at " + config.ip);
    }

    // Configure arm type
    rm_set_arm_run_mode(handle_, c_model);

    // Configure UDP push
    rm_realtime_push_config_t push_cfg{};
    push_cfg.cycle = config.udp_cycle;
    push_cfg.port = config.udp_port;
    push_cfg.enable = 1;
    // ... set individual push flags (udp_joint_speed, etc.)
    rm_set_realtime_push(handle_, &push_cfg);

    // Start worker thread
    worker_ = std::thread(&Impl::workerLoop, this);
}
```

- [ ] **Step 3: Write arm.cpp — Worker loop**

```cpp
void Arm::Impl::workerLoop() {
    while (running_) {
        std::unique_lock<std::mutex> lock(cmd_mutex_);
        cmd_cv_.wait(lock, [this] { return !cmd_queue_.empty() || !running_; });

        while (!cmd_queue_.empty()) {
            Cmd cmd = cmd_queue_.front();
            cmd_queue_.pop();
            lock.unlock();

            executeCmd(cmd);

            lock.lock();
        }
    }
}

void Arm::Impl::executeCmd(const Cmd& cmd) {
    try {
        int ret = 0;
        switch (cmd.type) {
            case CmdType::MoveJ: {
                float j[7] = {};
                for (int i = 0; i < cmd.movej.dof; ++i) j[i] = cmd.movej.j[i];
                ret = rm_movej(handle_, j, cmd.movej.dof, cmd.movej.speed,
                               cmd.blocking ? 1 : 0, cmd.movej.traj_connect);
                break;
            }
            case CmdType::MoveJP: {
                float pose[6] = {cmd.movep.x, cmd.movep.y, cmd.movep.z,
                                 cmd.movep.roll, cmd.movep.pitch, cmd.movep.yaw};
                ret = rm_movej_p(handle_, pose, cmd.movep.speed,
                                 cmd.blocking ? 1 : 0, cmd.movep.traj_connect);
                break;
            }
            case CmdType::MoveL: {
                float pose[6] = {cmd.movel.x, cmd.movel.y, cmd.movel.z,
                                 cmd.movel.roll, cmd.movel.pitch, cmd.movel.yaw};
                ret = rm_movel(handle_, pose, cmd.movel.speed,
                               cmd.blocking ? 1 : 0, cmd.movel.traj_connect);
                break;
            }
            case CmdType::MoveC: {
                float mid[6] = {cmd.movec.mx, cmd.movec.my, cmd.movec.mz,
                                cmd.movec.mroll, cmd.movec.mpitch, cmd.movec.myaw};
                float end[6] = {cmd.movec.ex, cmd.movec.ey, cmd.movec.ez,
                                cmd.movec.eroll, cmd.movec.epitch, cmd.movec.eyaw};
                ret = rm_movec(handle_, mid, end, cmd.movec.speed,
                               cmd.movec.loop, cmd.blocking ? 1 : 0);
                break;
            }
            case CmdType::Stop:
                ret = rm_stop(handle_);
                break;
            case CmdType::Gripper:
                ret = rm_set_gripper_position(handle_, cmd.gripper.position,
                                              cmd.gripper.speed, cmd.blocking ? 1 : 0);
                break;
            default:
                break;
        }

        check(ret, "arm command");

        std::lock_guard<std::mutex> lock(done_mutex_);
        last_motion_ok_ = true;
        motion_done_ = true;
        done_cv_.notify_all();

        if (cmd.blocking && motion_cb_) {
            motion_cb_(true);
        }
    } catch (const ArmError& e) {
        std::lock_guard<std::mutex> lock(done_mutex_);
        last_motion_ok_ = false;
        motion_done_ = true;
        done_cv_.notify_all();

        if (cmd.blocking && motion_cb_) {
            motion_cb_(false);
        }
        throw;
    }
}
```

- [ ] **Step 4: Write arm.cpp — Public API delegations**

```cpp
Arm::Arm(const ArmConfig& config) : impl_(std::make_unique<Impl>(config)) {}
Arm::~Arm() = default;

void Arm::moveJ(const JointPosition& target, SpeedRatio speed, bool blocking, int tc) {
    impl_->moveJ(target, speed, blocking, tc);
}
void Arm::moveJ_P(const CartesianPose& target, SpeedRatio speed, bool blocking, int tc) {
    impl_->moveJP(target, speed, blocking, tc);
}
void Arm::moveL(const CartesianPose& target, SpeedRatio speed, bool blocking, int tc) {
    impl_->moveL(target, speed, blocking, tc);
}
void Arm::moveC(const CartesianPose& mid, const CartesianPose& end,
                SpeedRatio speed, int loop, bool blocking) {
    impl_->moveC(mid, end, speed, loop, blocking);
}
void Arm::stop() { impl_->stop(); }
void Arm::gripper(int pos, SpeedRatio speed, bool blocking) { impl_->gripper(pos, speed, blocking); }

JointPosition Arm::jointPosition() const { return impl_->jointPosition(); }
CartesianPose  Arm::toolPose() const     { return impl_->toolPose(); }
ArmState       Arm::state() const        { return impl_->state(); }

void Arm::enableForceControl(const std::array<double,6>& p) { impl_->enableForce(p); }
void Arm::disableForceControl() { impl_->disableForce(); }
void Arm::onMotionComplete(MotionCallback cb) { impl_->onMotionComplete(std::move(cb)); }
```

---

### Task 6: ArmNode

**Files:**
- Create: `include/realman/arm_node.hpp`
- Create: `src/arm_node.cpp`

- [ ] **Step 1: Write arm_node.hpp**

```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "realman/arm.hpp"

namespace rm {

class ArmNode : public rclcpp::Node {
public:
    explicit ArmNode(const rclcpp::NodeOptions& options, const ArmConfig& config);

    Arm& arm() { return arm_; }

private:
    void setupServices();

    Arm arm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_svc_;
};

} // namespace rm
```

- [ ] **Step 2: Write arm_node.cpp**

```cpp
#include "realman/arm_node.hpp"
#include <std_srvs/srv/trigger.hpp>

namespace rm {

ArmNode::ArmNode(const rclcpp::NodeOptions& options, const ArmConfig& config)
    : rclcpp::Node("arm_node", options), arm_(config)
{
    setupServices();
}

void ArmNode::setupServices() {
    using Trigger = std_srvs::srv::Trigger;

    stop_svc_ = this->create_service<Trigger>(
        "~/stop",
        [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
            try {
                arm_.stop();
                res->success = true;
                res->message = "stopped";
            } catch (const ArmError& e) {
                res->success = false;
                res->message = e.what();
            }
        });
}

} // namespace rm
```

---

### Task 7: Example — hello_arm

**Files:**
- Create: `examples/hello_arm.cpp`

- [ ] **Step 1: Write hello_arm.cpp**

```cpp
#include "realman/arm.hpp"
#include <iostream>
#include <vector>

int main() {
    rm::ArmConfig cfg;
    cfg.ip = "192.168.1.18";
    cfg.model = rm::ArmModel::RM_65;
    cfg.dof = 6;

    try {
        rm::Arm arm(cfg);
        std::cout << "Connected to arm" << std::endl;

        // Move joint 1 to 0.5 rad, others stay at 0
        rm::JointPosition target(std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.0});
        arm.moveJ(target, 30);
        std::cout << "MoveJ complete" << std::endl;

        auto state = arm.state();
        std::cout << "Joint 1 position: " << state.joint_position.radians[0] << " rad" << std::endl;

    } catch (const rm::ArmError& e) {
        std::cerr << "Error: " << e.what() << " (code " << e.code() << ")" << std::endl;
        return 1;
    }

    return 0;
}
```

---

### Task 8: Example — external_trigger

**Files:**
- Create: `examples/external_trigger.cpp`

- [ ] **Step 1: Write external_trigger.cpp**

```cpp
#include "realman/arm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

class ExternalTriggerNode : public rclcpp::Node {
public:
    ExternalTriggerNode()
        : Node("external_trigger"), arm_(rm::ArmConfig{})
    {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/arm_trigger", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                try {
                    if (msg->data == "home") {
                        rm::JointPosition home(
                            std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
                        arm_.moveJ(home, 30);
                        RCLCPP_INFO(this->get_logger(), "Homed");
                    } else if (msg->data == "grip") {
                        arm_.gripper(100, 50);
                        RCLCPP_INFO(this->get_logger(), "Gripped");
                    }
                } catch (const rm::ArmError& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
                }
            });
    }

private:
    rm::Arm arm_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExternalTriggerNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

### Task 9: Example — ArmNode standalone

**Files:**
- Create: `examples/arm_node.cpp`

- [ ] **Step 1: Write arm_node.cpp (executable)**

```cpp
#include "realman/arm_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rm::ArmConfig cfg;
    // Override from ROS params if available
    auto node_opts = rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rm::ArmNode>(node_opts, cfg);
    RCLCPP_INFO(node->get_logger(), "ArmNode ready");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

---

### Task 10: README

**Files:**
- Create: `README.md`

- [ ] **Step 1: Write README.md**

```markdown
# RealMan Pipeline

C++17 / ROS2 Humble library for RealMan robot arm control.

## Quick Start

```cpp
#include "realman/arm.hpp"

rm::ArmConfig cfg;
cfg.ip = "192.168.1.18";
cfg.model = rm::ArmModel::RM_65;

rm::Arm arm(cfg);
arm.moveJ(rm::JointPosition({0.5, 0, 0, 0, 0, 0}), 30);
```

## Build

```bash
export REALMAN_SDK=/path/to/RM_API2   # contains include/ + lib/
mkdir build && cd build
cmake .. -DREALMAN_SDK=$REALMAN_SDK
make -j$(nproc)
```

## License

MIT
```
