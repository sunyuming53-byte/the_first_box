# RealMan Pipeline — 设计文档

> 2026-06-25 | C++ ROS2 Humble | 真机开发

## 1. 问题

RealMan 官方提供了 `RM_API2`（C SDK，TCP 直连机械臂）和 `ros2_rm_robot`（ROS2 驱动包），但两者之间缺一层：**ROS2 生态下可复用的 C++ 类库**。

当前开发者要用 ROS2 控制机械臂，必须每次手写 `create_publisher<Movej>`、管理 43 个消息类型、自己实现异步编排和错误处理。外部系统通过 ROS2 话题/服务/动作触发机械臂运动时，需要在业务逻辑中嵌入大量胶水代码。

## 2. 解决方案

提供一个轻量 C++ 库 `realman-pipeline`，包含两个类：

- **`rm::Arm`**（普通类，非 Node）：封装运动控制、状态查询、夹爪、力控。PIMPL 隐藏 ROS2 依赖。直接调用 RM_API2 的 C API，内部自行管理 ROS2 Node 和通信。
- **`rm::ArmNode`**（rclcpp::Node）：薄壳，持有 `Arm`，自动对外暴露 ROS2 service/action/topic，开箱即用。

## 3. 架构

```
用户代码层
  ┌─────────────────────────────────────────────────────┐
  │  用户自己的 rclcpp::Node                             │
  │  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
  │  │subscriber│  │  timer   │  │  业务逻辑          │  │
  │  │(外部信号)│  │(周期控制)│  │  arm.moveJ(...)   │  │
  │  └──────────┘  └──────────┘  └──────────────────┘  │
  │                    ↓                                │
  │              rm::Arm (普通类, PIMPL)                 │
  └─────────────────────────────────────────────────────┘
            ↓ C API
  ┌─────────────────────────────────────────────────────┐
  │              libRM_Service.so                       │
  └─────────────┬───────────────────────────────────────┘
                ↓ TCP
          真实机械臂控制器
```

层级依赖：`用户代码 → rm::Arm → RM_API2 (libRM_Service.so) → TCP → 控制器`

## 4. 数据类型 (types.hpp)

纯 C++ struct，不依赖 ROS2 头文件，SI 单位：

```cpp
namespace rm {

enum class ArmModel {
    RM_65, RM_75, ECO65, ECO63, RML_63, RML_63_III, GEN_72, GEN_72_II
};

struct JointPosition {
    std::vector<double> radians;
};

struct CartesianPose {
    double x, y, z;          // 米
    double roll, pitch, yaw; // 弧度
};

using SpeedRatio = uint8_t;  // 0-100

struct ArmState {
    JointPosition joint_position;
    CartesianPose tool_pose;
    std::array<double, 6> joint_current;       // A
    std::array<double, 6> joint_temperature;   // ℃
    bool  is_moving;
    int   error_code;
    std::string error_message;
};

struct ArmConfig {
    std::string ip{"192.168.1.18"};
    int tcp_port{8080};
    ArmModel model{ArmModel::RM_65};
    int dof{6};
    // UDP 主动上报
    std::string udp_ip{"192.168.1.10"};
    int udp_port{8089};
    int udp_cycle{5};
    int udp_force_coordinate{0};
    bool udp_joint_speed{true};
    bool udp_arm_current_status{false};
    bool udp_lift_state{false};
    bool udp_expand_state{false};
    bool udp_hand{false};
    bool udp_aloha{false};
    // 轨迹跟随
    int trajectory_mode{0};
    int radio{0};
};

} // namespace rm
```

## 5. rm::Arm 接口

```cpp
class Arm {
public:
    explicit Arm(const ArmConfig& config);
    ~Arm();

    // 运动控制
    void moveJ(const JointPosition& target, SpeedRatio speed = 50,
               bool blocking = true, int trajectory_connect = 0);
    void moveJ_P(const CartesianPose& target, SpeedRatio speed = 50,
                 bool blocking = true, int trajectory_connect = 0);
    void moveL(const CartesianPose& target, SpeedRatio speed = 50,
               bool blocking = true, int trajectory_connect = 0);
    void moveC(const CartesianPose& mid, const CartesianPose& end,
               SpeedRatio speed = 50, int loop = 1, bool blocking = true);
    void stop();

    // CANFD 透传
    void moveJ_CANFD(const JointPosition& target, int mode = 0);
    void moveP_CANFD(const CartesianPose& target, int mode = 0);

    // 状态（读缓存，非阻塞）
    JointPosition jointPosition() const;
    CartesianPose  toolPose() const;
    ArmState       state() const;

    // 坐标系
    std::vector<std::string> getWorkFrames();
    void setWorkFrame(const std::string& name);

    // 夹爪
    void gripper(int position, SpeedRatio speed = 50, bool blocking = true);

    // 力位混合控制（需六维力选配）
    void enableForceControl(const std::array<double, 6>& params);
    void disableForceControl();

    // 运动完成回调
    using MotionCallback = std::function<void(bool success)>;
    void onMotionComplete(MotionCallback cb);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
```

**设计决策**：
- PIMPL 隐藏 ROS2 依赖，用户头文件不含 rclcpp
- blocking 默认 true（新人友好的心智模型）
- 异常表示错误（`rm::ArmError`），不用返回值错误码
- state() 读缓存，不主动查询

## 6. rm::ArmNode 接口

```cpp
class ArmNode : public rclcpp::Node {
public:
    explicit ArmNode(const rclcpp::NodeOptions& options,
                     const ArmConfig& config);
    Arm& arm();

    // 自动暴露的 ROS2 接口:
    //   Service: ~/move_j, ~/move_l, ~/move_c, ~/stop, ~/gripper
    //   Topic:   ~/arm_state (20Hz)
};
```

## 7. 错误处理

```cpp
class ArmError : public std::runtime_error {
public:
    int code() const;           // 原始错误码
    const char* what() const;   // 错误描述
};
```

运动失败、连接超时、控制器报错均通过 `ArmError` 异常抛出。

## 8. 依赖

- C++17
- `libRM_Service.so`（来自 `ros2_rm_robot/rm_driver/lib/`）
- `rm_define.h` / `rm_interface.h`（来自 RM_API2）
- ROS2 Humble (rclcpp)

## 9. 项目结构

```
realman-pipeline/
├── CMakeLists.txt
├── include/realman/
│   ├── arm.hpp
│   ├── arm_node.hpp
│   ├── types.hpp
│   └── config.hpp
├── src/
│   ├── arm.cpp
│   ├── arm_node.cpp
│   └── error.cpp
├── examples/
│   ├── 01_hello_arm.cpp
│   ├── 02_external_trigger.cpp
│   └── 03_arm_node.cpp
└── README.md
```

## 10. 不在范围内（V1）

- 升降机控制
- Modbus RTU/TCP
- 灵巧手
- 电子围栏/虚拟墙
- 在线编程文件管理
- 多机械臂同步控制
- Gazebo 仿真集成
- Python 绑定
