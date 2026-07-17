# Issue #30 解决方案报告 — DAIS 电机弧度与导轨直线位移之间缺失丝杆导程转换

## 问题
`DaisHardware` 和 `dais::Motor` 读写的位置/速度单位是**电机轴弧度**（rad、rad/s），
但 URDF 中 `joint_dais` 是**棱柱关节**，期望单位为**米**（m、m/s）。
缺少丝杆导程（screw lead）的转换。

## 根因
D-AIS 电机通过丝杆将旋转运动转换为直线运动：
```
linear_displacement (m) = motor_radians × screw_lead (m) / (2π)
linear_velocity    (m/s) = motor_rad_per_s × screw_lead (m) / (2π)
```
导程 10mm (0.01m) 经机械团队确认。

## 修复方案

### 架构决策
- **转换层**: DaisHardware（ROS2 层），不在 dais::Motor 子模块
- **原因**: 保持纯 C++ 驱动零 ROS 依赖不变，遵循 M65BaseHardware.wheel_radius 模式
- **零子模块修改**

### 变更文件 (5 修改 + 2 新建)

| 文件 | 操作 | 行数 |
|------|------|------|
| `omr_hardware/include/omr_hardware/dais_hardware.hpp` | +1 | `double screw_lead_m_ = 0.01;` |
| `omr_hardware/src/dais_hardware.cpp` | 修改 | `on_init` + `read` + `write` |
| `omr_bringup/urdf/dais/dais.ros2_control.xacro` | +2 | arg + param |
| `omr_bringup/launch/bringup.launch.py` | +8/-4 | launch arg + 两处传参 |
| `omr_hardware/CMakeLists.txt` | +6 | BUILD_TESTING 门控 |
| `omr_hardware/test/test_dais_conversion.cpp` | 新建 | 115行, 15个TEST |
| `omr_hardware/test/CMakeLists.txt` | 新建 | 4行 |

### 转换公式 (10mm 导程)

#### read() — 状态上报
```cpp
hw_position_state_ = s.position_rad * screw_lead_m_ / (2.0 * M_PI);  // rad → m
hw_velocity_state_ = s.velocity_rpm * screw_lead_m_ / 60.0;           // rpm → m/s
```

#### write() — 指令下发
```cpp
motor_->set_velocity_command(hw_velocity_cmd_ * (2.0 * M_PI) / screw_lead_m_);  // m/s → rad/s
```

### 验证表

| 输入 | 电机侧 | 导轨侧 |
|------|--------|--------|
| 电机 1 转（2π rad） | 2π rad | 0.01 m |
| 600 RPM | 600 rpm | 0.1 m/s |
| 导轨 0.1 m/s 目标 | 62.83 rad/s | 0.1 m/s |

### 测试覆盖 (15 gtest 用例)

| 类别 | 测试数 | 覆盖 |
|------|--------|------|
| PositionConversion | 5 | 零值、一整圈、反转、半圈、5/10/20mm导程 |
| VelocityReadConversion | 4 | 零值、600RPM→0.1m/s、反向、多导程 |
| VelocityWriteConversion | 4 | 零值、0.1m/s→62.83rad/s、负向、多导程 |
| RoundTrip | 2 | 往返一致性、负向往返 |

### 不包含
- URDF velocity limit (2.0 m/s) — 需硬件实测后调整
- dais_controllers.yaml PID 重调 — 后置任务
- motor_client / BT blackboard — 不在本次范围

### 代码规范检查 ✅

| 规则 | 状态 | 说明 |
|------|------|------|
| 4-space indent | ✅ | 所有新增/修改行保持一致缩进 |
| 100 col limit | ✅ | 已修复 line 113 → 分行 (dais_hardware.cpp) |
| BreakBeforeBraces: Attach | ✅ | 无新增大括号，保持一致 |
| PointerAlignment: Left | ✅ | 无指针相关修改 |
| C++23 | ✅ | 符合 |
| 零子模块修改 | ✅ | dais_motor 零变更 |
| Python 语法 | ✅ | `py_compile` 通过 |
| 注释风格 | ✅ | `//` 单行注释 |

> 注: Docker clang-format 检查无法运行（镜像未构建且网络不可用），已通过手动逐行审查代替。
