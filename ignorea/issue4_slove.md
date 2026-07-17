# Issue #63 解决报告 — position_rad 反馈虚假 + 未实现硬件位置模式

## 范围

`src/omr_hardware/third_party/dais_motor/`（纯 C++ Modbus 驱动，零 ROS 依赖）

## 问题描述

Issue #63 含两个严重问题：

### Bug 1：位置反馈读错寄存器

`comm_loop()` 用 **H0B_13**（输入脉冲计数器 / 指令累加）填充 `MotorState::position_rad`，
而不是编码器真实位置 **H0B_07**。

| 寄存器 | 含义 | 原用法 |
|--------|------|--------|
| H0B_07 | 绝对位置计数器（真反馈） | 未读 |
| H0B_13 | 输入脉冲计数器（指令累加） | 误用作 `position_rad` |
| H0B_15 | 编码器位置偏差 | 未暴露 |

**影响：** 电机堵转 / 丢步时 `position_rad` 仍按指令递增，上层以为在闭环，实际是伪反馈。

### Bug 2：未实现驱动器硬件位置模式

- 仅有 `set_velocity_command`；无 `set_position_command`
- `configure_device()` 硬编码 `H02_00=0`（速度），未配置电子齿轮与多段位置
- Issue / 评论曾假设位置模式下 **H06_03 变为目标脉冲** —— **硬件探测证伪**

## 根因与探测结论

### 反馈侧
H0B_13 是指令侧累加器，与编码器实际位置解耦；正确反馈源为 H0B_07（instr. units）。

### 控制侧（关键门控）
对 `H02_00=1` 后向 `H06_03` 做 FC10 Int32 写入：电机不动，`H0B_07` 不变 → **不能把 H06_03 当位置目标**。

实测通过的通信位置路径（AIMotor 手册「通信控制位置」+ 硬件确认）：

```
H02_00 = 1          # 位置模式
H05_00 = 2          # 指令源 = 内部多段 (H11)
H11_04 = 1          # 绝对位移
H11_12 = 目标脉冲   # Int32，FC10，low-first
H03_04 = 28         # DI2 → FunIN.28 多段使能
H03_05: 0 → 1       # 边沿触发一段运行（OFF 需约 50ms）
```

验收反馈：`H0B_07` / `H0B_15`（问题 A 已就绪）。

## 解决方案

分两阶段实施，默认保持 **速度模式**，不破坏现有速度路径与 `DaisHardware`。

### 问题 A：真位置反馈

1. 新增 `REG_ABS_POSITION`（H0B_07）、`REG_POSITION_ERROR`（H0B_15）
2. `comm_loop` 改读 H0B_07 → `position_rad`；读 H0B_15 → `position_error_rad`
3. `MotorConfig.encoder_resolution`（默认 131072）用于 H0B_15 换算

换算：

```text
position_rad       = H0B_07_counts * 2π / gear_ratio_denom     # instr. units
position_error_rad = H0B_15_counts * 2π / encoder_resolution   # enc. units
```

### 问题 B：硬件位置模式 API

1. `ControlMode { Speed=0, Position=1 }`，默认 `Speed`
2. `set_position_command(double rad)` + 独立 `cmd_position_` atomic
3. FC10：`write_u32` / `write_i32`（low-first）
4. Position 配置：`H02_00=1`、`H05_00=2`、电子齿轮 `H05_07`/`H05_09`、H11 多段参数、DI2=Fun28
5. `comm_loop` 分支：Speed 写 H06_03 rpm；Position 在指令变化时写 H11_12 并边沿触发
6. 使能时锁存当前 H0B_07，避免瞬间跳变

### 评论风险对应处理

| 风险 | 处理 |
|------|------|
| 线程安全（单一 atomic） | 拆分 `cmd_velocity_` / `cmd_position_`；模式配置期固定，不热切 |
| comm_loop 分支回归 | 按 `control_mode` 分支；默认 Speed；速度斜坡硬件回归通过 |
| H06_03 语义切换 | **证伪**：位置不用 H06_03，改用 H11_12，两路寄存器不混用 |
| 电子齿轮 32 位写 | 位置配置时 FC10 写 H05_07 / H05_09 |

## 改动文件清单

| 文件 | 改动 |
|------|------|
| `dais_motor/include/dais_motor/registers.hpp` | H0B_07/H0B_15、H05_00、H11_*、DI2；移除未用的 `REG_PULSE_COUNTER` |
| `dais_motor/include/dais/motor.hpp` | `ControlMode`、`set_position_command`、位置 profile 字段 |
| `dais_motor/src/motor.cpp` | FC10 helper、configure/enable/comm_loop 双模式、触发 50ms OFF |
| `dais_motor/test/motor_hw_test.cpp` | `--mode position` 步进 + 默认速度斜坡 |
| `dais_motor/scripts/probe_position_mode.py` | 新建：H06_03 证伪 / H11 路径验证 |
| `dais_motor/docs/register_map.md` | 记录通信位置路径与 H06_03 非位置目标结论 |
| `dais_motor/CMakeLists.txt` | `motor_hw_test` 可执行目标（此前已有） |

## 不包含（明确边界）

- 不改 `DaisHardware` / ros2_control 导出 `HW_IF_POSITION` 命令
- 不实现 Homing / 清零 H0B_07
- 不运行时热切换控制模式（须 disable → configure → enable）
- Issue #64 / #65 其余项（FC10 能力已顺带具备，便于后续 #64）

## 验证结果

| 检查项 | 结果 |
|--------|------|
| Python 探测：H06_03 作位置目标 | FAIL（无运动） |
| Python 探测：H05_00=2 + H11_12 + 触发 | PASS（+200 脉冲到位） |
| `./motor_hw_test --mode position` | PASS：+0.5 / +1.0 / 回原点均到位 |
| `./motor_hw_test --mode speed` | PASS：60→600→0 rpm 跟踪正常 |
| Docker 编译 `dais_motor` + `motor_hw_test` | 通过 |

### 位置验收要点

- 连续步进曾因 `H03_05` 边沿过短导致第二步不动；`trigger_multi_segment()` 增加 **50ms OFF** 后连续步进稳定
- 触发仅在位置指令变化时执行，不阻塞每个 20ms 通信周期

## 使用方式

```bash
# 默认速度模式（行为与修复前一致）
./motor_hw_test /dev/ttyUSB0 1

# 位置模式步进
./motor_hw_test /dev/ttyUSB0 1 --mode position
```

```cpp
dais::MotorConfig cfg;
cfg.control_mode = dais::ControlMode::Position;  // 显式开启
dais::Motor motor(cfg);
// connect → configure_device → enable
motor.set_position_command(target_rad);  // 绝对位置
```
