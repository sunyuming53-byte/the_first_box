# Issue #63 / #64 / #65 合并解决报告 — dais_motor 位置反馈、位置模式、配置解耦与故障保护

## 范围

`src/omr_hardware/third_party/dais_motor/` 及其上层接入点 `src/omr_hardware/src/dais_hardware.cpp`

> 目标：将 Issue #63、#64、#65 相关问题合并梳理，给出本轮已完成项、仍存疑项与后续边界。

## 背景与问题总览

三个 issue 关注的是同一条控制链路上的不同层次问题：

### Issue #63：位置反馈与位置模式主问题

1. `position_rad` 读错寄存器：原先读 **H0B_13**（输入脉冲累加），不是编码器真实位置 **H0B_07**
2. 缺少驱动器硬件位置模式 API：只有速度命令，没有 `set_position_command`
3. 曾假设位置模式可直接写 **H06_03** 作为目标位置 —— 后经硬件验证证伪

### Issue #64：配置与 32 位写支持

1. 缺少 32 位 Modbus 写（FC10）能力
2. `configure_device()` 未显式写 `H02_00`
3. `enable()` 内部重复调用 `configure_device()`，配置与使能未解耦

### Issue #65：保护与误差监控

1. 读到故障码后只打印日志，不停机、不失能
2. 未读取 **H0B_15** 编码器偏差计数器，无法直接暴露跟踪误差

---

## 根因分析

### 1. 反馈侧把“指令累加”误当“真实位置”

原实现中：

- `H0B_13` 被当成 `position_rad`
- 实际上 H0B_13 只是输入脉冲计数器
- 电机堵转/丢步时它仍可能继续变化，导致上层拿到伪反馈

正确反馈源应为：

| 寄存器 | 含义 | 正确用途 |
|--------|------|----------|
| H0B_07 | 绝对位置计数器（32 位） | `position_rad` |
| H0B_15 | 编码器位置偏差（32 位） | `position_error_rad` |
| H0B_13 | 输入脉冲计数器（32 位） | 仅指令侧参考，不应作为真实位置 |

### 2. 位置模式寄存器语义最初判断错误

最初 issue 假设：

```text
H02_00=1 后，H06_03 变成位置目标（Int32）
```

但硬件探测结果表明：

- 对 `H06_03` 做 FC10 Int32 写入时，电机**不动**
- `H0B_07` 无变化

最终确认可用的通信位置路径为：

```text
H02_00 = 1          # 位置模式
H05_00 = 2          # 指令源 = 内部多段(H11)
H11_04 = 1          # 绝对位置
H11_12 = 目标脉冲   # Int32, FC10, low-first
H03_04 = 28         # DI2 → FunIN.28
H03_05: 0 → 1       # 边沿触发运行
```

### 3. 保护逻辑缺失

原 `comm_loop()` 对故障码的处理只有：

1. 读取 `H0B_34`
2. 保存到 `state_fault_`
3. 打印日志
4. 继续循环、继续发命令

这意味着驱动器已进入故障时，软件侧仍可能继续写速度或位置指令。

### 4. 配置职责不清

原先 `enable()` 内部再次调用 `configure_device()`，带来两个问题：

1. 每次使能都重复写控制模式、力矩限、加减速、位置模式参数
2. “配置”和“使能”两个动作语义混在一起，调用约定不清晰

---

## 解决方案与实现结果

## 一、Issue #63：真位置反馈 + 硬件位置模式

### A. 位置反馈修复

完成项：

1. 新增/使用 `REG_ABS_POSITION`（H0B_07）
2. 新增/使用 `REG_POSITION_ERROR`（H0B_15）
3. `comm_loop()` 改为：
   - H0B_07 → `position_rad`
   - H0B_15 → `position_error_rad`
4. `MotorConfig.encoder_resolution` 用于误差换算

换算关系：

```text
position_rad       = H0B_07_counts * 2π / gear_ratio_denom
position_error_rad = H0B_15_counts * 2π / encoder_resolution
```

效果：

- `position_rad` 终于表示真实编码器位置
- 上层可直接使用 `position_error_rad` 监测跟踪误差

### B. 位置模式实现

完成项：

1. 新增 `ControlMode { Speed, Position }`
2. 默认仍为 `Speed`，保持原有速度链路兼容
3. 新增 `set_position_command(double rad)`
4. 新增 `cmd_position_`，与 `cmd_velocity_` 分离
5. 新增 FC10 写辅助：
   - `write_u32`
   - `write_i32`
6. `configure_device()` 在 Position 模式下配置：
   - `H02_00=1`
   - `H05_00=2`
   - `H05_07` / `H05_09`
   - `H11_*` 多段参数
   - `DI2 = FunIN.28`
7. `comm_loop()` 在 Position 模式下：
   - 仅在命令变化时写 `H11_12`
   - 再对 `H03_05` 做边沿触发
8. `enable()` 时先锁存当前 H0B_07，避免使能瞬间跳变

### C. 位置模式连续触发修正

实测曾出现：

- 第一步到位
- 第二步不动作

根因是 `H03_05` 的触发边沿不干净。修正为：

```text
先拉 0
等待约 50ms
再拉 1
```

结果：连续步进稳定。

---

## 二、Issue #64：32 位写与配置/使能解耦

### A. FC10 / 32 位写能力

完成项：

1. 增加 `modbus_write_registers` 前向声明
2. 落地 `write_u32` / `write_i32`
3. 实际用于：
   - `H05_07` 编码器分辨率
   - `H05_09` 齿轮比分母
   - `H11_12` 位置目标

注意：

- **Issue #64 原文中“位置模式写 H06_03 Int32”已被硬件证伪**
- FC10 能力仍然是必需的，但真实目标寄存器是 **H11_12**，不是 H06_03

### B. 显式写控制模式

完成项：

1. `configure_device()` 中显式写 `H02_00`
2. 不再依赖驱动器上电默认模式

### C. 配置与使能解耦

完成项：

1. `enable()` **不再**内部调用 `configure_device()`
2. API 契约改为：

```cpp
motor.connect();
motor.configure_device();  // while disabled
motor.enable();
```

效果：

- 配置动作不再重复执行
- 语义更清晰
- 便于上层管理“重配”和“重使能”

---

## 三、Issue #65：故障保护与偏差监控

### A. H0B_15 偏差读取

完成项：

1. 每周期读取 H0B_15
2. 暴露为 `MotorState::position_error_rad`

这部分已经随 Issue #63 一并解决。

### B. 故障保护

完成项：

1. 当 `H0B_34 != 0` 时，首次触发保护
2. 保护动作包括：
   - 清零速度命令
   - 置位 `comm_error_`
   - 置位 `fault_latched_`
   - 速度模式下写 `H06_03=0`
   - 位置模式下降低 DI2 触发
   - 写 `H32_01=0` 自动失能
3. 之后 `set_velocity_command` / `set_position_command` 会被忽略
4. 新增：
   - `MotorState.comm_error`
   - `Motor::has_error()`

恢复方式：

```text
排除故障 → （必要时重新 configure_device）→ enable()
```

### C. 上层 DaisHardware 联动

完成项：

1. `read()` 中发现 `comm_error` / `fault_code != 0` 时节流报错
2. `write()` 中若 `motor_->has_error()`，则跳过下发新命令

效果：

- 底层驱动先自保
- 上层硬件接口不再在故障后继续给命令

---

## 改动文件清单

| 文件 | 改动 |
|------|------|
| `src/omr_hardware/third_party/dais_motor/include/dais_motor/registers.hpp` | H0B_07 / H0B_15 / H11 / 控制模式相关寄存器；补充 H0B_24 现状说明 |
| `src/omr_hardware/third_party/dais_motor/include/dais/motor.hpp` | `ControlMode`、`set_position_command`、`position_error_rad`、`comm_error`、`has_error()` |
| `src/omr_hardware/third_party/dais_motor/src/motor.cpp` | 真位置反馈、FC10 helper、位置模式、配置解耦、故障保护、电流符号修正 |
| `src/omr_hardware/third_party/dais_motor/test/motor_hw_test.cpp` | 速度/位置模式测试，输出误差与错误状态 |
| `src/omr_hardware/third_party/dais_motor/scripts/probe_position_mode.py` | 证伪 H06_03，验证 H11 路径 |
| `src/omr_hardware/third_party/dais_motor/docs/register_map.md` | 更新位置模式真实路径与寄存器说明 |
| `src/omr_hardware/src/dais_hardware.cpp` | 上层故障联动，错误时停止继续写命令 |

---

## 验证结果

| 检查项 | 结果 |
|--------|------|
| H06_03 作为位置目标 | FAIL（无运动） |
| H05_00=2 + H11_12 + 触发 | PASS |
| `motor_hw_test --mode position` | PASS（+0.5 / +1.0 / 回原点） |
| `motor_hw_test --mode speed` | PASS（速度斜坡正常） |
| Docker 编译 `dais_motor` | PASS |
| Docker 编译 `omr_hardware` | PASS |

---

## 当前仍保留的边界 / 未解决项

以下不在本轮正式闭环范围内：

1. **不改 ros2_control 命令接口为 position**
   - `DaisHardware` 目前仍只导出 `HW_IF_VELOCITY`

2. **不实现 Homing / H0B_07 清零逻辑**

3. **不支持运行中热切换控制模式**
   - 仍要求：`disable -> configure_device -> enable`

4. **H0B_24 相位电流的“位宽”仍未最终定案**
   - 现象上确认原实现存在 bug：把可能的有符号值按无符号解析，导致空闲时可能显示 655A 左右
   - 目前已先修正为：**按 signed Int16 × 0.01A 解析**
   - 但是否真为 Int32，现有仓库内文档证据不足，需后续硬件对照确认

---

## 经验结论

这三组 issue 的本质不是彼此独立，而是同一个驱动从“能转”走向“可闭环、可保护、可维护”的连续修复过程：

1. **先修真反馈**，否则闭环没有意义
2. **再修位置模式真实通路**，且必须以硬件探测为准，不能只信手册假设
3. **再补配置职责与保护语义**，否则可用但不安全

最终结果：

- `dais_motor` 现已具备真实位置反馈
- 具备可工作的硬件位置模式
- 具备显式模式配置与配置/使能解耦
- 具备基础故障自保护

已从“只能粗略速度控制”提升到“可做真实位置闭环实验与上层联调”的状态。
