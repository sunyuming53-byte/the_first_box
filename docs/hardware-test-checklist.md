# RealMan 真机测试操作手册

> 目标：按照这份手册逐步操作，验证 `realman_driver`、`realman_vision`、`realman_calibration` 三个包在真机上的功能是否正常。

---

## 0. 准备工作

### 0.1 硬件检查

- [ ] 机械臂已开机，网线已连接到 MiniPC
- [ ] Intel RealSense D400 相机已通过 USB 3.0 连接到 MiniPC
- [ ] 棋盘格标定板已打印（11×8 内角点，方格边长 30mm），贴在平整硬板上
- [ ] 机械臂工作范围内无杂物，急停按钮可触及

### 0.2 环境初始化

打开终端，执行以下命令：

```bash
# 1. 加载 ROS2 环境
source /opt/ros/humble/setup.bash
source ~/ws/realman/install/setup.bash

# 2. 确认机械臂 IP（通常默认是 192.168.1.18，不确定的话问负责人）
ping 192.168.1.18    # 应该能 ping 通

# 3. 确认相机连接
ls /dev/video*        # 应该能看到 realsense 相关的 video 设备
```

### 0.3 重要提醒

| 项目 | 说明 |
|------|------|
| **单位** | 代码中角度用**弧度**，长度用**米**。示教器上角度用**度**，长度用**毫米**。对比时注意换算：1 rad ≈ 57.3°，1 m = 1000 mm |
| **安全** | 运行运动测试前确保周围无人。急停按钮随手可及 |
| **报错记录** | 任何步骤报错，截图或复制完整错误信息（包含错误码），记录到测试报告中 |
| **测试数据** | 所有测试产物（图片、CSV、YAML）保留，不要删除，测试结束后统一归档 |

---

## 1. Phase 1：基础硬件连通性

### T1.1 — 机械臂 TCP 连接

**操作：**

```bash
# 运行 gripper_test，它会自动连接机械臂（默认 IP 192.168.1.18）
ros2 run realman_driver gripper_test
```

**期望看到：**
```
[1] Connected to arm
[2] Gripper pre-check:
  enable: 1  online: 1  mode: ...
```

**通过标准：**
- 不报错，能看到 "Connected to arm"
- 能看到夹爪状态信息（online: 1 表示夹爪在线）

**如果失败：**
1. 检查 IP 是否正确：`ping 192.168.1.18`
2. 检查机械臂是否开机（示教器有显示）
3. 确认机械臂和 MiniPC 在同一网段
4. 如果机械臂 IP 不是 192.168.1.18，可以用 `--ip` 指定：
   ```bash
   ros2 run realman_calibration collect_data --ip <实际IP> --count 1
   ```

- [ ] **T1.1 通过**

---

### T1.2 — RealSense 相机采集

**操作：**

```bash
ros2 run realman_calibration collect_data --ip 192.168.1.18 --output /tmp/test_cam --count 5
```

> 有显示器时：会弹出相机预览窗口，能看到实时彩色画面。
> 无显示器时（SSH 远程）：自动降级为 headless 模式，终端输出状态日志。

**期望看到：**
- 预览窗口（或终端日志）显示彩色图像帧
- 帧率稳定，无闪烁或黑屏

**通过标准：**
- 连续能看到画面，不报错
- 按 `Q` 可正常退出

- [ ] **T1.2 通过**

---

## 2. Phase 2：机械臂运动与状态读取

### T2.1 — 关节位置读取

**操作：**

```bash
ros2 run realman_driver joint_test
```

> 连接机械臂后每秒打印一次 6 个关节角度（同时显示弧度和度），共 20 次。按 `Ctrl+C` 可提前退出。

**期望看到：**

```
[1] Joints (rad): J1=0.xxx  J2=0.xxx  J3=0.xxx  J4=0.xxx  J5=0.xxx  J6=0.xxx
               (deg): J1=xx.x   J2=xx.x   J3=xx.x   J4=xx.x   J5=xx.x   J6=xx.x
```

**通过标准：**
- 6 个关节的读数都是正常弧度值（范围大约在 -3.14 到 +3.14 之间）
- 读数不为全零（全零说明未读到数据）
- 手动轻推机械臂某个关节（松开刹车后），对应读数有变化

- [ ] **T2.1 通过**

---

### T2.2 — 末端位姿读取

**操作：**

先用示教器记下当前机械臂末端位姿（示教器上通常显示 x/y/z 毫米 和 roll/pitch/yaw 度）。

有显示器时，运行采集程序在 overlay 窗口读取：

```bash
ros2 run realman_calibration collect_data --ip 192.168.1.18 --count 1
```

> 有显示器时：CV 窗口左上角显示 `Arm: x=... y=... z=... roll=... pitch=... yaw=...`（角度已换算为度）。无显示器时可改用 `joint_test`（输出不含 toolPose），建议在有显示器的环境测试。

**通过标准：**
- CV overlay 显示的位姿数值与示教器一致
- **单位换算：** 示教器角度÷57.3 = 弧度，示教器位置÷1000 = 米
- 示例：示教器显示 X=500mm, Roll=30° → overlay 显示接近 X=0.5m, Roll=30°（overlay 已转换为度）

- [ ] **T2.2 通过**

---

### T2.3 — moveJ 弧度/度转换验证（最高优先级）

> **这是最重要的测试。如果转换有 bug，机械臂会以错误的角度运动，可能造成碰撞。**

**操作：**

1. 在示教器上记下当前 Joint1 的角度（比如显示 10°）
2. 运行 movej_test：

```bash
ros2 run realman_driver movej_test
```

3. 观察机械臂实际运动后，在示教器上再次读取 Joint1 角度

**通过标准：**
- 机械臂 Joint1 实际旋转约 28.6°（即 0.5 rad），其他关节基本不动
- 终端输出 "MoveJ complete"，不报错
- **如果 Joint1 只转了 ~0.5° 而不是 ~28.6°，说明转换有 bug！立刻停止测试，报告负责人。**

- [ ] **T2.3 通过**

---

### T2.4 — 急停功能

**操作：**

```bash
# 终端 1：启动 arm_node
ros2 run realman_driver arm_node --ros-args -p arm_ip:=192.168.1.18

# 终端 2：发送一个运动指令让机械臂动起来（通过服务或有专门的测试），
# 然后立即调用紧急停止
ros2 service call /arm_node/stop std_srvs/srv/Trigger "{}"
```

**通过标准：**
- 机械臂在调用 stop 后 1 秒内完全停止
- 不报错，service 返回 success: true

- [ ] **T2.4 通过**

---

### T2.5 — 直线运动 moveL

**操作：**

```bash
ros2 run realman_driver movel_test
```

> 程序连接机械臂 → 读取当前 TCP 位姿 → Z 轴 +50mm → 执行 moveL → 读取并打印最终位姿和 Z 偏移量。

**期望看到：**

```
Current TCP: x=... y=... z=... roll=... pitch=... yaw=...
Target TCP:  x=... y=... z=... roll=... pitch=... yaw=...
Executing moveL (speed=20, blocking)...
Final TCP:   x=... y=... z=... roll=... pitch=... yaw=...
Delta Z = 0.0500 m  (expected ~0.050)
moveL test PASSED
```

**通过标准：**
- 机械臂末端沿直线上升约 50mm
- 示教器上轨迹为直线
- 终端 Delta Z ≈ 0.050 m

- [ ] **T2.5 通过**

---

## 3. Phase 3：夹爪操作

### T3.1 — 夹爪基本开合

**操作：**

```bash
ros2 run realman_driver gripper_test
```

**观察：**
- 步骤 [4]：夹爪完全张开
- 步骤 [5]：夹爪闭合到一半（500）
- 步骤 [6]：力控抓取（如果有物体放入）
- 步骤 [7]：夹爪完全闭合（1000）
- 步骤 [8]：夹爪再次张开

**通过标准：**
- 各步骤夹爪动作正常
- 终端打印 "All gripper tests passed."
- 无超时报错

- [ ] **T3.1 通过**

---

### T3.2 — 力控抓取 gripperPick

**操作：**

在夹爪中间放入一个物体（如纸杯、小块泡沫），运行 gripper_test。

**通过标准：**
- 步骤 [6] 夹爪闭合到接触物体后自动停止（不会夹扁物体）
- gripperState 输出的 mode 字段显示力控模式

- [ ] **T3.2 通过**

---

### T3.3 — 夹爪状态读取

**操作：**

分三次运行 gripper_test，每次在启动前手动将夹爪调整到不同位置（全开/半开/全闭）。

**通过标准：**
- 三次的 `actpos` 值递增或递减一致
- `actpos=0` 接近全开，`actpos=1000` 接近全闭

- [ ] **T3.3 通过**

---

## 4. Phase 4：棋盘格检测验证

### T4.1 — 棋盘格检出率

**操作：**

```bash
ros2 run realman_calibration collect_data --ip 192.168.1.18 --output /tmp/calib_test --count 18
```

> 启动后进入交互模式：
> - **绿色圆点 overlay** = 棋盘格角点被成功检测
> - **红色叉号** = 未检测到
> - 看到绿色角点 → 按 **S** 保存当前帧
> - 按 **Q** 退出

将棋盘格放在相机视野的 **至少 10 个不同角度**，每个角度尝试按一次 S。建议覆盖：
- 正面（棋盘格正对相机）
- 左倾 30° / 右倾 30°
- 上仰 30° / 下俯 30°
- 左旋 45° / 右旋 45°
- 近距 / 远距各一张

**通过标准：**
- 10 个不同角度中 ≥ 8 个成功检出（显示绿色角点）
- 角点 overlay 稳定，不闪烁

**常见问题：**
- 棋盘格反光 → 调整光源角度
- 棋盘格太远 → 靠近相机
- 一直检测不到 → 检查 `--board-w` / `--board-h` 是否与打印的棋盘格内角点数一致

- [ ] **T4.1 通过**

---

## 5. Phase 5：数据采集集成

### T5.1 — 完整采集 18 张图片

**操作：**

```bash
ros2 run realman_calibration collect_data \
    --ip 192.168.1.18 \
    --output /tmp/calib_test \
    --count 18 \
    --board-w 11 --board-h 8 --square-size 0.03
```

> 按照 T4.1 的方法，在 18 个不同角度各按一次 S。注意：
> - 机械臂每张图之间要有**明显不同**的旋转（不能只平移棋盘格）
> - 采集完后系统会自动结束（如果旋转多样性过关）

**通过标准：**
- 程序结束时提示 "Rotation diversity OK"（不是 "WARNING: rotation diversity insufficient"）
- `/tmp/calib_test/images/` 下有 18 张 jpg 图片（00001.jpg ~ 00018.jpg）
- `/tmp/calib_test/robot_poses.csv` 有 19 行（1 行 header + 18 行数据）

```bash
# 验证命令
ls /tmp/calib_test/images/ | wc -l        # 应该输出 18
wc -l /tmp/calib_test/robot_poses.csv      # 应该输出 19
```

- [ ] **T5.1 通过**

---

### T5.2 — 旋转多样性门控验证

**操作：**

重新采集一次，这次故意**只在相似角度**拍（机械臂不动，只平动棋盘格）。

**通过标准：**
- 程序提示 "WARNING: rotation diversity insufficient"
- 按 Q 可以正常退出，不会崩溃

- [ ] **T5.2 通过**

---

## 6. Phase 6：全流水线端到端

### T6.1 — run_pipeline 一键标定

**方式 A（推荐）：交互式全流程**

```bash
# 一键跑通采集 + 标定（不用先 collect_data）
ros2 run realman_calibration run_pipeline \
    --ip 192.168.1.18 \
    --output /tmp/calib_full \
    --count 18 \
    --mode in_hand \
    --board-w 11 --board-h 8 --square-size 0.03
```

> 程序会依次执行：采集图片 → 相机标定 → 位姿处理 → 手眼标定。
> Stage 1 是交互采集阶段，跟 T5.1 一样操作。

**方式 B（分步执行）：**

```bash
# 第一步：采集数据
ros2 run realman_calibration collect_data \
    --ip 192.168.1.18 --output /tmp/calib_full --count 18

# 第二步：相机内参标定
ros2 run realman_calibration calibrate_camera \
    --output /tmp/calib_full --board-w 11 --board-h 8 --square-size 0.03

# 第三步：手眼标定
ros2 run realman_calibration compute_hand_eye \
    --output /tmp/calib_full --mode in_hand
```

**通过标准：**
- 所有阶段不报错
- 最终输出 `/tmp/calib_full/calibration_result.yaml` 文件

- [ ] **T6.1 通过**

---

### T6.2 — 标定结果质量评估

**操作：**

```bash
cat /tmp/calib_full/calibration_result.yaml
```

**逐项检查：**

| 检查项 | 字段 | 标准 | 实际值 | 通过？ |
|--------|------|------|--------|--------|
| 旋转矩阵 | `rotation_matrix` | 3×3，行列式 ≈ 1.0（误差 < 0.01） | | |
| 平移向量 | `translation_vector` | 3×1，值 < 2m | | |
| 重投影误差 | `reprojection_error` | < 100.0 像素 | | |
| 条件数 | `condition_number` | > 0 且 < 1000 | | |
| 求解方法 | `method` | Tsai / Park / Horaud / Daniilidis 之一 | | |
| 目测验证 | 观察 R 矩阵 | 若相机固定在夹爪附近正方向，R 应接近单位阵 | | |

> **目测辅助判断**：如果 R 矩阵和单位阵差很多，或者 t 向量和相机到法兰的物理距离差很多，可能标定有问题。

- [ ] **T6.2 通过**

---

## 7. Phase 7：ROS2 节点集成

### T7.1 — calib_node 启动

**操作：**

```bash
# 终端 1：启动 calib_node
ros2 run realman_calibration calib_node --ros-args -p arm_ip:=192.168.1.18
```

**通过标准：**

```bash
# 终端 2：检查节点是否存在
ros2 node list
```
输出中能看到 `/calib_node`。

- [ ] **T7.1 通过**

---

### T7.2 — 服务调用

**操作：**

在 calib_node 运行的状态下，依次调用各服务：

```bash
# 相机标定服务（立即返回）
ros2 service call /calib_node/calibrate std_srvs/srv/Trigger "{}"

# 手眼标定服务（立即返回，需先跑过 calibrate）
ros2 service call /calib_node/hand_eye std_srvs/srv/Trigger "{}"

# 紧急停止服务（立即返回）
ros2 service call /calib_node/stop std_srvs/srv/Trigger "{}"
```

> ⚠️ `~/run` 服务会进入交互采集模式，通过 ros2 service call 调用会一直阻塞直到人工按 Q 或 S 完成采集，不适合一键验证。建议用 T6.1 的 `run_pipeline` 替代。

**通过标准：**
- 每个 service call 返回 success: true
- calibrate 服务返回相机标定结果（重投影误差、使用图像数）
- hand_eye 服务返回手眼标定结果（方法名称）
- stop 服务返回 success: true，机械臂急停

- [ ] **T7.2 通过**

---

### T7.3 — TF 坐标变换广播验证

**操作：**

在 calib_node 完成 hand_eye 标定后：

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

**通过标准：**
- 能看到连续的 TF 变换输出（Translation + Rotation 数值）
- 不报 "can't find transform" 错误

- [ ] **T7.3 通过**

---

## 8. 测试汇总

### 完成情况

| 阶段 | 测试项 | 状态 | 备注 |
|------|--------|------|------|
| P1 | T1.1 — Arm TCP 连接 |  | |
| P1 | T1.2 — 相机采集 |  | |
| P2 | T2.1 — 关节位置读取 |  | |
| P2 | T2.2 — 末端位姿读取 |  | |
| **P2** | **T2.3 — 弧度/度转换** |  |  |
| P2 | T2.4 — 急停 |  | |
| P2 | T2.5 — 直线运动 moveL |  | |
| P3 | T3.1 — 夹爪基本开合 |  | |
| P3 | T3.2 — 力控抓取 |  | |
| P3 | T3.3 — 夹爪状态读取 |  | |
| P4 | T4.1 — 棋盘格检出率 |  | |
| P5 | T5.1 — 18 张完整采集 |  | |
| P5 | T5.2 — 旋转多样性门控 |  | |
| P6 | T6.1 — 全流水线标定 |  | |
| P6 | T6.2 — 标定结果质量 |  | |
| P7 | T7.1 — calib_node 启动 |  | |
| P7 | T7.2 — 服务调用 |  | |
| P7 | T7.3 — TF 广播 |  | |

### 测试环境记录

| 项目 | 填写 |
|------|------|
| 测试日期 | 202_ - _ - _ |
| 机械臂型号 | |
| 机械臂 IP | |
| 相机型号 | |
| 标定板规格 | 内角点 __ × __ ，方格 ___ mm |

---

## 附录：常见问题

### A. 网络不通

```bash
# 确认同一网段
ip addr show    # 看 MiniPC 的 IP
ping 192.168.1.18

# 确认端口 8080 开放
nc -zv 192.168.1.18 8080
```

### B. 相机无法打开

```bash
# 检查 realsense 设备
rs-enumerate-devices

# 重新插拔 USB 线
# 确认是 USB 3.0 口（蓝色）而非 USB 2.0
```

### C. 棋盘格检测不到

1. 确保光线均匀，无强烈反光
2. 棋盘格占画面 1/4 ~ 1/2 面积
3. 棋盘格平整，无弯曲
4. 用 `--board-w 11 --board-h 8` 确认内角点数正确
