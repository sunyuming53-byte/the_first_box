# omr_lio 使用教程

基于 M65 底盘、Mid-360 激光雷达、S-FAST_LIO SLAM 和 Nav2 的自主导航操作指南。

---

## 1. 环境准备

### 硬件要求

- Mid-360 激光雷达已通过网线连接到工控机
- M65 底盘已通过串口线连接到工控机（默认 `/dev/ttyBase`）
- 激光雷达与底盘电源已接通

### 软件要求

- ROS2 Humble 环境已配置（`source /opt/ros/humble/setup.bash`）
- 工作空间已编译

```bash
cd ~/pipeline
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> 首次使用或拉取新代码后必须执行 `colcon build`。

### 网络配置

Mid-360 默认 IP 为 `192.168.1.50`（具体参考激光雷达说明书），工控机需配置同一网段：

```bash
sudo ip addr add 192.168.1.100/24 dev eth0  # eth0 替换为实际网口名
```

---

## 2. 建图（SLAM）

启动 LIO 建图节点，接收激光雷达和 IMU 数据，实时输出里程计和点云。

```bash
ros2 launch omr_lio lio_mapping.launch.py launch_livox_driver:=true
```

### 启用的节点

| 节点 | 说明 |
|------|------|
| `lio_node` | S-FAST_LIO 核心，订阅 `/livox/lidar` 和 `/livox/imu` |
| `livox_ros_driver2_node` | Mid-360 驱动（条件：`launch_livox_driver:=true`） |

### 运行时话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/lio/odom` | nav_msgs/Odometry | LIO 里程计（频率 ~100Hz） |
| `/cloud_registered` | sensor_msgs/PointCloud2 | 配准后的全局点云 |
| `/laser_map` | sensor_msgs/PointCloud2 | 增量式点云地图 |

TF 树：`map` → `odom`（由 LIO 发布）。

### 保存点云地图

编辑配置文件 `src/omr_lio/config/lio.yaml`：

```yaml
pcd_save:
    pcd_save_en: true      # false → true
    interval: -1            # -1 表示所有帧保存到一个 PCD 文件
```

保存后重启 LIO，地图会自动保存到程序工作目录的 `pcd/` 文件夹下，文件名为 `map_*.pcd`。

> 注意：关闭 `pcd_save_en` 再重启可关闭自动保存。PCD 文件较大（通常几百 MB），建议建图完成后复制到其他位置。

---

## 3. 航点录制

录制航点有两种方式：自动录制（按移动距离触发）和遥控手动录制（通过话题触发）。

两种方式均依赖 LIO 里程计 `/lio/odom`，请确保 LIO 在建图或定位模式下运行。

### 3.1 自动录制（按距离）

```bash
ros2 run omr_lio record_waypoints
```

机器人每移动 0.3 米（默认阈值）自动记录一个航点。生成的 CSV 保存在 `src/omr_lio/data/waypoints_<时间戳>.csv`。

可调整参数：

```bash
# 修改最小距离阈值为 0.5 米
ros2 run omr_lio record_waypoints --ros-args -p min_distance:=0.5

# 指定输出路径和文件名
ros2 run omr_lio record_waypoints --ros-args \
  -p output_dir:=/home/user/waypoints \
  -p file_name:=factory_floor1.csv
```

### 3.2 遥控手动录制

遥控机器人走到目标位置后，通过话题触发录制。

**录制机械臂检测点：**

```bash
ros2 topic pub /waypoint_task std_msgs/String "data: arm_inspect"
```

执行后，当前机器人位置会被记录，任务的 CSV 列标记为 `arm_inspect`。

**录制定时停靠点：**

```bash
ros2 topic pub /waypoint_stop_seconds std_msgs/Float32 "data: 10.0"
```

执行后，当前机器人位置会被记录，任务的 CSV 列标记为 `stop_10.0s`。

> 这些航点后续转换为 YAML 后，`inspection_sequencer` 会在到达时发布任务事件。

### 3.3 CSV 格式说明

自动生成的 CSV 包含以下列：

```
seq,stamp,frame_id,x,y,z,qx,qy,qz,qw,yaw,task,tol
0001,1704067200.123456,odom,1.234000,3.456000,0.000000,0.000000,0.000000,0.707107,0.707107,1.570796,arm_inspect,0.300
```

---

## 4. 转换 CSV 为 YAML

录制完成后，将 CSV 转换为 inspection_sequencer 可读的 YAML 格式：

```bash
ros2 run omr_lio csv_to_yaml \
  src/omr_lio/data/waypoints_20250101_120000.csv \
  inspection_points.yaml
```

生成的 YAML 格式如下：

```yaml
waypoints:
  - name: wp_001
    pose:
      x: 1.234
      y: 3.456
      z: 0.0
      qx: 0.0
      qy: 0.0
      qz: 0.707
      qw: 0.707
    task: arm_inspect
  - name: wp_002
    pose:
      x: 5.678
      y: 3.456
      z: 0.0
      qx: 0.0
      qy: 0.0
      qz: -0.707
      qw: 0.707
```

### 编辑 YAML

建议手动编辑 YAML 文件：

- 将 `wp_001` 改为有意义的名称，如 `cabinet_A_front`
- 添加或删除航点
- 修改任务类型（`task` 字段可选，不填或填 `none` 表示无任务）

> 注意：CSV 格式的 z 始终为 0（2D 平面导航），如需不同高度可在 YAML 中手动修改。

---

## 5. 自主导航

### 5.1 完整启动

通过 bringup 一次性启动 M65 底盘驱动、LIO 定位、Nav2 导航栈和 inspection_sequencer：

```bash
# 启动全部：M65 底盘 + LIO + Nav2 + inspection_sequencer
ros2 launch omr_bringup bringup.launch.py launch_m65:=true launch_m65_lio:=true
```

这一步包含：

| 组件 | 说明 |
|------|------|
| M65 底盘驱动 | `m65_controller_manager` + `diff_drive_controller` |
| robot_state_publisher | M65 运动学模型和 TF |
| Livox 驱动 | Mid-360 激光雷达 |
| LIO 节点 | 实时定位（`/lio/odom`、`/cloud_registered`） |
| Estopper | 紧急停障（默认 0.3m 阈值） |
| Nav2 规划器 | SmacHybrid2D（hybrid-A*）路径规划 |
| Nav2 控制器 | Regulated Pure Pursuit（跟踪路径） |
| velocity_smoother | 速度平滑，输出到 `/m65_controller_manager/diff_drive_controller/cmd_vel` |
| inspection_sequencer | 航点顺序执行器 |

### 5.2 使用 inspection_sequencer

#### 加载航点文件

启动时通过参数传入 YAML 航点文件：

```bash
ros2 launch omr_bringup bringup.launch.py launch_m65:=true launch_m65_lio:=true \
  inspection_sequencer.waypoints_file:=/path/to/inspection_points.yaml
```

#### 导航到指定航点

```bash
# 导航到 YAML 中定义的 cabinet_A_front 航点
ros2 topic pub /inspection_sequencer/go_to_waypoint std_msgs/String "data: cabinet_A_front"
```

sequencer 会执行：

1. 查找 YAML 中名为 `cabinet_A_front` 的航点
2. 通过 Nav2 action `NavigateToPose` 导航到目标位置
3. 到达后，如果该航点配置了 `task` 字段，执行任务握手

#### 直接导航（不经过 sequencer）

```bash
# 直接发布 PoseStamped 到 Nav2 action
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: map
  pose:
    position: {x: 2.5, y: 1.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.707, qw: 0.707}
"
```

### 5.3 任务握手流程

如果航点 YAML 配置了 `task` 字段（如 `arm_inspect`），到达该航点后会自动执行任务握手：

```
inspection_sequencer                外部模块（如机械臂调度器）
       │                                    │
       │── publish: start:arm_inspect:wp_001 ──→  /chassis/task_event
       │                                    │
       │←── subscribe: done ─────────────  /chassis/task_done
       │                                    │
       │── publish: done:arm_inspect:wp_001 ──→  /chassis/task_event
       │                                    │
```

握手协议：

| 步骤 | 方向 | 话题 | 格式 |
|------|------|------|------|
| 通知任务开始 | sequencer → 外部 | `/chassis/task_event` | `start:<task>:<waypoint_name>` |
| 任务完成反馈 | 外部 → sequencer | `/chassis/task_done` | 任意字符串（收到即视为完成） |
| 通知任务结束 | sequencer → 外部 | `/chassis/task_event` | `done:<task>:<waypoint_name>` |

sequencer 默认等待任务超时时间 180 秒，超时后继续执行下一个航点。

---

## 6. 紧急停障

EstopperNode 随 `m65_lio_nav.launch.py` 自动启动（`launch_estopper:=true`，默认开启）。

工作原理：订阅 `/cloud_registered` 点云，检测前方 0.3 米内是否有障碍物，如果障碍点数超过阈值，则发布紧急停止信号。

### 配置

```yaml
# src/omr_lio/config/estopper.yaml
/**:
  ros__parameters:
    stop_distance: 0.3       # 障碍物触发距离（米）
    min_points_in_zone: 5    # 最少触发点数（过滤噪点）
    front_only: true         # true = 仅检测前方，false = 全向检测
```

### 输出

| 话题 | 类型 | 说明 |
|------|------|------|
| `/lio/emergency_stop` | Bool | `true` = 停止，`false` = 正常 |

---

## 7. 话题速查表

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/livox/lidar` | PointCloud2 | 激光雷达 → LIO | Mid-360 原始点云 |
| `/livox/imu` | IMU | 激光雷达 → LIO | Mid-360 内建 IMU |
| `/lio/odom` | Odometry | LIO → 全系统 | LIO 里程计（导航用） |
| `/cloud_registered` | PointCloud2 | LIO → 全系统 | 配准后的全局点云 |
| `/laser_map` | PointCloud2 | LIO → 可视化 | 增量式点云地图 |
| `/lio/emergency_stop` | Bool | Estopper → 安全 | 紧急停障信号 |
| `/inspection_sequencer/go_to_waypoint` | String | 用户 → Sequencer | 导航到指定航点名 |
| `/chassis/task_event` | String | Sequencer → 外部 | 任务事件通知 |
| `/chassis/task_done` | String | 外部 → Sequencer | 任务完成确认 |
| `/waypoint_task` | String | 用户 → Recorder | 手动录制任务航点 |
| `/waypoint_stop_seconds` | Float32 | 用户 → Recorder | 手动录制停靠时长 |
| `/navigate_to_pose` | Action (Nav2) | 用户 → Nav2 | 直接导航到 Pose |
| `/m65_controller_manager/diff_drive_controller/cmd_vel` | Twist | Nav2 → 底盘 | 速度指令 |

---

## 8. 完整工作流示例

### 场景：工厂巡检

从零开始，完成建图、录制航点、配置巡检路线、执行自主导航。

```bash
# ────────── 终端 1：启动全部硬件 ──────────
ros2 launch omr_bringup bringup.launch.py launch_m65:=true launch_m65_lio:=true
```

```bash
# ────────── 终端 2：录制航点 ──────────
# 启动录音点程序
ros2 run omr_lio record_waypoints

# 遥控机器人走到第一个巡检点，录制（机械臂检测任务）
ros2 topic pub /waypoint_task std_msgs/String "data: arm_inspect"

# 走到第二个巡检点，录制（拍照任务）
ros2 topic pub /waypoint_task std_msgs/String "data: photo_capture"

# 走到第三个点，录制（定时停靠 30 秒）
ros2 topic pub /waypoint_stop_seconds std_msgs/Float32 "data: 30.0"

# ... 重复走到所有目标点并录制
# 录制完成后 Ctrl+C 停止 record_waypoints
```

```bash
# ────────── 终端 3：转换 CSV → YAML ──────────
# 查看生成的 CSV 文件
ls src/omr_lio/data/

# 转换
ros2 run omr_lio csv_to_yaml \
  src/omr_lio/data/waypoints_20250101_120000.csv \
  inspection_points.yaml

# 编辑 YAML，给航点起有意义的名字
# vim inspection_points.yaml
```

```bash
# ────────── 终端 1：重新启动，加载航点文件 ──────────
# Ctrl+C 停掉之前的 bringup，然后重新启动并传入航点文件
ros2 launch omr_bringup bringup.launch.py launch_m65:=true launch_m65_lio:=true \
  inspection_sequencer.waypoints_file:=/path/to/inspection_points.yaml
```

```bash
# ────────── 终端 2：逐点执行巡检 ──────────
# 导航到 cabinet_A 并执行 arm_inspect 任务
ros2 topic pub /inspection_sequencer/go_to_waypoint std_msgs/String "data: cabinet_A"

# 到达后机械臂自动执行检测，完成后继续
ros2 topic pub /inspection_sequencer/go_to_waypoint std_msgs/String "data: cabinet_B"

# 导航到 photo_point 执行拍照
ros2 topic pub /inspection_sequencer/go_to_waypoint std_msgs/String "data: photo_point"

# 导航到 charging_station 自动充电
ros2 topic pub /inspection_sequencer/go_to_waypoint std_msgs/String "data: charging_station"
```

### 查看当前状态

```bash
# 查看所有活跃节点
ros2 node list

# 查看 TF 树
ros2 run tf2_tools view_frames.py

# 可视化点云和导航（RViz2）
ros2 run rviz2 rviz2

# 监听任务事件
ros2 topic echo /chassis/task_event

# 查看紧急停障状态
ros2 topic echo /lio/emergency_stop
```

---

## 9. 常见问题

### 9.1 LIO 没有输出里程计

- 检查激光雷达网线连接和 IP 配置
- 检查 topic `/livox/lidar` 是否有数据：`ros2 topic echo /livox/lidar`
- 确认 launch 时传入了 `launch_livox_driver:=true`

### 9.2 底盘不动

- 检查串口权限：`sudo chmod 666 /dev/ttyBase`
- 检查 controller_manager 是否启动：`ros2 control list_controllers`
- 确认 `diff_drive_controller` 状态为 active

### 9.3 导航路径不平滑

- 调整 `nav2_params.yaml` 中的 `RegulatedPurePursuitController` 参数
- 降低 `desired_linear_vel`（默认 0.5 m/s）
- 增大 `min_lookahead_dist`（默认 0.3 m）

### 9.4 紧急停障误触发

- 增大 `estopper.yaml` 中的 `stop_distance`（默认 0.3 米）
- 增大 `min_points_in_zone`（默认 5 点）
- 检查点云中是否有自身结构反射

### 9.5 inspection_sequencer 报 "Unknown waypoint"

- 确认 YAML 文件路径正确
- 确认 `/inspection_sequencer/go_to_waypoint` 发布的名称与 YAML 中的 `name` 完全匹配（区分大小写）
- 确认启动时传入了 `inspection_sequencer.waypoints_file:=/path/to/file.yaml`
