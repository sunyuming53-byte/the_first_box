# rm_driver 功能包

底层驱动功能包，通过 TCP 与机械臂通信，订阅/发布 ROS2 话题。是 ROS2 控制的**核心基础包**。

GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_driver

## 1. 使用

### 基础启动（默认 IP: 192.168.1.18）

```bash
ros2 launch rm_driver rm_<arm_type>_driver.launch.py
```

`<arm_type>`: `65`, `63`, `63_III`, `eco65`, `eco63`, `75`, `gen72`

### 进阶使用：修改 IP 配置

配置文件位置: `rm_driver/config/rm_<model>_config.yaml`

```yaml
rm_driver:
  ros__parameters:
    arm_ip: "192.168.1.18"        # TCP 连接 IP
    tcp_port: 8080                # TCP 端口
    arm_type: "RM_65"             # 型号: RM_65 | RM_eco65 | RM_eco63 | RML_63 | RM_75 | GEN_72
    arm_dof: 6                    # 自由度

    udp_ip: "192.168.1.10"        # UDP 主动上报 IP
    udp_cycle: 5                  # UDP 上报周期（5的倍数）
    udp_port: 8089                # UDP 上报端口
    udp_force_coordinate: 0       # 六维力基准坐标: 0=传感器 1=工作坐标系 2=工具坐标系
    udp_hand: false               # 灵巧手 UDP 上报使能
    udp_plus_base: false          # 末端设备基础信息上报
    udp_plus_state: false         # 末端设备实时信息上报
    udp_joint_speed_state: true   # 关节速度上报
    udp_lift_state: false         # 升降关节上报
    udp_expand_state: false       # 拓展关节上报
    udp_arm_current_status: false # 机械臂状态上报
    udp_aloha_state: false        # aloha 状态上报

    trajectory_mode: 0            # 高跟随模式: 0=完全透传 1=曲线拟合 2=滤波
    radio: 0                      # 平滑系数(0-100) / 滤波参数(0-1000)
    arm_joints: ["joint1","joint2","joint3","joint4","joint5","joint6"]
```

修改后需重新编译:
```bash
cd ~/ros2_ws
colcon build
```

## 2. 架构

```
├── CMakeLists.txt
├── config/                        # 各型号 YAML 配置
│   ├── rm_63_config.yaml
│   ├── rm_65_config.yaml
│   ├── rm_75_config.yaml
│   ├── rm_eco65_config.yaml
│   ├── rm_eco63_config.yaml
│   └── rm_gen72_config.yaml
├── include/rm_driver/             # API 头文件
├── launch/                        # 各型号 launch 文件
│   ├── rm_63_driver.launch.py
│   ├── rm_65_driver.launch.py
│   ├── rm_75_driver.launch.py
│   ├── rm_eco65_driver.launch.py
│   ├── rm_eco63_driver.launch.py
│   └── rm_gen72_driver.launch.py
├── lib/                           # RM_Service 动态库
├── package.xml
└── src/rm_driver.cpp              # 驱动主代码
```

## 3. 话题

启动后通过 `ros2 topic list` 查看。主要话题分类:

- **运动控制**: MoveJ, MoveJ_P, MoveL, MoveC, MoveJ_CANFD 等
- **状态查询**: 关节角度、末端位姿、六维力等
- **配置管理**: 坐标系、工具、IO、Modbus 等
- **UDP 上报**: 实时状态推送
