# ROS2 自定义消息接口 (rm_ros_interfaces)

该功能包定义所有 RM 机械臂在 ROS2 下的控制消息和状态消息格式。

GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_ros_interfaces

## 消息文件列表

共 43 个 `.msg` 文件:

```
msg/
├── Armcurrentstatus.msg    # 机械臂当前状态
├── Armoriginalstate.msg    # 机械臂原始状态
├── Armstate.msg            # 机械臂状态
├── Cartepos.msg            # 笛卡尔位姿
├── Carteposcustom.msg      # 自定义笛卡尔位姿
├── Forcepositionmovejoint.msg # 力位混合控制关节运动
├── Forcepositionmovepose.msg  # 力位混合控制位姿运动
├── Force_Position_State.msg   # 力位混合控制状态
├── Getallframe.msg         # 获取所有坐标系
├── GetArmState_Command.msg # 获取机械臂状态指令
├── Gripperpick.msg         # 夹爪拾取
├── Gripperset.msg          # 夹爪设置
├── Handangle.msg           # 灵巧手角度
├── Handforce.msg           # 灵巧手力
├── Handposture.msg         # 灵巧手姿态
├── Handseq.msg             # 灵巧手序列
├── Handspeed.msg           # 灵巧手速度
├── Handstatus.msg          # 灵巧手状态
├── Jointcurrent.msg        # 关节电流
├── Jointenflag.msg         # 关节使能标志
├── Jointerrclear.msg       # 清除关节错误
├── Jointerrorcode.msg      # 关节错误代码
├── Jointposeeuler.msg      # 关节位姿(欧拉角)
├── Jointpos.msg            # 关节位置
├── Jointposcustom.msg      # 自定义关节位置
├── Jointspeed.msg          # 关节速度
├── Jointteach.msg          # 关节示教
├── Jointtemperature.msg    # 关节温度
├── Jointvoltage.msg        # 关节电压
├── Liftheight.msg          # 升降机高度
├── Liftspeed.msg           # 升降机速度
├── Liftstate.msg           # 升降机状态
├── Movec.msg               # 圆弧运动
├── Movej.msg               # 关节运动
├── Movejp.msg              # 关节空间到指定位姿
├── Movel.msg               # 直线运动
├── Ortteach.msg            # 姿态示教
├── Posteach.msg            # 位置示教
├── Rmerr.msg               # 错误信息
├── Rmplusbase.msg          # 末端设备基础信息
├── Rmplusstate.msg         # 末端设备实时信息
├── Setforceposition.msg    # 设置力位混合控制
├── Setrealtimepush.msg     # 设置实时推送
├── Sixforce.msg            # 六维力
└── Stop.msg                # 停止
```

## 核心消息定义

### 关节运动 Movej.msg
```
float32[] joint          # 关节角度 (rad)
uint8 speed              # 速度比例 0-100
bool block               # true=阻塞, false=非阻塞
uint8 trajectory_connect # 0=立即规划, 1=与下一条轨迹一起规划
uint8 dof                # 自由度
```

### 直线运动 Movel.msg
```
geometry_msgs/Pose pose  # 位姿 (m + 四元数)
uint8 speed              # 速度比例 0-100
uint8 trajectory_connect # 0=立即, 1=合并规划
bool block               # 阻塞模式
```

### 圆弧运动 Movec.msg
```
geometry_msgs/Pose pose_mid   # 中间位姿
geometry_msgs/Pose pose_end   # 目标位姿
uint8 speed                   # 速度比例 0-100
uint8 trajectory_connect      # 0=立即, 1=合并规划
bool block                    # 阻塞模式
uint8 loop                    # 循环次数
```

### 关节空间到位姿 Movejp.msg
```
geometry_msgs/Pose pose  # 目标位姿
uint8 speed              # 速度比例 0-100
uint8 trajectory_connect # 0=立即, 1=合并规划
bool block               # 阻塞模式
uint8 dof                # 自由度
```

### 关节错误代码 Jointerrorcode.msg
```
uint16[] joint_error     # 各关节错误码
uint8 dof                # 自由度
```

### 获取所有坐标系 Getallframe.msg
```
string[10] frame_name    # 坐标系名称数组 (最多10个)
```

### 清除关节错误 Jointerrclear.msg
```
uint8 joint_num          # 关节序号 (1-6 或 1-7)
```
