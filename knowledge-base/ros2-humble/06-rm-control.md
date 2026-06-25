# rm_control 功能包

Moveit2 与 rm_driver 之间的通信桥接包。将 Moveit2 规划的路径点用**三次样条插值**细分后，以透传方式发送给 rm_driver。

GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_control

## 1. 使用

```bash
ros2 launch rm_control rm_<arm_type>_control.launch.py
```

`<arm_type>`: `65`, `63`, `63_III`, `eco65`, `eco63`, `75`, `gen72`

**注意**: 单独启动无实际作用，需配合 rm_driver + moveit2 节点使用。

### 可配置参数（在 launch 文件中）

- `follow`: 跟随模式 — `true`: 高跟随（精细控制，需仔细计算参数）| `false`: 低跟随（简单但有丢点可能）
- `arm_type`: 型号 — `65`, `651`(eco65), `634`(eco63), `632`(63), `75`, `72`(GEN72)

## 2. 架构

```
├── CMakeLists.txt
├── include/
│   ├── cubicSpline.h              # 三次样条插值
│   └── rm_control.h
├── launch/
│   ├── rm_63_control.launch.py
│   ├── rm_65_control.launch.py
│   └── ...
├── package.xml
└── src/rm_control.cpp
```

## 3. 话题

**Publisher**（发布给 rm_driver）:
- `/rm_driver/movej_canfd_cmd` (`rm_ros_interfaces/msg/Jointpos`) — 细分后的路径点

**Action Server**（与 Moveit2 通信）:
- `/rm_group_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) — 接收 Moveit2 规划的轨迹

**数据流**:
```
Moveit2 (轨迹规划) → Action → rm_control (插值细分) → Topic → rm_driver (透传) → 机械臂
```
