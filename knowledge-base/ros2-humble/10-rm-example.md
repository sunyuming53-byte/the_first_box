# rm_example 功能包

提供机械臂基本控制功能的示例代码。

GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_example

## 使用示例

所有示例都需要**先启动 rm_driver**:
```bash
ros2 launch rm_driver rm_<arm_type>_driver.launch.py
```

### 1. 更换工作坐标系
```bash
ros2 run rm_example rm_change_work_frame
```

验证:
```bash
ros2 topic echo /rm_driver/get_curr_workFrame_result
ros2 topic pub --once /rm_driver/get_curr_workFrame_cmd std_msgs/msg/Empty "{}"
```

### 2. 获取机械臂状态
```bash
ros2 run rm_example rm_get_state
```
返回: 关节角度、末端位置姿态（欧拉角）

### 3. MoveJ 关节运动
```bash
# 6自由度
ros2 launch rm_example rm_6dof_movej.launch.py
# 7自由度
ros2 launch rm_example rm_7dof_movej.launch.py
```

### 4. MoveJ_P 位姿运动
```bash
ros2 run rm_example movejp_demo
# GEN72 特殊:
ros2 run rm_example movejp_gen72_demo
```

### 5. MoveL 直线运动
```bash
ros2 run rm_example movel_demo
# GEN72 特殊:
ros2 run rm_example movel_gen72_demo
```

### 6. MoveC 圆弧运动
```bash
ros2 run rm_example movec_demo
# GEN72 特殊:
ros2 run rm_example movec_gen72_demo
```

### 7. 机械臂回零
```bash
ros2 run rm_example rm_movej_zero
```

### 8. 透传跟随
```bash
# 关节透传
ros2 launch rm_example rm_6dof_canfd.launch.py
ros2 launch rm_example rm_7dof_canfd.launch.py

# 位姿透传
ros2 launch rm_example rm_6dof_movepcanfd.launch.py
ros2 launch rm_example rm_7dof_movepcanfd.launch.py
```

### 9. 升降机构控制
```bash
ros2 run rm_example rm_lift
```

### 10. 末端夹爪控制
```bash
ros2 run rm_example rm_gripper
```

### 11. 六维力控制
```bash
ros2 run rm_example rm_force_control
```

### 12. 样条曲线运动
```bash
ros2 launch rm_example rm_sio_move.launch.py
```

## 架构

```
├── CMakeLists.txt
├── launch/
│   ├── rm_6dof_movej.launch.py
│   ├── rm_7dof_movej.launch.py
│   ├── rm_6dof_canfd.launch.py
│   ├── rm_7dof_canfd.launch.py
│   └── ...
├── src/
│   ├── rm_change_work_frame.cpp
│   ├── rm_get_state.cpp
│   ├── movejp_demo.cpp
│   ├── movejp_gen72_demo.cpp
│   ├── movel_demo.cpp
│   ├── movel_gen72_demo.cpp
│   ├── movec_demo.cpp
│   ├── movec_gen72_demo.cpp
│   └── ...
└── package.xml
```
