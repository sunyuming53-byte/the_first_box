# rm_moveit2_config 功能包

Moveit2 适配包，实现虚拟机械臂和真实机械臂的规划控制。

GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_moveit2_config

## 1. 使用

### Moveit2 控制虚拟机械臂（仅 rviz2）

```bash
# 标准版
ros2 launch rm_<arm_type>_config demo.launch.py

# 六维力版 (eco63/gen72/gen72_II 不可用)
ros2 launch rm_<arm_type>_config demo_6f.launch.py

# 一体化六维力版 (gen72/gen72_II 不可用)
ros2 launch rm_<arm_type>_config demo_6fb.launch.py
```

启动后拖动控制球到目标位置 → 点击"规划执行"。

### Moveit2 控制真实机械臂（四步启动）

```bash
# 第1步: 驱动
ros2 launch rm_driver rm_<arm_type>_driver.launch.py

# 第2步: 模型
ros2 launch rm_description rm_<arm_type>_display.launch.py

# 第3步: 通信桥
ros2 launch rm_control rm_<arm_type>_control.launch.py

# 第4步: Moveit2
ros2 launch rm_<arm_type>_config real_moveit_demo.launch.py
```

## 2. 架构

功能包采用**子包模式**，每个型号对应一个子包:

```
├── rm_63_config/          # RML63 moveit2 配置
│   ├── config/            # joint_limits.yaml, kinematics.yaml, *.srdf, *.xacro
│   └── launch/            # demo, real_moveit_demo, gazebo_moveit_demo
├── rm_65_config/          # RM65 moveit2 配置
├── rm_75_config/          # RM75 moveit2 配置
├── rm_eco65_config/       # ECO65 moveit2 配置
├── rm_eco63_config/       # ECO63 moveit2 配置
├── rm_gen72_config/       # GEN72 moveit2 配置
└── rm_gen72_II_config/    # GEN72-II moveit2 配置
```

## 3. 话题

- 发布/订阅标准 moveit2 话题（`/joint_states`, `/move_group` 等）
- Action Server: `/follow_joint_trajectory`
- 通过 `/joint_trajectory` action 与 rm_control 联动
