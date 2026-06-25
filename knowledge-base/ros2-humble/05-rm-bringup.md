# rm_bringup 功能包

批量启动多个节点的快速启动包。一条命令启动 `rm_driver` + `rm_description` + `rm_control` + `moveit2`。

GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_bringup

## 1. 使用

### Moveit2 控制真实机械臂

```bash
# 标准版
ros2 launch rm_bringup rm_<arm_type>_bringup.launch.py

# 六维力版 (eco63/gen72/gen72_II 不可用)
ros2 launch rm_bringup rm_<arm_type>_6f_bringup.launch.py

# 一体化六维力版 (gen72/gen72_II 不可用)
ros2 launch rm_bringup rm_<arm_type>_6fb_bringup.launch.py
```

### 控制 Gazebo 仿真机械臂

```bash
ros2 launch rm_bringup rm_<arm_type>_gazebo.launch.py
ros2 launch rm_bringup rm_<arm_type>_6f_gazebo.launch.py
ros2 launch rm_bringup rm_<arm_type>_6fb_gazebo.launch.py
```

`<arm_type>`: `65`, `63`, `63_III`, `eco65`, `eco63`, `75`, `gen72`, `gen72_II`

## 2. 架构

```
├── CMakeLists.txt
├── launch/                         # 大量 launch 文件，覆盖所有型号+变体
│   ├── rm_65_bringup.launch.py     # moveit2 真实控制
│   ├── rm_65_gazebo.launch.py      # gazebo 仿真控制
│   ├── rm_65_6f_bringup.launch.py  # 六维力 moveit2
│   ├── rm_65_6fb_bringup.launch.py # 一体化六维力 moveit2
│   └── ...（所有型号 × 变体组合）
└── package.xml
```

## 3. 话题

该功能包自身**不产生话题**，仅为其他功能包的 launch 聚合。
