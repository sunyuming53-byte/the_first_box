# rm_gazebo 功能包

Gazebo 仿真功能包，在仿真环境中搭建虚拟机械臂，配合 Moveit2 进行仿真控制。

GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_gazebo

## 1. 使用

### 启动 Gazebo 仿真机械臂

```bash
# 标准版
ros2 launch rm_gazebo gazebo_<arm_type>_demo.launch.py

# 六维力版 (eco63/gen72/gen72_II 不可用)
ros2 launch rm_gazebo gazebo_<arm_type>_6f_demo.launch.py

# 一体化六维力版 (gen72/gen72_II 不可用)
ros2 launch rm_gazebo gazebo_<arm_type>_6fb_demo.launch.py
```

`<arm_type>`: `65`, `63`, `63_III`, `eco65`, `eco63`, `75`, `gen72`, `gen72_II`

### 启动 Moveit2 控制仿真机械臂

```bash
ros2 launch rm_<arm_type>_config gazebo_moveit_demo.launch.py
ros2 launch rm_<arm_type>_config gazebo_moveit_demo_6f.launch.py
ros2 launch rm_<arm_type>_config gazebo_moveit_demo_6fb.launch.py
```

例如 RM65:
```bash
ros2 launch rm_gazebo gazebo_65_demo.launch.py
ros2 launch rm_65_config gazebo_moveit_demo.launch.py
```

## 2. 架构

```
├── CMakeLists.txt
├── config/                         # 各型号 Gazebo URDF 描述
│   ├── gazebo_65_description.urdf.xacro
│   ├── gazebo_65_6fb_description.urdf.xacro
│   └── ... (所有型号+变体)
├── launch/                         # 启动文件
│   ├── gazebo_65_demo.launch.py
│   ├── gazebo_65_6f_demo.launch.py
│   ├── gazebo_65_6fb_demo.launch.py
│   └── ... (所有型号+变体)
└── package.xml
```
