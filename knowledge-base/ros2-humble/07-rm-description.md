# rm_description 功能包

显示机械臂模型和 TF 变换，提供 URDF 文件和 rviz2 配置，支持虚拟与现实联动。

GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_description

## 1. 使用

### 启动模型显示

```bash
# 标准版
ros2 launch rm_description rm_<arm_type>_display.launch.py

# 六维力版 (eco63/gen72/gen72_II 不可用)
ros2 launch rm_description rm_<arm_type>_6f_display.launch.py

# 一体化六维力版 (gen72/gen72_II 不可用)
ros2 launch rm_description rm_<arm_type>_6fb_display.launch.py
```

`<arm_type>`: `65`, `63`, `63_III`, `eco65`, `eco63`, `75`, `gen72`, `gen72_II`

### 配合驱动查看真实状态

```bash
# 终端1: 启动模型
ros2 launch rm_description rm_65_display.launch.py
# 终端2: 启动驱动
ros2 launch rm_driver rm_65_driver.launch.py
# 终端3: 启动 rviz2，加载 rm_description/rviz/ 下的配置文件
rviz2
```

## 2. 架构

```
├── CMakeLists.txt
├── launch/                        # 各型号+变体的 display launch
├── meshes/                        # STL 模型文件
│   ├── rm_63_arm/                 # base_link.STL, link1~6.STL
│   ├── rm_65_arm/
│   ├── rm_75_arm/                 # link7.STL (七轴)
│   ├── rm_eco65_arm/
│   ├── rm_eco63_arm/
│   └── rm_gen72_arm/
├── rviz/                          # rviz2 配置文件
│   ├── rm_63.rviz, rm_65.rviz, rm_75.rviz, ...
├── urdf/
│   ├── rm_65.urdf / rm_65.urdf.xacro    # 标准 URDF
│   ├── rm_65_6f.urdf                    # 六维力版本
│   ├── rm_65_6fb.urdf                   # 一体化六维力版本
│   ├── rm_65_gazebo.urdf / .xacro       # Gazebo 仿真用 URDF
│   └── ... (覆盖所有型号)
└── package.xml
```

## 3. 话题

主要发布 TF 变换和 robot_description 参数，供 rviz2 和 moveit2 使用。
