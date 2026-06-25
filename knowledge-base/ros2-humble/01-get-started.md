# ROS2 Humble 快速开始

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Moveit2
- 机械臂型号: RM65/75/ECO63/ECO65/RML63/GEN72
- GitHub: [ros2_rm_robot (humble分支)](https://github.com/RealManRobot/ros2_rm_robot/tree/humble)

## 1. 搭建环境

### 1.1 安装 ROS2

```bash
cd ~/ros2_rm_robot/rm_install/scripts/
sudo bash ros2_install.sh
```

手动安装参考: [ROS2 Humble Install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 1.2 安装 Moveit2

```bash
cd ~/ros2_rm_robot/rm_install/scripts/
sudo bash moveit2_install.sh
```

手动安装参考: [Moveit2 Install](https://moveit.ros.org/install-moveit2/binary/)

### 1.3 配置功能包环境

```bash
cd ~/ros2_rm_robot/rm_driver/lib/
sudo bash lib_install.sh
```

### 1.4 编译

```bash
mkdir -p ~/ros2_ws/src
cp -r ros2_rm_robot ~/ros2_ws/src
cd ~/ros2_ws
colcon build --packages-select rm_ros_interfaces
source ./install/setup.bash
colcon build
source ./install/setup.bash
```

## 2. 功能运行

### 2.1 运行虚拟机械臂（Gazebo + Moveit2）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rm_gazebo gazebo_<arm_type>_demo.launch.py
ros2 launch rm_<arm_type>_config gazebo_moveit_demo.launch.py
```

`<arm_type>` 替换为: `65`, `75`, `eco65`, `eco63`, `63`, `63_III`, `gen72`, `gen72_II`

例如 RM65:
```bash
ros2 launch rm_gazebo gazebo_65_demo.launch.py
ros2 launch rm_65_config gazebo_moveit_demo.launch.py
```

### 2.2 控制真实机械臂

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rm_bringup rm_<arm_type>_bringup.launch.py
```

例如 RM65:
```bash
ros2 launch rm_bringup rm_65_bringup.launch.py
```

## 3. 功能包列表

| 序号 | 功能包 | 作用 |
|------|--------|------|
| 1 | rm_install | 环境安装与配置 |
| 2 | rm_driver | 底层驱动，订阅/发布机械臂话题 |
| 3 | rm_bringup | 多节点快速启动 |
| 4 | rm_description | 模型描述，提供 URDF 与 TF 变换 |
| 5 | rm_ros_interfaces | ROS2 消息文件 (.msg) |
| 6 | rm_moveit2_config | Moveit2 适配配置（虚拟/真实控制） |
| 7 | rm_control | Moveit2 与驱动通信桥（路径细分+透传） |
| 8 | rm_gazebo | Gazebo 仿真 |
| 9 | rm_example | 使用案例示例 |
| 10 | rm_doc | 技术文档 |

## 4. 安全提示

- 使用机械臂时需遵守操作规范
- 确保安全距离，避免碰撞
- 仿真先行验证再控制真实机械臂
