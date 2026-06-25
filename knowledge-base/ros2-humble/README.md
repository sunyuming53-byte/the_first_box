# RealMan 机械臂 ROS2 Humble 知识库

> 来源: https://develop.realman-robotics.com/robot/ros2/getStarted/
> ROS2 版本: Humble (Ubuntu 22.04)
> 最新版本: 1.6.0
> GitHub: https://github.com/RealManRobot/ros2_rm_robot/tree/humble

## 支持的机械臂型号

RM65系列、RM75系列、ECO63系列、ECO65系列、RML63系列、GEN72系列

## 文档目录

| 文档 | 内容 |
|------|------|
| [01-get-started.md](./01-get-started.md) | 快速开始：环境搭建、编译、运行虚拟/真实机械臂 |
| [02-environment-setup.md](./02-environment-setup.md) | 环境安装：ROS2、Moveit2、依赖库安装脚本 |
| [03-package-overview.md](./03-package-overview.md) | 10 个功能包总览及作用说明 |
| [04-rm-driver.md](./04-rm-driver.md) | rm_driver — 底层驱动功能包（话题/配置/架构） |
| [05-rm-bringup.md](./05-rm-bringup.md) | rm_bringup — 多节点快速启动功能包 |
| [06-rm-control.md](./06-rm-control.md) | rm_control — Moveit2 与驱动通信功能包 |
| [07-rm-description.md](./07-rm-description.md) | rm_description — 机械臂模型与TF变换 |
| [08-rm-moveit2-config.md](./08-rm-moveit2-config.md) | rm_moveit2_config — Moveit2 规划控制 |
| [09-rm-gazebo.md](./09-rm-gazebo.md) | rm_gazebo — Gazebo 仿真功能包 |
| [10-rm-example.md](./10-rm-example.md) | rm_example — 控制示例（MoveJ/MoveL/MoveC等） |
| [11-topic-description.md](./11-topic-description.md) | ROS2 话题说明、错误代码列表 |
| [12-ros-interfaces.md](./12-ros-interfaces.md) | 自定义消息文件 (.msg) 定义 |
| [demos/](./demos/) | Humble 专用 Demo 示例 |

## 快速上手速查

```bash
# 1. 安装依赖
cd ~/ros2_rm_robot/rm_install/scripts/
sudo bash ros2_install.sh     # 安装 ROS2 Humble
sudo bash moveit2_install.sh   # 安装 Moveit2

# 2. 配置 & 编译
cd ~/ros2_rm_robot/rm_driver/lib/
sudo bash lib_install.sh       # 安装 API 库

mkdir -p ~/ros2_ws/src
cp -r ros2_rm_robot ~/ros2_ws/src
cd ~/ros2_ws
colcon build --packages-select rm_ros_interfaces
source ./install/setup.bash
colcon build
source ./install/setup.bash

# 3. 运行虚拟机械臂
ros2 launch rm_gazebo gazebo_65_demo.launch.py      # 启动 Gazebo（以RM65为例）
ros2 launch rm_65_config gazebo_moveit_demo.launch.py # 启动 Moveit2

# 4. 控制真实机械臂（默认IP: 192.168.1.18）
ros2 launch rm_bringup rm_65_bringup.launch.py

# 5. 单独启动驱动
ros2 launch rm_driver rm_65_driver.launch.py
```

## 模型代号速查

| 模型 | launch 参数 | arm_type 配置 | DOF |
|------|------------|---------------|-----|
| RM65 | 65 | RM_65 | 6 |
| RM75 | 75 | RM_75 | 7 |
| ECO65 | eco65 | RM_eco65 | 6 |
| ECO63 | eco63 | RM_eco63 | 6 |
| RML63 | 63 | RML_63 | 6 |
| RML63-III | 63_III | RML_63 | 6 |
| GEN72 | gen72 | GEN_72 | 7 |
| GEN72-II | gen72_II | GEN_72 | 7 |
