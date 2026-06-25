# 环境安装详解

## 安装清单

- ROS2 Humble
- Moveit2
- colcon 编译工具
- RM 机械臂 API 动态库 (libRM_Service.so)
- Gazebo (可选)

## 自动化安装

三个脚本位于 `ros2_rm_robot/rm_install/scripts/` 和 `ros2_rm_robot/rm_driver/lib/`:

```bash
# 1. ROS2 Humble
cd ~/ros2_rm_robot/rm_install/scripts/
sudo bash ros2_install.sh

# 2. Moveit2
sudo bash moveit2_install.sh

# 3. API 库
cd ~/ros2_rm_robot/rm_driver/lib/
sudo bash lib_install.sh
```

## 编译流程

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cp -r ros2_rm_robot ~/ros2_ws/src
cd ~/ros2_ws

# 先编译消息接口（其他包依赖它）
colcon build --packages-select rm_ros_interfaces
source ./install/setup.bash

# 编译全部
colcon build
source ./install/setup.bash
```

## 手动安装参考

- ROS2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- Moveit2: https://moveit.ros.org/install-moveit2/binary/
