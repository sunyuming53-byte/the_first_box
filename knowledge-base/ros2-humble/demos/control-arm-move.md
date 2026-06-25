# Humble Demo: 机械臂运动控制 Arm_Move_Demo

演示 MoveJ、MoveJ_P、MoveL、MoveC 四种规划运动。

## 环境

- 系统: Ubuntu 22.04
- ROS: humble
- 依赖: ros2_rm_robot (humble 分支)

## 代码结构

```
├── CMakeLists.txt
├── launch/
│   ├── rm_65_move.launch.py      # RM65 启动
│   └── rm_75_move.launch.py      # RM75 启动
├── src/api_Move_demo.cpp         # 源码
└── package.xml
```

## 环境配置

```bash
mkdir -p ~/ros2_ws/src
cp -r ros2_rm_robot ~/ros2_ws/src
cd ~/ros2_ws
colcon build --packages-select rm_ros_interfaces
source ./install/setup.bash
colcon build
source ./install/setup.bash
```

## 运行

```bash
# 终端1: 启动驱动
ros2 launch rm_driver rm_65_driver.launch.py

# 终端2: 运行运动控制 demo
ros2 launch control_arm_move rm_65_move.launch.py
# 或 RM75:
ros2 launch control_arm_move rm_75_move.launch.py
```

## 预期输出

```
[INFO] arm_dof is 7                    # 自由度提示
[INFO] *******Movej succeeded          # MoveJ 成功
[INFO] *******Movej_p succeeded        # MoveJ_P 成功
[INFO] *******MoveL succeeded          # MoveL 成功
[INFO] *******MoveC succeeded          # MoveC 成功
```

## 关键代码: 消息发布器

```cpp
// MoveJ_P 发布器
rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movej_p_publisher_;

// MoveL 发布器
rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;

// MoveJ 发布器
rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr movej_publisher_;

// MoveC 发布器
rclcpp::Publisher<rm_ros_interfaces::msg::Movec>::SharedPtr movec_publisher_;
```
