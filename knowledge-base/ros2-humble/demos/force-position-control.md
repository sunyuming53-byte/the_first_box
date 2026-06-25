# Humble Demo: 力位混合控制规划

演示力位混合控制功能，适用于笛卡尔运动（MoveL 等），不适用于关节运动（MoveJ）。

## 环境

- 系统: Ubuntu 22.04
- ROS: humble
- 依赖: ros2_rm_robot (humble 分支)
- 适用: RM65、RM75 (其他型号可能点位不可达)

## 代码结构

```
├── CMakeLists.txt
├── launch/
│   └── force_position_control_demo.launch
├── src/api_force_position_control_demo.cpp
└── package.xml
```

## 运行

```bash
# 终端1: 启动驱动
ros2 launch rm_driver rm_65_driver.launch.py

# 终端2: 启动力位混合控制 demo
ros2 launch force_position_control force_position_control_demo.launch.py
```

## 执行流程

1. MoveJ_P 运动到起始位姿
2. 开启力位混合控制
3. MoveL 笛卡尔运动
4. 关闭力位混合控制

## 预期输出

```
[INFO] *******Movej_p succeeded                     # 起始位姿到达
[INFO] *******Set Force Postion succeeded           # 力位混合控制设置成功
[INFO] *******MoveL succeeded                       # 笛卡尔运动完成
[INFO] *******Stop Force Postion succeeded          # 力位混合控制关闭
[INFO] *******All step run over                     # 全部完成
```

## 关键代码: 订阅/发布器

```cpp
// 订阅器
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_p_subscription_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_subscription_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_force_postion_subscription_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_force_postion_subscription_;

// 发布器
rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movej_p_publisher_;
rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_publisher_;
rclcpp::Publisher<rm_ros_interfaces::msg::Setforceposition>::SharedPtr set_force_position_publisher_;
```
