# Humble Demo: 获取机械臂状态 Get_Arm_State_Demo

演示获取控制器版本、关节状态、位姿状态、六维力信息。

## 环境

- 系统: Ubuntu 22.04
- ROS: humble
- 依赖: ros2_rm_robot (humble 分支)

## 代码结构

```
├── CMakeLists.txt
├── launch/
│   └── get_arm_state_demo.launch.py
├── src/api_Get_Arm_State_demo.cpp
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

# 单独编译 demo 包
mkdir -p ~/demo_ws/src
cd ~/demo_ws/src
# 将 get_arm_state 源码放入 src
colcon build --packages-select get_arm_state
source ~/demo_ws/install/setup.bash
```

## 运行

```bash
# 终端1: 启动驱动
ros2 launch rm_driver rm_65_driver.launch.py

# 终端2: 获取状态
ros2 launch get_arm_state get_arm_state_demo.launch.py
```

## 预期输出

```
[INFO] joint state is: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # 关节角度 (rad)
[INFO] pose state is: [0.0, 0.0, 0.534, 3.141, 0.0, 0.0]       # 末端位姿 (欧拉角 XYZ-RPY)
[INFO] pose state is:                                            # 末端位姿 (四元数)
       position.x = 0.0
       position.y = 0.0
       position.z = 0.534
       orientation.x = 1.0, y = 0.0, z = 0.0, w = 0.000296
[INFO] Planversion is 7B0156                                     # 控制器版本
       (7=自由度, B=无六维力, 156=程序版本1.5.6)
[INFO] Productversion is GEN72-BI                                # 设备型号
```

## 关键代码: 状态订阅

```cpp
// 订阅关节状态
rclcpp::Subscription<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_state_sub_;

// 订阅位姿状态 (欧拉角)
rclcpp::Subscription<rm_ros_interfaces::msg::Jointposeeuler>::SharedPtr pose_state_sub_;

// 订阅臂状态
rclcpp::Subscription<rm_ros_interfaces::msg::Armstate>::SharedPtr arm_state_sub_;

// 订阅六维力
rclcpp::Subscription<rm_ros_interfaces::msg::Sixforce>::SharedPtr six_force_sub_;

// 订阅控制器版本
rclcpp::Subscription<rm_ros_interfaces::msg::Armoriginalstate>::SharedPtr plan_version_sub_;
```
