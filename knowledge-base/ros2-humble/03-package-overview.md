# 功能包总览

ROS2 Humble 共提供 10 个功能包:

| # | 包名 | 类型 | 作用 |
|---|------|------|------|
| 1 | rm_install | 辅助 | 环境安装脚本、依赖安装、编译方法 |
| 2 | rm_driver | 驱动 | 底层驱动，TCP 与机械臂通信，订阅/发布 ROS2 话题 |
| 3 | rm_bringup | 启动 | 多节点聚合 launch，一键启动 Moveit2+驱动+模型 |
| 4 | rm_description | 模型 | URDF 模型文件 + TF 变换 + rviz2 配置 |
| 5 | rm_ros_interfaces | 消息 | 所有 .msg 消息定义（43个消息文件） |
| 6 | rm_moveit2_config | 控制 | Moveit2 适配（虚拟+真实机械臂规划控制） |
| 7 | rm_control | 通信桥 | Moveit2→rm_driver 路径细分+透传 |
| 8 | rm_gazebo | 仿真 | Gazebo 仿真环境 + 虚拟机械臂 |
| 9 | rm_example | 示例 | 基本控制功能示例（MoveJ/L/C、坐标系、夹爪等） |
| 10 | rm_doc | 文档 | 功能包使用说明文档 |

## 数据流关系

```
真实机械臂控制:
  Moveit2 (rm_moveit2_config)
    → Action /follow_joint_trajectory
    → rm_control (插值细分)
    → Topic /rm_driver/movej_canfd_cmd
    → rm_driver (TCP透传)
    → 机械臂本体

  rm_description → TF 变换 → rviz2 显示

仿真控制:
  Moveit2 (rm_moveit2_config)
    → rm_gazebo (Gazebo 仿真)
    → 虚拟机械臂

快速启动:
  rm_bringup = rm_driver + rm_description + rm_control + rm_moveit2_config (一键)
```

## 依赖关系

```
rm_ros_interfaces (基础消息)
    ↓
rm_driver (依赖 API 动态库 libRM_Service.so)
    ↓
rm_description (依赖 URDF 模型)
    ↓
rm_control (依赖 rm_driver + rm_ros_interfaces)
    ↓
rm_moveit2_config (依赖 rm_description + rm_control)
    ↓
rm_bringup (聚合以上所有)
    ↓
rm_gazebo (依赖 rm_moveit2_config)
rm_example (依赖 rm_driver)
```
