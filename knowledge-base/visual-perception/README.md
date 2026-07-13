# Visual Perception Knowledge Base — OMRobot

机械臂视觉感知完整知识库，涵盖 YOLOV8 视觉识别、手眼标定、坐标变换三大核心模块。

> 来源：
> - [YOLOV8 视觉识别 Demo](https://develop.realman-robotics.com/symbiosis/demo/YOLOV8VisualRecognition/)
> - [手眼标定 SDK 开发指南](https://develop.realman-robotics.com/AI/developerGuide/hand/)

## 覆盖范围

| 模块 | 描述 | 原始生态 |
|------|------|----------|
| YOLOV8 视觉识别 | D435 相机 + YOLOV8 物体检测 + ROS 话题发布 | ROS1 Noetic (Ubuntu 20.04) |
| 手眼标定 | Eye-in-Hand / Eye-to-Hand 标定原理、采集流程、OpenCV 计算 | 独立 Python 脚本 |
| 坐标变换 | 物体坐标从相机坐标系到机械臂基坐标系的链式变换 | 通用（数学通用） |

## 文档目录

| 文档 | 内容 |
|------|------|
| [01-yolov8-recognition.md](./01-yolov8-recognition.md) | YOLOV8 视觉识别管线：硬件、ROS 节点、消息定义、抓取流程 |
| [02-hand-eye-calibration.md](./02-hand-eye-calibration.md) | 手眼标定：AX=XB 原理、两种配置、采集步骤、代码解析 |
| [03-coordinate-transformation.md](./03-coordinate-transformation.md) | 坐标变换：齐次矩阵推导、3D点与位姿变换公式、代码示例 |
| [index.md](./index.md) | 使用场景 → 文档映射、关键集成点 |

## 与本仓库的关系

当前仓库基于 **ROS2 Humble**，上述文档原始参考为 **ROS1 Noetic**。差异：

| 项目 | ROS1 文档 | ROS2 本仓库 |
|------|-----------|-------------|
| 通信 | Topic（`/rm_driver/Arm_Current_State`） | ROS2 Service |
| 视觉功能包 | `vi_grab` + `vi_msgs`（Python） | 暂无，需从 ROS1 迁移 |
| 手眼标定结果 | 通用（旋转矩阵 + 平移向量） | 可直接复用 |

## 快速速查

```python
# 物体抓取核心公式 (Eye-in-Hand)
p_base = H_ee_base * H_cam_ee * p_cam

# p_cam   → 相机识别到的物体 3D 坐标 (x, y, z)
# H_cam_ee → 手眼标定结果：相机→末端的齐次变换矩阵（固定值）
# H_ee_base→ 机械臂 API 返回的当前末端位姿（实时）
# p_base  → 物体在机械臂基坐标系下的位置
```
