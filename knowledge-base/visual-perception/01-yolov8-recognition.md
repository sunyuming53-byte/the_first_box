# 01 — YOLOV8 视觉识别管线

> 来源: https://develop.realman-robotics.com/symbiosis/demo/YOLOV8VisualRecognition/
> 代码仓库: https://github.com/RealManRobot/YOLOv8-Visual-Recognition

## 1. 硬件配置

| 组件 | 型号 | 连接方式 |
|------|------|----------|
| 机械臂 | RM65-B（视觉版，末端集成 D435）/ RM65-B + 视觉转接板 | 网线 → 主控 |
| 深度相机 | Intel RealSense D435 | USB → 主控 |
| 夹爪 | 因时 EG2-4C2 两指电动夹爪 | 末端 6 芯接口 |
| 主控 | ARM (Jetson Xavier NX) 或 X86 PC | — |

## 2. 软件环境

- **OS**: Ubuntu 20.04 + ROS Noetic
- **Python**: 3.8+（推荐 conda 虚拟环境）
- **CUDA**: Jetson Xavier NX 自带 JetPack

**Python 依赖**:
```
pip3 install -r requirements.txt
pip3 install ultralytics
```

**D435 驱动**: librealsense2 + pyrealsense2（ARM 架构需源码编译）

## 3. 工程结构

```
src/
├── rm_robot/          # 机械臂 ROS 包（替换为最新版）
├── vi_grab/           # 视觉抓取功能包
│   ├── launch/
│   │   └── vi_grab_demo.launch    # 启动文件（4 个节点）
│   ├── scripts/
│   │   ├── vi_catch_yolov8.py     # YOLOV8 检测 + ROS 发布节点
│   │   ├── vision_grab.py         # 抓取执行脚本
│   │   └── vision_pour_water_modbus.py
│   ├── model/          # YOLOV8 模型文件
│   └── CMakeLists.txt
└── vi_msgs/           # 自定义消息包
    └── msg/
        └── ObjectInfo.msg
```

## 4. 自定义消息

`ObjectInfo.msg`:
```
string object_class    # 物体类别（如 "bottle"）
float64 x              # 物体在相机坐标系下的 X (m)
float64 y              # 物体在相机坐标系下的 Y (m)
float64 z              # 物体在相机坐标系下的 Z (m)
```

## 5. 四个 ROS 节点（由 `vi_grab_demo.launch` 启动）

| 节点 | 功能 | 关键话题 |
|------|------|----------|
| `msg_pub` | 主动获取机械臂状态 | — |
| `robot_driver` | 机械臂功能启动 | — |
| `object_detect` | YOLOV8 推理 + 3D 坐标发布 | `/object_pose` (pub), `/choice_object` (sub) |
| `object_catch` | 坐标变换 + 分段抓取执行 | — |

## 6. 视觉识别节点核心流程

```
D435 采集 RGB-D 帧
  ↓
YOLOV8 推理 (model.predict, conf=0.5)
  ↓
获取检测框中心点 (ux, uy)
  ↓
get_3d_camera_coordinate() → 查深度图 + 相机内参反投影
  ↓
得到相机坐标系 3D 坐标 (x, y, z)
  ↓
发布 ObjectInfo 到 /object_pose 话题
```

**关键代码片段**:
```python
model = YOLO('model/yolov8n.pt')
results = model.predict(color_image, conf=0.5)
for box in results[0].boxes.xyxy:
    x1, y1, x2, y2 = map(int, box)
    ux, uy = int((x1+x2)/2), int((y1+y2)/2)
    dis, camera_coordinate = get_3d_camera_coordinate(
        [ux, uy], aligned_depth_frame, depth_intrin
    )
    # 发布
    object_info_msg.object_class = name
    object_info_msg.x, .y, .z = camera_coordinate
    object_pub.publish(object_info_msg)
```

## 7. 抓取执行流程

### 用户触发
```bash
# 发布需要抓取的物体名（COCO 类别）
rostopic pub /choice_object std_msgs/String "bottle"
```

### 抓取逻辑（四段式）

```
1. movej_p → 物体前方 7cm（避免碰撞）
     ↓
2. movel   → 直线运动到物体精确位置
     ↓
3. gripper_close → 闭合夹爪
     ↓
4. movel 上抬 5cm → 旋转末端关节倒水
```

**关键约束**: 抓取前需手动将机械臂示教到初始姿态，保持末端水平。

## 8. 坐标变换入口

`vision_grab.py` 中的 `convert()` 函数负责将相机坐标系物体坐标变换到机械臂基坐标系：

```python
def convert(x, y, z, x1, y1, z1, rx, ry, rz):
    """
    输入:
      (x, y, z)      → 物体在相机坐标系下坐标
      (x1..rz)       → 机械臂末端位姿（从 /rm_driver/Arm_Current_State 获取）
    输出:
      (x, y, z, rx, ry, rz) → 物体在机械臂基坐标系下位姿
    """
    # T_camera_to_end_effector: 手眼标定结果（rotation_matrix + translation_vector）
    # T_base_to_end_effector: 当前机械臂末端位姿 → 齐次矩阵
    # obj_base = T_base_to_end_effector * T_camera_to_end_effector * obj_camera
```

详细推导见 [03-coordinate-transformation.md](./03-coordinate-transformation.md)。

## 9. 与 ROS2 迁移关键点

| ROS1 话题 | 作用 | ROS2 等价 |
|-----------|------|-----------|
| `/rm_driver/Arm_Current_State` | 获取当前臂位姿 | ROS2 Service 或 Topic（由 `rm_driver` 包提供） |
| `/rm_driver/ArmCurrentState` | 获取四元数位姿 | 同上 |
| `/object_pose` (ObjectInfo) | 识别结果发布 | 需创建等价 ROS2 msg |
| `/choice_object` (String) | 用户指定目标 | 同上 |

手眼标定结果（`rotation_matrix` + `translation_vector`）为纯数学常量，ROS 版本无关，可直接复用。
