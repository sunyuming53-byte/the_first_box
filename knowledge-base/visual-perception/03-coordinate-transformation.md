# 03 — 坐标变换

> 综合自 YOLOV8 Demo 和手眼标定文档中的坐标变换代码

## 1. 核心公式

一切坐标变换最终都可以归结为**齐次变换矩阵的后乘法链式乘法**。

### Eye-in-Hand

```
p_base = H_ee_base * H_cam_ee * p_cam

  H_ee_base: 机械臂末端 → 基座（实时，从 API 获取）
  H_cam_ee:  相机 → 末端（手眼标定结果，固定常量）
  p_cam:     物体在相机坐标系下的齐次坐标 [x, y, z, 1]^T
```

### Eye-to-Hand

```
p_base = H_cam_base * p_cam

  H_cam_base: 相机 → 基座（手眼标定结果，固定常量）
  p_cam:      物体在相机坐标系下的齐次坐标 [x, y, z, 1]^T
```

## 2. 物体为 3D 点时的变换

### Eye-in-Hand

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

# 手眼标定结果（常量）
rotation_matrix = np.array([
    [-0.00235395,  0.99988123, -0.01523124],
    [-0.99998543, -0.00227965,  0.00489370],
    [ 0.00485839,  0.01524254,  0.99987202]
])
translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])

def convert(x, y, z, x1, y1, z1, rx, ry, rz):
    """
    物体 3D 坐标: 相机系 → 机械臂基坐标系
    
    输入:
      x, y, z         — 物体在相机坐标系下的位置（视觉识别结果）
      x1..rz          — 机械臂末端的位姿（从 API 获取，单位弧度）
    
    输出:
      [x, y, z]       — 物体在机械臂基坐标系下的位置
    """
    obj_camera = np.array([x, y, z])
    end_effector_pose = np.array([x1, y1, z1, rx, ry, rz])

    # Step 1: 手眼标定 → 相机→末端 的齐次变换矩阵
    T_cam2ee = np.eye(4)
    T_cam2ee[:3, :3] = rotation_matrix
    T_cam2ee[:3, 3]  = translation_vector

    # Step 2: 末端位姿 → 末端→基座 的齐次变换矩阵
    pos = end_effector_pose[:3]
    ori = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
    T_ee2base = np.eye(4)
    T_ee2base[:3, :3] = ori
    T_ee2base[:3, 3]  = pos

    # Step 3: 链式变换
    p_cam_homo = np.append(obj_camera, [1])
    p_ee = T_cam2ee.dot(p_cam_homo)
    p_base = T_ee2base.dot(p_ee)

    return list(p_base[:3])
```

### Eye-to-Hand

```python
def convert(x, y, z):
    """
    物体 3D 坐标: 相机系 → 机械臂基坐标系（眼在手外）
    
    输入:
      x, y, z — 物体在相机坐标系下的位置
    
    输出:
      [x, y, z] — 物体在机械臂基坐标系下的位置
    """
    obj_camera = np.array([x, y, z])

    T_cam2base = np.eye(4)
    T_cam2base[:3, :3] = rotation_matrix   # 手眼标定结果
    T_cam2base[:3, 3]  = translation_vector

    p_cam_homo = np.append(obj_camera, [1])
    p_base = T_cam2base.dot(p_cam_homo)

    return list(p_base[:3])
```

## 3. 物体为 6D 位姿时的变换

当视觉识别输出物体的完整 6D 位姿（含旋转）时：

### Eye-in-Hand

```python
def decompose_transform(matrix):
    """齐次矩阵 → (平移, rx, ry, rz)"""
    translation = matrix[:3, 3]
    rotation = matrix[:3, :3]
    sy = np.sqrt(rotation[0,0]**2 + rotation[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        rx = np.arctan2(rotation[2,1], rotation[2,2])
        ry = np.arctan2(-rotation[2,0], sy)
        rz = np.arctan2(rotation[1,0], rotation[0,0])
    else:
        rx = np.arctan2(-rotation[1,2], rotation[1,1])
        ry = np.arctan2(-rotation[2,0], sy)
        rz = 0
    return translation, rx, ry, rz

def convert(x, y, z, rx, ry, rz, x1, y1, z1, rx1, ry1, rz1):
    """
    物体 6D 位姿: 相机系 → 机械臂基坐标系
    
    输入:
      x..rz          — 物体在相机坐标系下的位姿（视觉识别结果）
      x1..rz1        — 机械臂末端的位姿（从 API 获取）
    
    输出:
      (translation, rx, ry, rz) — 物体在基坐标系下的位姿
    """
    obj_camera = np.array([x, y, z, rx, ry, rz])
    end_effector_pose = np.array([x1, y1, z1, rx1, ry1, rz1])

    # 手眼标定矩阵
    T_cam2ee = np.eye(4)
    T_cam2ee[:3, :3] = rotation_matrix
    T_cam2ee[:3, 3]  = translation_vector

    # 末端位姿 → 齐次矩阵
    pos = end_effector_pose[:3]
    ori = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
    T_ee2base = np.eye(4)
    T_ee2base[:3, :3] = ori
    T_ee2base[:3, 3]  = pos

    # 物体位姿 → 齐次矩阵
    pos2 = obj_camera[:3]
    ori2 = R.from_euler('xyz', obj_camera[3:], degrees=False).as_matrix()
    T_obj2cam = np.eye(4)
    T_obj2cam[:3, :3] = ori2
    T_obj2cam[:3, 3]  = pos2

    # 链式: base ← ee ← cam ← obj
    T_obj2ee = T_cam2ee.dot(T_obj2cam)
    T_obj2base = T_ee2base.dot(T_obj2ee)

    return decompose_transform(T_obj2base)
```

### Eye-to-Hand

```python
def convert(x, y, z, rx, ry, rz):
    """
    物体 6D 位姿: 相机系 → 机械臂基坐标系（眼在手外）
    """
    obj_camera = np.array([x, y, z, rx, ry, rz])

    T_cam2base = np.eye(4)
    T_cam2base[:3, :3] = rotation_matrix
    T_cam2base[:3, 3]  = translation_vector

    pos = obj_camera[:3]
    ori = R.from_euler('xyz', obj_camera[3:], degrees=False).as_matrix()
    T_obj2cam = np.eye(4)
    T_obj2cam[:3, :3] = ori
    T_obj2cam[:3, 3]  = pos

    T_obj2base = T_cam2base.dot(T_obj2cam)
    return decompose_transform(T_obj2base)
```

## 4. YOLOV8 Demo 中的坐标系关系

```
标定板
  ↑ H_board_cam (相机外参)
相机 (固定在末端)
  ↑ H_cam_ee (手眼标定结果 X)
末端
  ↑ H_ee_base (机械臂 API 实时获取)
基座
```

YOLOV8 Demo 中 `convert()` 的逻辑与标准 Eye-in-Hand 一致：
1. D435 返回 `(x, y, z)` = 物体在相机坐标系的 3D 坐标
2. `rotation_matrix + translation_vector` = 手眼标定得到的 `H_cam_ee`
3. 从 `/rm_driver/Arm_Current_State` 获取末端位姿 → `H_ee_base`
4. 后乘得到基坐标系下的物体位置

## 5. 欧拉角 → 旋转矩阵

手眼标定代码中使用 Z-Y-X 欧拉角顺序：

```python
def euler_angles_to_rotation_matrix(rx, ry, rz):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                   [ 0,          1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0,           0,          1]])
    return Rz @ Ry @ Rx
```

## 6. 坐标系命名约定

| 缩写 | 全称 |
|------|------|
| `cam` / `c` | 相机坐标系 (Camera) |
| `ee` / `end` | 末端坐标系 (End-Effector) |
| `base` / `rob` / `b` | 机械臂基坐标系 (Base) |
| `obj` | 物体坐标系 (Object) |
| `board` | 标定板坐标系 (Calibration Board) |

## 7. 与 ROS2 仓库的关系

坐标变换的数学逻辑是纯 Python/NumPy 实现，不依赖 ROS 版本。在本仓库中复用只需：

1. 将 `rotation_matrix` 和 `translation_vector` 保存为配置常量
2. 从 `rm::Arm` 获取当前末端位姿（替代 ROS1 topic 读取）
3. 其余数学部分直接移植
