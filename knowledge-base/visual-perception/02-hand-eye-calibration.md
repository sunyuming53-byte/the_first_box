# 02 — 手眼标定

> 来源: https://develop.realman-robotics.com/AI/developerGuide/hand/
> 代码仓库: https://github.com/RealManRobot/hand_eye_calibration

## 1. 概述

**手眼标定**将相机坐标系与机械臂坐标系统一，使机械臂能精确到达相机定位的目标。标定结果**不需要重复执行**，除非相机与机械臂的相对位置发生变动。

适用场景：机器人抓取、动态环境交互、精密测量、视觉伺服控制。

支持安装方式：正装、侧装、倒装。

## 2. 两种配置

### 2.1 Eye-in-Hand（眼在手上）

```
相机固定在机械臂末端，随臂运动。

硬件布局：
  机械臂基座（固定）
      │
      ▼
  机械臂末端 ←── 相机（固定连接）
      │              │
      ▼              ▼
   标定板（固定在工作台上）
```

**标定目标**: 相机 → 末端 的变换矩阵 `H_cam_end`（即 X）

### 2.2 Eye-to-Hand（眼在手外）

```
相机固定在机械臂外部。

硬件布局：
  机械臂基座（固定）  相机（固定）
      │                   │
      ▼                   ▼
  机械臂末端 ←── 标定板（固定在末端）
```

**标定目标**: 相机 → 基座 的变换矩阵 `H_cam_base`

## 3. AX = XB 数学推导

### Eye-in-Hand

设以下 4x4 齐次变换矩阵：

| 符号 | 含义 | 来源 |
|------|------|------|
| `H_end_base` (A) | 末端在基坐标系下的位姿 | 机械臂 API（已知） |
| `H_cam_end` (B/X) | 相机在末端坐标系下的位姿 | **标定目标（待求）** |
| `H_board_cam` (C) | 标定板在相机坐标系下的位姿 | 相机外参标定（可求） |
| `H_board_base` (D) | 标定板在基坐标系下的位姿 | 固定不变 |

标定板固定，机械臂移动两个位姿，构建变换回路：

```
H_end1_base * H_cam_end * H_board_cam1⁻¹ = H_end2_base * H_cam_end * H_board_cam2⁻¹

⟹ (H_end2_base⁻¹ * H_end1_base) * H_cam_end = H_cam_end * (H_board_cam2 * H_board_cam1⁻¹)

⟹ AX = XB
```

其中：
- **A** = 两次运动间机械臂末端的相对变换（从 API 获取）
- **B** = 两次运动间相机的相对运动（从相机标定求得）
- **X** = 相机→末端的固定变换（4x4 齐次矩阵 `[R|t; 0 1]`）

### Eye-to-Hand

移动机械臂到两个位姿，有：

```
H_end1_base * H_cam_base * H_board_cam1 = H_end2_base * H_cam_base * H_board_cam2

⟹ AX = XB (形式相同，但 X = H_cam_base)
```

## 4. 标定流程

### 4.1 环境要求

| 项目 | 版本 |
|------|------|
| OS | Ubuntu / Windows |
| Python | 3.9+ |
| numpy | 2.0.2 |
| opencv-python | 4.10.0.84 |
| pyrealsense2 | 2.55.1.6486 |
| scipy | 1.13.1 |

### 4.2 器材

- 机械臂: RM65 / RM75 / RM63 / GEN72
- 相机: Intel RealSense D435
- 标定板: 棋盘格（纸质打印或淘宝购买）
- 网线 + USB 数据线

### 4.3 config.yaml 参数

```yaml
XX: 11    # 横向角点数（长边格子数 - 1）
YY: 8     # 纵向角点数（短边格子数 - 1）
L: 0.03   # 单个方格尺寸（米）
```

### 4.4 Eye-in-Hand 采集步骤

1. 网线连接电脑和机械臂，USB 连接相机
2. 设置电脑 IP 与机械臂同一网段（如 192.168.1.x）
3. 标定板**固定放置**在平面上，相机**固定在机械臂末端**
4. 运行 `collect_data.py`
5. 拖动机械臂使标定板**清晰、完整**出现在相机视野中
6. 按键盘 `s` 采集一帧（保存图片 + 机械臂位姿）
7. 重复 5-6 共 **15-20 次**

**关键约束**:
- 标定板与相机镜面呈现一定角度（不要正对）
- 每次旋转角 > **30°**，确保 X/Y/Z 三轴都有足够旋转变化
- 先绕末端 Z 轴旋转多角度，再绕 X 轴旋转

### 4.5 Eye-to-Hand 采集步骤

与 Eye-in-Hand 的区别：
- 标定板**固定在机械臂末端**
- 相机**固定不动**
- 移动机械臂使标定板出现在相机视野中

### 4.6 计算标定结果

```bash
# Eye-in-Hand → 得到相机→末端的 R, t
python compute_in_hand.py

# Eye-to-Hand → 得到相机→基座的 R, t
python compute_to_hand.py
```

## 5. 代码结构

```
hand_eye_calibration/
├── collect_data.py       # 采集标定板图片 + 机械臂位姿
├── compute_in_hand.py    # Eye-in-Hand 标定计算
├── compute_to_hand.py    # Eye-to-Hand 标定计算
├── save_poses.py         # 位姿 → 齐次矩阵转换（Eye-in-Hand 用）
├── save_poses2.py        # 位姿 → 齐次矩阵转换（Eye-to-Hand 用）
├── config.yaml           # 标定板参数
├── requirements.txt
└── libs/
    ├── auxiliary.py
    └── log_settings.py
```

### 关键代码片段

**相机标定**（获取每张图的标定板外参）:
```python
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, size, None, None
)
# rvecs[i], tvecs[i] → 标定板在第 i 张图相机坐标系下的位姿
```

**机械臂位姿 → 齐次矩阵**（`save_poses.py`）:
```python
def pose_to_homogeneous_matrix(pose):
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)  # Z-Y-X 顺序
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = [x, y, z]
    return H
```

**手眼标定求解**（TSAI 算法）:
```python
# Eye-in-Hand
R_cam2end, T_cam2end = cv2.calibrateHandEye(
    R_end2base, T_end2base,      # 机械臂末端→基座（多帧）
    rvecs, tvecs,                # 标定板→相机（多帧）
    cv2.CALIB_HAND_EYE_TSAI
)

# Eye-to-Hand
R_cam2base, T_cam2base = cv2.calibrateHandEye(
    R_base2end, T_base2end,      # 基座→末端（多帧）
    rvecs, tvecs,                # 标定板→相机（多帧）
    cv2.CALIB_HAND_EYE_TSAI
)
```

## 6. 常见问题

### 问题: `Not enough informative motions`

```
[ERROR] calibrateHandEyeTsai Hand-eye calibration failed!
Not enough informative motions--include larger rotations.
```

**原因**: 采集的图片旋转量不足，缺少足够大的旋转运动。

**解决**:
1. 每次运动旋转角度 > 30°
2. 确保 X/Y/Z 三轴都有旋转变化
3. 先绕 Z 轴多角度采集，再绕 X 轴采集
4. 增加采集数量至 15-20 张

### 问题: 识别坐标全为 (0, 0, 0)

**原因**: 深度图无效或物体没有被正确识别（模型问题 / 光照问题）。

**解决**: 调整初始姿态改善识别效果，确保标定板或目标物体光照充足。

## 7. 精度

受采集图片质量影响，平移向量误差约 **1cm 以内**。
