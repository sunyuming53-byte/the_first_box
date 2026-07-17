# Issue #12 修复报告：CameraStream 冲突解决方案

## 问题描述

`VisionClient` 和 `CalibDataCollector` 通过 `CameraStreamAdapter` / `CameraStream` 直接打开
librealsense2 SDK 访问 RealSense D435，与 `bringup.launch.py` 同时启动的 `realsense2_camera_node`
争用同一 USB 设备。librealsense2 不支持单设备多消费者，导致运行时只有一个进程能拿到帧，其余
静默失败。

## 解决方案

新增 `TopicCameraAdapter` 实现 `ICamera` 接口，通过 ROS2 topic 订阅 `/camera/color/image_raw`
和 `/camera/color/camera_info`，由 `realsense2_camera_node` 作为唯一硬件持有者。统一真机和仿真：
Gazebo 相机插件发布同名 topic，两端通用。

## 改动文件清单（11 个文件）

### 新建文件

| 文件 | 说明 |
|------|------|
| `src/omr_controller/include/omr_controller/clients/topic_camera_adapter.hpp` | TopicCameraAdapter 类声明 |
| `src/omr_controller/src/clients/topic_camera_adapter.cpp` | TopicCameraAdapter 实现 |
| `src/omr_controller/test/topic_camera_adapter_test.cpp` | 6 个单元测试 |

### 修改文件

| 文件 | 改动内容 |
|------|----------|
| `src/omr_controller/src/orchestrator.cpp` | TaskOrchestrator 中 `CameraStreamAdapter` → `TopicCameraAdapter` |
| `src/omr_controller/src/calib/collector.cpp` | CalibDataCollector::Impl 中 `CameraStream cam_` → `std::unique_ptr<ICamera> camera_` |
| `src/omr_controller/include/omr_controller/calib/collector.hpp` | CalibDataCollector 构造函数接受 `ICamera`，CalibDataConfig 移除 `CameraConfig camera` |
| `src/omr_controller/apps/calib_node.cpp` | CalibNode 注入 `TopicCameraAdapter` |
| `src/omr_controller/apps/collect.cpp` | CLI 工具注入 `CameraStreamAdapter`（保留 SDK 直连） |
| `src/omr_controller/apps/run_pipeline.cpp` | CLI 工具注入 `CameraStreamAdapter`（保留 SDK 直连） |
| `src/omr_controller/CMakeLists.txt` | 添加 `cv_bridge` 依赖 + `topic_camera_adapter_test` 测试目标 |
| `src/omr_controller/package.xml` | 添加 `cv_bridge` 依赖 |

## 改动后功能说明

### TopicCameraAdapter 类

- 继承 `ICamera` 接口，通过 ROS2 topic 订阅获取图像帧和相机内参
- 构造函数接受 `rclcpp::Node::SharedPtr` 和可选的自定义 topic 名称
- `next()` 使用 `std::condition_variable` 阻塞等待，行为与 `CameraStream::next()` 一致
- `depth_intrinsics()` 从 `/camera/color/camera_info` 解析内参矩阵 K 和畸变系数 D
- 默认订阅 topic：`/camera/color/image_raw`（best_effort QoS）、`/camera/color/camera_info`（reliable QoS）

### 数据流变化

```
改动前（3 方 SDK 竞争）：
  realsense2_camera_node ──SDK──→ D435 ←──SDK── CameraStreamAdapter (VisionClient)
                                       ←──SDK── CameraStream (CalibDataCollector)

改动后（唯一硬件持有者）：
  realsense2_camera_node ──SDK──→ D435
         │
         ├── pub /camera/color/image_raw ──→ TopicCameraAdapter (VisionClient)
         │                                    TopicCameraAdapter (CalibDataCollector)
         │                                    RViz / Foxglove
         │
         └── pub /camera/color/camera_info ──→ depth_intrinsics()
```

### 各入口方案

| 入口 | 改动前 | 改动后 | 说明 |
|------|--------|--------|------|
| TaskOrchestrator → VisionClient | CameraStreamAdapter (SDK 直连) | TopicCameraAdapter (topic 订阅) | 消除冲突 |
| CalibNode → CalibDataCollector | CameraStream (SDK 直连) | TopicCameraAdapter (topic 订阅) | 消除冲突 |
| collect CLI 工具 | CameraStream (SDK 直连) | CameraStreamAdapter (SDK 直连) | 不变，独立运行无冲突 |
| run_pipeline CLI 工具 | CameraStream (SDK 直连) | CameraStreamAdapter (SDK 直连) | 不变，独立运行无冲突 |

### 不改动的部分

- `omr_vision::CameraStream` 保持不变（非 ROS 场景使用）
- `ICamera` 接口不变
- `VisionClient` 不变
- `realsense2_camera_node` / `bringup.launch.py` 不变

## 验证结果

| 检查项 | 结果 |
|--------|------|
| clang-format | 通过 |
| 全工作空间构建 (6 packages) | 通过 |
| 测试 (23/23) | 全部通过 |
| 新增 TopicCameraAdapter 单元测试 (6) | 全部通过 |
| 原有测试回归 (17) | 零回归 |
