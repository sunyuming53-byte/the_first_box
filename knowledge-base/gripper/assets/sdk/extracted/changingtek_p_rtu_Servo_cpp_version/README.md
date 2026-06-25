# Changingtek 伺服步进控制器 C++ SDK

基于 Modbus RTU 协议的伺服步进电机控制 SDK (C++ 版本)。
本 SDK 采用跨平台设计，一套代码可同时支持 **Windows** 和 **Linux (x86/ARM)** 系统。

## 目录结构

- `Changingtek_p_rtu_Servo.h`: SDK 头文件 (核心接口)
- `Changingtek_p_rtu_Servo.cpp`: SDK 实现文件 (Modbus RTU 通信逻辑)
- `main.cpp`: 主程序 (包含 5 次循环控制演示)
- `compile_and_run.bat`: Windows 编译运行脚本 (MSVC)
- `Makefile`: Linux 编译脚本

## 编译与运行指南

### 1. Windows 环境

**前提条件**: 安装 Visual Studio (支持 C++ 开发)。

**操作步骤**:
1. 确保 USB 转串口模块已插入，并确认端口号 (例如 `COM3`)。
2. 如需修改端口，编辑 `main.cpp` 中的 `port` 变量。
3. 双击运行 `compile_and_run.bat`，或在命令行中执行：
   ```cmd
   compile_and_run.bat
   ```

---

### 2. Linux 环境 (Ubuntu / ARM 架构)

**前提条件**: 系统已安装 `g++` 和 `make` 工具。

**操作步骤**:

1. **编译**:
   打开终端进入项目目录，执行 `make` 命令：
   ```bash
   cd /path/to/changingtek_p_rtu_Servo_cpp_version
   make
   ```
   编译成功后会生成可执行文件 `example_servo`。

2. **配置权限与端口**:
   *   确认设备端口：通常为 `/dev/ttyUSB0` (USB转串口) 或 `/dev/ttyTHS1` (嵌入式板载串口)。
   *   如果端口不是默认的 `/dev/ttyUSB0`，请修改 `main.cpp` 第 20 行：
       ```cpp
       port = "/dev/ttyTHS1"; // 示例：修改为实际端口
       ```
       修改后需要重新运行 `make`。
   *   授予串口读写权限 (如遇到 Permission denied 错误)：
       ```bash
       sudo chmod 666 /dev/ttyUSB0
       ```

3. **运行**:
   ```bash
   ./example_servo
   ```

4. **清理**:
   删除编译生成的文件：
   ```bash
   make clean
   ```

## 功能特性

- **跨平台兼容**: 自动识别操作系统 (`_WIN32` 宏)，底层分别调用 Windows API 和 Linux 系统调用 (termios)，无需修改代码即可移植。
- **通信优化**: 针对伺服控制优化，使用 Modbus 0x10 功能码实现多寄存器同步写入 (位置/速度/力矩/加减速)，显著降低通信延迟。
- **线程安全**: 内部集成互斥锁 (`std::mutex`)，支持在主线程控制运动的同时，后台线程实时读取电机状态。
