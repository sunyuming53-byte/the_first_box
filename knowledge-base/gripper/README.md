# Gripper Knowledge Base — Changingtek CTAG2F90D

伺服步进夹爪（创想科技 / 知行机器人）完整资料库。

## Hardware

| Item | Detail |
|------|--------|
| **Model** | CTAG2F90D |
| **Type** | Servo Stepper Gripper (伺服步进夹爪) |
| **Vendor** | Changingtek (创想科技) / 知行机器人 |
| **Communication** | Modbus RTU over RS-485, via USB-to-Serial (CH341) |
| **Baud Rate** | 115200, 8N1 |
| **Slave ID** | 1 (default) |

## File Inventory

```
assets/
├── manuals/          # PDF documentation
│   ├── Changingtek_Actuator_Operation_Manual_EN.pdf          (1.1 MB) - English hardware manual
│   ├── actuator_operation_manual_V2.1.0 .pdf                 (5.5 MB) - 执行器操作手册 V2.1.0
│   ├── modbus_protocol_servo_stepper.pdf                     (332 KB) - Modbus 协议文档
│   └── servo_stepper_software_manual_V3.1.pdf                (2.8 MB) - 上位机软件操作手册
├── ros/              # ROS visualization package (URDF + STL meshes)
│   └── crt_ctag2f90d_gripper_visualization.zip               (1.2 MB)
├── sdk/              # Communication SDKs
│   ├── changingtek_p_rtu_Servo.py                            (11 KB) - Python (minimalmodbus)
│   └── changingtek_p_rtu_Servo_cpp_version.zip               (28 KB) - C++ (raw Modbus RTU)
├── software/         # Windows tools
│   └── UServoController_1.0.0.8Beta.zip                      (41 MB) - 上位机调试软件 (Qt6)
└── drivers/          # USB-to-Serial driver
    └── CH341_serial_driver.zip                               (806 KB) - CH341SER.EXE
```

## Knowledge Base Structure

```
knowledge-base/gripper/
├── README.md              # This file — overview & inventory
├── index.md               # Pipeline use case → document mapping
├── 01-hardware-overview.md
├── 02-communication-protocol.md
├── 03-python-sdk.md
├── 04-cpp-sdk.md
├── 05-ros-integration.md
├── 06-software-tools.md
└── assets/                # All original files (see inventory above)
```

## Quick Start

### Python Control
```python
from changingtek_p_rtu_Servo import MotorController

gripper = MotorController(port="/dev/ttyUSB0", slave_id=1)
gripper.temp_move(position_mm=0, speed_pct=50, force_pct=25, accel=60, decel=60)
```

### C++ Control
```cpp
#include "Changingtek_p_rtu_Servo.h"
Changingtek_p_rtu_Servo gripper("/dev/ttyUSB0", 1);
gripper.connect();
gripper.temp_move(0, 50, 25, 60, 60);
```

### ROS Visualization
```bash
roslaunch crt_ctag2f90d_gripper_visualization display.launch
```

### Windows Debugging
1. Install CH341 driver (`assets/drivers/串口驱动CH341.zip`)
2. Run `UServoController_1.0.0.8Beta/UServoController.exe`

## Key Pipeline Dependencies

The gripper communicates via **Modbus RTU over RS-485**. Integrating with the RealMan arm requires:
- A USB-to-RS485 converter (CH341 chipset)
- Modbus RTU library (`minimalmodbus` for Python, raw serial for C++)
- Register map knowledge (see `02-communication-protocol.md`)
