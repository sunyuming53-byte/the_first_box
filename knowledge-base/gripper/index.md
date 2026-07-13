# Gripper Pipeline Index

Mapping of documentation to pipeline use cases for Changingtek CTAG2F90D gripper integration.

## Use Case → Document Map

| Pipeline Stage | What You Need | Primary Document | Supporting Files |
|---|---|---|---|
| **Hardware Setup** | Wiring, power, RS-485 pinout | `01-hardware-overview.md` | `assets/manuals/` (PDFs) |
| **Driver Installation** | CH341 USB-to-Serial driver | `06-software-tools.md` § Drivers | `assets/drivers/CH341_serial_driver.zip` |
| **Protocol Understanding** | Modbus RTU register map | `02-communication-protocol.md` | `assets/manuals/modbus_protocol_servo_stepper.pdf` |
| **Python Integration** | Control gripper from ROS2 node | `03-python-sdk.md` | `assets/sdk/changingtek_p_rtu_Servo.py` |
| **C++ Integration** | Embed in rm::Arm pipeline | `04-cpp-sdk.md` | `assets/sdk/changingtek_p_rtu_Servo_cpp_version.zip` |
| **ROS Visualization** | RViz / Gazebo URDF model | `05-ros-integration.md` | `assets/ros/crt_ctag2f90d_gripper_visualization.zip` |
| **Manual Testing** | Debug with Windows GUI | `06-software-tools.md` § UServoController | `assets/software/UServoController_1.0.0.8Beta.zip` |

## Critical Integration Points

### 1. Gripper ↔ Arm Subsystem
- The gripper is **independent** from the arm — separate serial port, separate protocol
- Arm uses TCP/IP via `libapi_c.so`; gripper uses RS-485 via USB serial
- Coordinate arm motion + gripper action in a single ROS2 node or state machine

### 2. Communication Stack
```
Your ROS2 Node
├── rm::Arm (TCP/IP) → RM65 Arm
└── MotorController (Modbus RTU /dev/ttyUSB0) → CTAG2F90D Gripper
```

### 3. Modbus Register Layout
```
WRITE (0x0102-0x0108): position, speed, force, accel, decel, trigger
READ  (0x0418-0x041B): position, speed, current
```

### 4. Known Values from SDK Demo
- **Position range**: 0 (open) to 9000 (closed), unit = μm (0.001mm) → 0-9mm stroke
- **Speed**: percentage (0-100)
- **Force**: percentage (0-100)
- **Accel/Decel**: 60 (default in demo)

## Dependencies

| Dependency | Python | C++ |
|---|---|---|
| Modbus library | `minimalmodbus` | Built-in (raw Modbus RTU) |
| Serial library | `pyserial` | Built-in (termios / WinAPI) |
| Thread safety | `threading` | `std::mutex` |
