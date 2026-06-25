# 01 — Hardware Overview

## CTAG2F90D Servo Stepper Gripper

| Parameter | Value |
|-----------|-------|
| Model | CTAG2F90D |
| Type | Servo Stepper Actuator (伺服步进执行器) |
| Vendor | Changingtek (创想科技) / 知行机器人 |
| Stroke | ~9 mm (0–9000 μm) |
| Communication | RS-485 (2-wire, A/B) |
| Protocol | Modbus RTU |
| Baud Rate | 115200, 8 data bits, no parity, 1 stop bit |
| Slave ID | 1 (default, configurable) |
| Power | External 24V DC (via actuator driver) |

## Physical Interface

```
[PC / SBC] --USB--> [CH341 USB-to-RS485] --A/B wires--> [Actuator Driver] --> [Gripper Motor]
```

- **CH341 adapter**: USB-to-serial converter, provides `/dev/ttyUSB0` (Linux) or `COMx` (Windows)
- **RS-485**: Differential signaling on A (D+) and B (D-) wires
- **Driver**: Integrated in the actuator, accepts Modbus RTU commands via RS-485

## Wiring Reference

Refer to the actuator manual: `assets/manuals/执行器操作手册（伺服步进）-V2.1.0 .pdf`

## Mechanical

- 3D mesh files (STL) available for visualization: `assets/ros/extracted/*/meshes/`
- URDF model: `assets/ros/extracted/*/urdf/crt_ctag2f90d_gripper_visualization_sync.urdf`
