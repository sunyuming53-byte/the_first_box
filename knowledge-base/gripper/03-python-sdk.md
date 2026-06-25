# 03 — Python SDK

## Source

`assets/sdk/changingtek_p_rtu_Servo.py` — 284 lines, `MotorController` class.

## Dependencies

```bash
pip install minimalmodbus pyserial
```

## API Reference

### `MotorController(port, slave_id=1, baudrate=115200, timeout=1.0)`

Constructor. Opens serial port and initializes Modbus RTU instrument.

| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `port` | `str` | required | Serial port (`/dev/ttyUSB0` on Linux, `COM3` on Windows) |
| `slave_id` | `int` | `1` | Modbus slave address |
| `baudrate` | `int` | `115200` | Serial baud rate |
| `timeout` | `float` | `1.0` | Read/write timeout in seconds |

### Motion Control

```python
set_target_position(position: int)   # Write 32-bit position (high + low registers)
set_target_speed(speed: int)         # Speed 0-100 (%)
set_target_force(force: int)         # Force 0-100 (%)
set_target_acceleration(accel: int)
set_target_deceleration(decel: int)
trigger_motion()                     # Write 1 to 0x0108 register
```

### Convenience Method

```python
temp_move(position_mm: int, speed_pct: int, force_pct: int,
          accel: int, decel: int, trigger: bool = True)
```

Sets all parameters and optionally triggers motion in one call.

### Feedback

```python
read_real_position() → int   # 32-bit signed position in μm
read_real_speed() → int      # Current speed
read_real_current() → int    # Current draw
```

### Monitoring

```python
start_monitoring(interval: float = 0.01)   # Background thread, prints [pos, speed, current]
stop_monitoring()                           # Stop background thread
```

## Usage Example

```python
from changingtek_p_rtu_Servo import MotorController

gripper = MotorController(port="/dev/ttyUSB0", slave_id=1)

# Close gripper
gripper.temp_move(position_mm=9000, speed_pct=50, force_pct=25, accel=60, decel=60)

# Read position
pos = gripper.read_real_position()  # e.g., 8950 (μm)

# Open gripper
gripper.temp_move(position_mm=0, speed_pct=50, force_pct=25, accel=60, decel=60)
```

## Thread Safety

All register read/write operations are protected by `threading.Lock`, so the monitoring thread and control thread can run concurrently without data races.

## Known Unit

Position is in **micrometers (μm)** based on the demo values:
- `0` = fully open (0 mm)
- `9000` = fully closed (9 mm stroke)
