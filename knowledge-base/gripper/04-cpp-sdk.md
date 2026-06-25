# 04 — C++ SDK

## Source

`assets/sdk/changingtek_p_rtu_Servo_cpp_version.zip` — cross-platform Modbus RTU implementation.

Extracted contents:

| File | Purpose |
|------|---------|
| `Changingtek_p_rtu_Servo.h` | Header — `Changingtek_p_rtu_Servo` class interface |
| `Changingtek_p_rtu_Servo.cpp` | Implementation — raw Modbus RTU frame construction, CRC16, serial I/O |
| `main.cpp` | Demo — 5-cycle open/close loop |
| `Makefile` | Linux build (g++) |
| `compile_and_run.bat` | Windows build (MSVC) |
| `README.md` | Cross-platform build instructions |

## Dependencies

**None.** The C++ SDK implements Modbus RTU directly using:
- **Linux**: `<termios.h>`, `<fcntl.h>`, `<unistd.h>` for serial I/O
- **Windows**: `<windows.h>` Win32 serial API
- CRC16 calculated in software (no external CRC library)

## Build

### Linux
```bash
cd changingtek_p_rtu_Servo_cpp_version
make
sudo chmod 666 /dev/ttyUSB0   # grant serial port access
./example_servo
```

### Windows
```cmd
compile_and_run.bat
```

## API Reference

### `Changingtek_p_rtu_Servo(port, slave_id=1, baudrate=115200, timeout=1.0)`

Constructor. Does not open port.

### `connect()` / `disconnect()`

Open/close serial port. Returns `bool` on connect.

### Motion Control

```cpp
void set_target_position(int position);
void set_target_speed(int speed);
void set_target_force(int force);
void set_target_acceleration(int acceleration);
void set_target_deceleration(int deceleration);
void trigger_motion();
void temp_move(int position_mm, int speed_pct, int force_pct,
               int accel, int decel, bool trigger = true);
```

### Feedback

```cpp
int read_real_position();   // 32-bit signed, μm
int read_real_speed();
int read_real_current();
```

### Monitoring

```cpp
void start_monitoring(double interval = 0.5);  // Background thread
void stop_monitoring();
```

## Usage Example

```cpp
#include "Changingtek_p_rtu_Servo.h"

int main() {
    Changingtek_p_rtu_Servo gripper("/dev/ttyUSB0", 1);
    if (!gripper.connect()) return 1;

    gripper.start_monitoring(0.5);

    for (int i = 0; i < 5; i++) {
        gripper.temp_move(9000, 50, 25, 60, 60);  // close
        sleep(3);
        gripper.temp_move(0, 50, 25, 60, 60);      // open
        sleep(3);
    }

    gripper.stop_monitoring();
    gripper.disconnect();
    return 0;
}
```

## Thread Safety

Internal `std::mutex` protects all serial I/O. Monitoring thread runs as `std::thread` (daemon-like on Linux, joinable on stop).

## Register Map

Identical to Python SDK. See `02-communication-protocol.md` for full register map.
