# 06 — Software Tools

## UServoController — Windows Debugging GUI

`assets/software/UServoController_1.0.0.8Beta.zip` (41 MB)

Qt6-based Windows application for manual gripper control and debugging.

### Features
- Serial port configuration (COM port, baud rate, slave ID)
- Manual position/speed/force control sliders
- Real-time position/speed/current monitoring
- Parameter database (SQLite: `database/database.db`)

### Usage
1. Install CH341 driver (see below)
2. Extract the ZIP
3. Run `UServoController.exe`
4. Select COM port and connect
5. Use sliders to control position, speed, force

### Dependencies (bundled)
- Qt6 (Qt6Charts, Qt6Core, Qt6Gui, Qt6Widgets)
- OpenGL software renderer (`opengl32sw.dll`)

## CH341 USB-to-Serial Driver

`assets/drivers/串口驱动CH341.zip` (806 KB)

Windows driver for the CH341 USB-to-Serial chip used in the RS-485 adapter.

### Installation
1. Extract the ZIP
2. Run `CH341SER.EXE` as Administrator
3. Plug in the USB-to-RS485 adapter
4. Verify in Device Manager → Ports (COM & LPT) → USB-SERIAL CH341 (COMx)

### Linux
CH341 is supported by the mainline kernel (`ch341` driver). No installation needed:
```bash
dmesg | grep ch341        # verify driver loaded
ls /dev/ttyUSB*           # find device node
sudo chmod 666 /dev/ttyUSB0  # grant access
```
