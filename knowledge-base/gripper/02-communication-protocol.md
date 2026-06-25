# 02 — Communication Protocol

## Overview

The CTAG2F90D gripper uses **Modbus RTU** over RS-485. All registers are **16-bit holding registers**.

## Serial Configuration

| Parameter | Value |
|-----------|-------|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Timeout | 1.0 s |

## Register Map

### Write Registers (Control / Target Values)

| Address | Name | Size | Description |
|---------|------|------|-------------|
| `0x0102` | `TARGET_POS_HIGH` | 16-bit | Target position (high 16 bits) |
| `0x0103` | `TARGET_POS_LOW` | 16-bit | Target position (low 16 bits) |
| `0x0104` | `TARGET_SPEED` | 16-bit | Target speed (percentage, 0–100) |
| `0x0105` | `TARGET_FORCE` | 16-bit | Target force/torque (percentage, 0–100) |
| `0x0106` | `TARGET_ACCELERATION` | 16-bit | Target acceleration |
| `0x0107` | `TARGET_DECELERATION` | 16-bit | Target deceleration |
| `0x0108` | `MOTION_TRIGGER` | 16-bit | Write `1` to trigger motion; auto-clears |

### Read Registers (Feedback / Status)

| Address | Name | Size | Description |
|---------|------|------|-------------|
| `0x0418` | `REAL_POS_HIGH` | 16-bit | Actual position (high 16 bits) |
| `0x0419` | `REAL_POS_LOW` | 16-bit | Actual position (low 16 bits) |
| `0x041A` | `REAL_SPEED` | 16-bit | Actual speed feedback |
| `0x041B` | `REAL_CURRENT` | 16-bit | Actual current feedback |

## Modbus Function Codes Used

| Function | Code | Purpose |
|----------|------|---------|
| Read Holding Registers | `0x03` | Read single or multiple registers |
| Write Single Register | `0x06` | Write one 16-bit register |
| Write Multiple Registers | `0x10` | Write consecutive registers (used for multi-register position + params) |

## Position Encoding

Position is a **32-bit signed integer** split across two 16-bit registers:

```
uint32_t combined = (reg_high << 16) | reg_low;
// Handle signed values:
if (combined & 0x80000000) combined -= 0x100000000;
```

Unit: **micrometers (μm)**. Range observed in demo: 0 (open) to 9000 (closed) → 0–9 mm stroke.

## Motion Sequence

1. Write target position (`0x0102` + `0x0103`) via `0x10` (multi-write)
2. Write target speed (`0x0104`)
3. Write target force (`0x0105`)
4. Write target accel/decel (`0x0106`, `0x0107`)
5. Write `1` to `0x0108` (motion trigger) — gripper moves to target

## Additional Protocol Details

For complete protocol documentation including error codes, multi-axis configurations, and advanced parameters, refer to:

- `assets/manuals/modbus_protocol_servo_stepper.pdf` — Full Modbus protocol specification
