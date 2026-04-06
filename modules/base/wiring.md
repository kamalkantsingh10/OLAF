# Base Module Wiring Guide

## Overview

Complete wiring reference for the OLAF base platform. Covers power distribution from the 36V hoverboard battery through all voltage rails, ODrive motor controller connections, ESP32 (ESP32-S3-DevKitC-1) sensor and communication wiring.

---

## Power Distribution

### Power Rails

| Rail | Source | Converter | Loads |
|------|--------|-----------|-------|
| 36V | Battery direct | None (fuse + switch) | ODrive v3.6 + 2× BLDC motors |
| 12V | 36V battery | 36V→12V buck | Servo power via Waveshare adapters (neck, ears) |
| 5V | 36V battery | 36V→5V buck | ESP32 (Head, Base), OLED eyes |

### Battery → Main Switch → Fuse

```
36V Battery (+) ──→ Emergency Stop Switch ──→ Fuse ──→ Power Bus (+)
36V Battery (-) ──→ Power Bus (-)
```

- **Emergency stop switch**: Cuts ALL power when pressed
- **Fuse**: Rated for motor stall current (check ODrive + motor specs)

### 36V Rail (ODrive)

```
Power Bus (+) ──→ ODrive DC+ terminal
Power Bus (-) ──→ ODrive DC- terminal
```

- Direct 36V — no conversion needed
- ODrive powers both BLDC motors from its motor output terminals

### 36V → 12V Buck Converter

```
Power Bus (+) ──→ 12V Buck IN+
Power Bus (-) ──→ 12V Buck IN-
12V Buck OUT+ ──→ Waveshare Adapter power (neck + ears)
12V Buck OUT- ──→ Waveshare Adapter GND
```

- Verify output is 12V with multimeter before connecting loads
- Powers Feetech STS3215 (neck) and SCS0009 (ear) servos via Waveshare Bus Servo Adapters

### 36V → 5V Buck Converter

```
Power Bus (+) ──→ 5V Buck IN+
Power Bus (-) ──→ 5V Buck IN-
5V Buck OUT+ ──→ ESP32 5V / OLED 5V
5V Buck OUT- ──→ ESP32 GND / OLED GND
```

- Verify output is 5V with multimeter before connecting loads
- Powers Base ESP32, Head ESP32, and OLED eye displays

---

## ODrive v3.6 Connections

### Motor Phase Wires

| ODrive Terminal | Connection | Notes |
|-----------------|------------|-------|
| M0 (A, B, C) | Left hoverboard motor phase wires | Match phase order — swap any 2 to reverse direction |
| M1 (A, B, C) | Right hoverboard motor phase wires | Match phase order — swap any 2 to reverse direction |

### Hall Sensor Cables

Each hoverboard motor has a 5-wire Hall sensor cable:

| Wire | ODrive Pin | Function |
|------|-----------|----------|
| Red | 5V | Hall sensor power |
| Black | GND | Hall sensor ground |
| Yellow | Hall A | Encoder channel A |
| Green | Hall B | Encoder channel B |
| Blue | Hall C | Encoder channel C |

> **Note:** Wire colors may vary by motor manufacturer. Verify with multimeter or motor datasheet. Connect to ODrive M0/M1 encoder ports respectively.

### UART to ESP32

| ODrive Pin | ESP32 GPIO | Function |
|------------|-----------|----------|
| TX (GPIO 1) | GPIO 38 (RX) | ODrive transmit → ESP32 receive |
| RX (GPIO 2) | GPIO 39 (TX) | ESP32 transmit → ODrive receive |
| GND | GND | Common ground |

- Uses ODrive's UART interface for velocity commands and odometry feedback
- Baud rate: 115200 (ODrive default)
- Pins on J3 (right header) for easier screw-terminal wiring

---

## ESP32 (ESP32-S3-DevKitC-1) Pin Assignments

### Summary Table

| Function | GPIO | Bus | Direction | Notes |
|----------|------|-----|-----------|-------|
| I2C0 SDA (IMU + OLED) | 4 | I2C0 (Master) | Bidirectional | Shared: BNO085 (0x4A) + SSD1306 (0x3C) |
| I2C0 SCL (IMU + OLED) | 5 | I2C0 (Master) | Clock | Shared: BNO085 (0x4A) + SSD1306 (0x3C) |
| BNO085 INT | 6 | GPIO | Input (active low) | Data-ready interrupt — avoids polling at 200Hz |
| BNO085 RST | 7 | GPIO | Output (active low) | Hardware reset — recovery from hangs |
| Pi SDA | 21 | I2C1 (Slave) | Bidirectional | J3 right header, near USB pins |
| Pi SCL | 47 | I2C1 (Slave) | Clock | J3 right header, near USB pins |
| ODrive TX | 39 | UART | TX | ESP32 → ODrive (J3 right header) |
| ODrive RX | 38 | UART | RX | ODrive → ESP32 (J3 right header) |

### I2C0 Master Bus — IMU + OLED (GPIO 4/5, Shared)

Both devices share one I2C bus on GPIO 4/5. Different addresses, no conflict.

**BNO085 (GY-BNO085 breakout):**

| ESP32 GPIO | BNO085 Pin | Notes |
|------------|------------|-------|
| GPIO 4 | SDA | Shared I2C0 bus |
| GPIO 5 | SCL | Shared I2C0 bus |
| GPIO 6 | INT | Data-ready interrupt (active low) — connect for 200Hz operation |
| GPIO 7 | RST | Hardware reset (active low) — pull high via 10kΩ, ESP32 drives low to reset |
| 3.3V | VIN | Power from ESP32 3.3V regulator |
| GND | GND | Common ground |
| 3.3V | CS | Tie HIGH — deselects SPI, required for I2C mode |
| GND | PS0 | Tie to GND — selects I2C protocol (PS0=LOW + PS1=LOW = I2C) |
| GND | PS1 | Tie to GND — selects I2C protocol |
| GND | AD0 | Tie to GND — sets I2C address to 0x4A |

**SSD1306 OLED (0.91"):**

| ESP32 GPIO | OLED Pin | Notes |
|------------|----------|-------|
| GPIO 4 | SDA | Shared I2C0 bus |
| GPIO 5 | SCL | Shared I2C0 bus |
| 3.3V | VCC | Power from ESP32 3.3V regulator |
| GND | GND | Common ground |

**Bus Notes:**
- **IMU:** BNO085 (GY-BNO085 breakout) — 9-axis AHRS with on-chip Hillcrest SH-2 sensor fusion
- **IMU address:** 0x4A (default, AD0 low). Alternate: 0x4B (AD0 high)
- **OLED address:** 0x3C (default) — verify with I2C scan, some boards use 0x3D
- **I2C speed:** 400kHz (fast mode)
- **Pull-ups:** On-board (GY-BNO085 breakout has built-in pull-ups) — do NOT add external pull-ups
- **INT pin:** BNO085 asserts INT low when new sensor data is ready. Use GPIO interrupt on ESP32 to read at exactly 200Hz without polling. Configure as `INPUT_PULLUP` on ESP32 side.
- **RST pin:** Drive low for >10µs to hardware-reset BNO085. Useful for recovery if sensor hangs. Default: pulled high (running).
- **BNO085 outputs:** Rotation vector (quaternion), euler angles, linear acceleration, gyro — no complementary filter needed on ESP32
- **Library:** Adafruit_BNO08x (Arduino)
- **OLED refresh:** Update at 10Hz max — do NOT call inside 200Hz PID loop
- **OLED mounting:** Back of base platform, facing rearward for PID tuning visibility
- **Init:** `Wire.begin(4, 5);`

### Pi Communication — I2C1 Slave Bus

| ESP32 GPIO | Pi Pin | Notes |
|------------|--------|-------|
| GPIO 21 | SDA (Pi I2C1) | J3-18, right header near USB pins |
| GPIO 47 | SCL (Pi I2C1) | J3-17, right header near USB pins |
| GND | GND | Common ground |

- **Slave address:** 0x0B (Base module)
- **Pins on J3 (right header)** for easier screw-terminal wiring alongside ODrive UART
- **Level shifting:** ESP32-S3 is 3.3V, Pi is 3.3V — no level shifter needed

### ESP32 Power

| Source | ESP32 Pin | Notes |
|--------|-----------|-------|
| 5V Buck OUT+ | 5V / VIN | Check board pinout for correct 5V input pin |
| 5V Buck OUT- | GND | Common ground |

---

## ODrive v3.6 Software Setup

### Prerequisites

- ODrive connected via USB to PC
- `odrive` Python package installed (in project poetry env)
- 36V battery connected and switched on
- **Wheels off ground** for calibration (motors will spin)

### 1. Configure Motor & Encoder Parameters

Run via `poetry run python3` or `odrivetool`:

```python
import odrive
from odrive.enums import *

odrv0 = odrive.find_any(timeout=10)

for i in [0, 1]:
    ax = getattr(odrv0, f'axis{i}')

    # Motor — hoverboard BLDC, 15 pole pairs
    ax.motor.config.pole_pairs = 15
    ax.motor.config.motor_type = MOTOR_TYPE_PMSM_CURRENT_CONTROL
    ax.motor.config.current_lim = 25                    # Amps
    ax.motor.config.calibration_current = 10             # Amps
    ax.motor.config.resistance_calib_max_voltage = 4     # Volts (low — hoverboard motors have low resistance)
    ax.motor.config.current_control_bandwidth = 100      # Hz
    ax.motor.config.torque_constant = 8.27 / 16          # KV=16, adjust if your motors differ

    # Encoder — built-in Hall sensors
    ax.encoder.config.mode = ENCODER_MODE_HALL
    ax.encoder.config.cpr = 90                           # pole_pairs × 6
    ax.encoder.config.calib_scan_distance = 150          # pulses
    ax.encoder.config.bandwidth = 100                    # Hz

    # Controller — velocity control for balancing
    ax.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    ax.controller.config.vel_limit = 10                  # turns/sec
    ax.controller.config.pos_gain = 1.0
    tc = ax.motor.config.torque_constant
    cpr = ax.encoder.config.cpr
    ax.controller.config.vel_gain = 0.02 * tc * cpr              # ~0.93
    ax.controller.config.vel_integrator_gain = 0.1 * tc * cpr    # ~4.65

odrv0.save_configuration()
odrv0.reboot()
```

### 2. Calibrate Both Motors

After reboot, reconnect and run calibration:

```python
odrv0 = odrive.find_any(timeout=10)

# Clear any errors
for i in [0, 1]:
    ax = getattr(odrv0, f'axis{i}')
    ax.error = 0
    ax.motor.error = 0
    ax.encoder.error = 0

# Calibrate each axis (motors will beep and spin)
for i in [0, 1]:
    ax = getattr(odrv0, f'axis{i}')
    ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.5)
    assert ax.motor.error == 0, f"Axis {i} motor error: {ax.motor.error}"
    assert ax.encoder.error == 0, f"Axis {i} encoder error: {ax.encoder.error}"
    print(f"Axis {i}: resistance={ax.motor.config.phase_resistance:.4f}Ω, inductance={ax.motor.config.phase_inductance:.6f}H")

# Mark pre-calibrated (skip calibration on future power-ups)
for i in [0, 1]:
    ax = getattr(odrv0, f'axis{i}')
    ax.motor.config.pre_calibrated = True
    ax.encoder.config.pre_calibrated = True

odrv0.save_configuration()
```

### 3. Verify Velocity Control

```python
odrv0 = odrive.find_any(timeout=10)

# Test axis 0
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 2    # 2 turns/sec forward
time.sleep(2)
print(f"Measured: {odrv0.axis0.encoder.vel_estimate:.2f} rev/s")  # Should be ~2.0
odrv0.axis0.controller.input_vel = 0
odrv0.axis0.requested_state = AXIS_STATE_IDLE
```

### UART Configuration

UART is enabled by default on ODrive v3.6 firmware 0.5.x:

| Parameter | Value |
|-----------|-------|
| `enable_uart` | `True` (default) |
| `uart_baudrate` | `115200` (default) |
| TX pin | ODrive GPIO 1 (J3 right header) |
| RX pin | ODrive GPIO 2 (J3 right header) |

No additional UART configuration needed — GPIO 1/2 are hardwired as UART on v3.6.

**ESP32 UART protocol (ASCII):**

```
v 0 2.5\n          # Set axis0 velocity to 2.5 turns/sec
v 1 -2.5\n         # Set axis1 velocity to -2.5 turns/sec
f 0\n               # Request axis0 feedback → returns "pos vel\n"
w axis0.requested_state 8\n   # Enter closed-loop control (state 8)
```

### Reference Values (from calibration 2026-03-30)

| Parameter | Axis 0 (Left) | Axis 1 (Right) |
|-----------|---------------|----------------|
| Phase resistance | 0.2574 Ω | 0.2934 Ω |
| Phase inductance | 0.560 mH | 0.613 mH |
| Velocity tracking | ±0.01 rev/s error at 2 rev/s | ±0.01 rev/s error at 2 rev/s |
| Firmware | 0.5.1 | — |
| Battery voltage | 38.0V | — |

### Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| `MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE` | `resistance_calib_max_voltage` too low | Increase from 4V to 6V, retry calibration |
| Motor doesn't spin during calibration | Phase wires disconnected or wrong order | Check A/B/C connections, swap any 2 to reverse |
| `ENCODER_ERROR_ILLEGAL_HALL_STATE` | Hall sensor wire loose or wrong order | Check 5-pin cable, verify with `encoder.hall_state` (should be 1-6) |
| Velocity tracking poor | Gains too low for load | Increase `vel_gain` by 50%, recalibrate |
| ODrive not found via USB | USB cable data-only or driver issue | Try different cable, check `lsusb` for ODrive |

---

## Safety Notes

1. **Always verify voltages with multimeter before connecting loads**
2. **Emergency stop switch** must cut battery power to all rails
3. **Fuse** protects against shorts — size for ODrive + motor stall current
4. **Never hot-swap motor phase wires** while ODrive is powered
5. **Low-voltage cutoff** at ~30V prevents battery over-discharge damage
6. **Common ground** — all GND connections must share a common ground reference
7. **Strain relief** — secure all wires with cable ties, especially motor phase wires which experience vibration

---

## Wiring Checklist

Use this checklist when assembling:

- [ ] Battery → E-stop switch → fuse → power bus
- [ ] Power bus → ODrive DC+/DC-
- [ ] Power bus → 12V buck converter input
- [ ] Power bus → 5V buck converter input
- [ ] 12V buck output verified at 12V
- [ ] 5V buck output verified at 5V
- [ ] ODrive M0 ← left motor phase wires (A, B, C)
- [ ] ODrive M1 ← right motor phase wires (A, B, C)
- [ ] ODrive M0 encoder ← left motor Hall sensors
- [ ] ODrive M1 encoder ← right motor Hall sensors
- [ ] ODrive UART TX → ESP32 GPIO 38 (RX)
- [ ] ODrive UART RX ← ESP32 GPIO 39 (TX)
- [ ] ODrive GND — ESP32 GND
- [ ] BNO085 SDA — ESP32 GPIO 4 (shared I2C0 bus)
- [ ] BNO085 SCL — ESP32 GPIO 5 (shared I2C0 bus)
- [ ] BNO085 INT — ESP32 GPIO 6 (data-ready interrupt)
- [ ] BNO085 RST — ESP32 GPIO 7 (hardware reset, pulled high)
- [ ] BNO085 VIN — 3.3V
- [ ] BNO085 GND — GND
- [ ] BNO085 CS — 3.3V (deselect SPI for I2C mode)
- [ ] BNO085 PS0 — GND (I2C mode)
- [ ] BNO085 PS1 — GND (I2C mode)
- [ ] BNO085 AD0 — GND (address 0x4A)
- [ ] OLED SDA — ESP32 GPIO 4 (shared I2C0 bus)
- [ ] OLED SCL — ESP32 GPIO 5 (shared I2C0 bus)
- [ ] OLED VCC — 3.3V
- [ ] OLED GND — common GND
- [ ] I2C scan confirms IMU (0x4A) + OLED (0x3C) on bus
- [ ] ESP32 5V/VIN — 5V buck output
- [ ] ESP32 GND — common GND
- [ ] All GNDs tied together (common ground)
- [ ] No wires near wheel rotation path
