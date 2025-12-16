# OLAF Wiring Guide

Complete wiring guide for OLAF robot system.

## Overview

OLAF uses a distributed architecture with:
- **1× Raspberry Pi 5** (orchestrator, I2C master)
- **4× ESP32 modules** (smart peripherals, I2C slaves)
- **Shared I2C bus** (single bus connecting all modules)
- **Distributed power** (buck converters per module from 12V battery)

## Before You Start

### Required Tools
- Soldering iron and solder
- Wire strippers
- Multimeter
- Heat shrink tubing
- Cable ties
- Label maker
- Helping hands / PCB holder

### Required Materials
- 22-26 AWG stranded wire (multiple colors)
- JST-XH connectors (4-pin for I2C)
- XT60 connectors (for battery)
- 2.2kΩ resistors (I2C pull-ups)
- Buck converters (5V output)
- 12V LiPo battery (4S, 5000+ mAh)

### Safety First
⚠️ **Warning:**
- Disconnect battery before any wiring work
- Verify polarity before connecting power
- Use fuses on battery output (15A recommended)
- Never short battery terminals
- Keep battery away from conductive objects

## System Architecture

```
                    12V Battery (in Torso)
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   Buck Conv #1       Buck Conv #2       Buck Conv #3
     (5V, 5A)          (5V, 3A)          (5V, 2A)
        │                  │                  │
   Raspberry Pi       Head+Ears           Neck+Torso
   + Hailo AI          Module             Modules
                                               │
                                          Base Module
                                          (12V direct
                                           for motors)

              I2C Bus (SDA, SCL, GND)
    Pi ───┬───┬───┬───┬
          │   │   │   │
         0x08│0x09│0x0A│0x0B
       Head  │Neck│Torso│Base
```

## Step 1: Power Distribution System

### Battery Placement
- Mount 12V battery in torso module
- Use velcro straps for easy removal
- Install XT60 connector on battery output
- Add 15A fuse inline after battery

### Buck Converter Installation

**Buck #1: Raspberry Pi (5V, 5A)**
- Input: 12V from battery
- Output: 5V, 5A
- Connection: Raspberry Pi GPIO header (5V, GND) or USB-C

**Buck #2: Head+Ears (5V, 3A)**
- Input: 12V from battery
- Output: 5V, 3A
- Powers: ESP32, OLEDs, servos, projector control

**Buck #3: Neck + Torso (5V, 2A each)**
- Input: 12V from battery
- Output: 5V, 2A
- Powers: ESP32s, servos, displays, sensors

**Base Module: Direct 12V**
- ODrive motor controller needs 12V
- ESP32 on Base has onboard 5V regulator

### Wiring

1. **Battery to Distribution Board:**
   ```
   Battery (+) ──[ 15A Fuse ]──┬── Buck #1 Input (+)
                               ├── Buck #2 Input (+)
                               ├── Buck #3 Input (+)
                               └── Base ODrive (+)

   Battery (-) ────────────────┴── All Grounds (common)
   ```

2. **Test Each Buck Converter:**
   - Set output to 5.0V using adjustment pot
   - Measure with multimeter (should be 4.95-5.05V)
   - Label each converter (Pi, Head, Neck, etc.)

## Step 2: I2C Bus Wiring

### I2C Basics
- **SDA (Data):** GPIO 2 on Raspberry Pi
- **SCL (Clock):** GPIO 3 on Raspberry Pi
- **Pull-ups:** 2.2kΩ resistors on both SDA and SCL (install once, typically at Pi or first module)
- **Bus topology:** All modules connect in parallel to same SDA/SCL lines

### Physical Wiring

**Color Convention:**
- Blue: SDA (data)
- Yellow: SCL (clock)
- Black: GND
- Red: 3.3V (logic level, optional)

**Raspberry Pi to Modules:**

```
Raspberry Pi GPIO Header:
  Pin 3 (GPIO 2) [SDA] ─┬── 2.2kΩ to 3.3V
  Pin 5 (GPIO 3) [SCL] ─┼── 2.2kΩ to 3.3V
  Pin 6 (GND)    [GND] ─┤
  Pin 1 (3.3V)   [3.3V]─┤
                        │
                4-wire cable (JST-XH recommended)
                        │
          ┌─────────────┴──────────┬──────────┬──────────┐
          │                        │          │          │
    Head+Ears (0x08)          Neck (0x09)  Torso (0x0A)  Base (0x0B)
    ESP32 GPIO21/22           ESP32 GPIO21/22
```

**Important:**
- All modules share **common ground**
- Only install pull-up resistors **once** (typically on Pi or first module)
- Keep cable length < 1 meter for 400kHz I2C
- Use twisted pair for SDA/SCL if possible (reduces noise)

### I2C Pull-Up Resistors

Install on Raspberry Pi or first module:

```
3.3V ──[ 2.2kΩ ]─┬── SDA (GPIO 2)
                 │
3.3V ──[ 2.2kΩ ]─┴── SCL (GPIO 3)
```

## Step 3: Module-Specific Wiring

### Head+Ears Module (0x08)

**Connections:**
1. **I2C from Pi:** SDA (GPIO21), SCL (GPIO22), GND
2. **Power:** 5V (from Buck #2), GND
3. **HDMI:** Video from Pi to projector
4. **Internal peripherals:**
   - 2× OLED eyes (SPI)
   - 2× Ear servos (UART)
   - Projector power (GPIO → optocoupler)
   - Projector focus (PWM servo)

See: `hardware/modules/head-ears/assembly/README.md` for detailed wiring

### Neck Module (0x09)

**Connections:**
1. **I2C from Pi:** SDA (GPIO21), SCL (GPIO22), GND
2. **Power:** 5V (from Buck #3), GND
3. **Internal peripherals:**
   - 3× Neck servos (UART)
   - 2× Presence sensors (I2C, separate from main bus)

See: `hardware/modules/neck/assembly/README.md`

### Torso Module (0x0A)

**Connections:**
1. **I2C from Pi:** SDA (GPIO21), SCL (GPIO22), GND
2. **Power:** 5V (from Buck #3), GND
3. **Internal peripherals:**
   - Heart display (SPI)
   - Thermal printer (UART)
   - LED indicators (GPIO)
   - Battery monitoring (ADC)

See: `hardware/modules/torso/assembly/README.md`

### Base Module (0x0B)

**Connections:**
1. **I2C from Pi:** SDA (GPIO21), SCL (GPIO22), GND
2. **Power:**
   - 12V direct from battery (for ODrive motors)
   - 5V from onboard regulator (for ESP32)
3. **Internal peripherals:**
   - ODrive motor controller (UART, 12V power)
   - MPU6050 IMU (I2C, separate from main bus)
   - Kickstand servo (UART)

See: `hardware/modules/base/assembly/README.md`

## Step 4: Raspberry Pi Connections

### GPIO Connections
- **GPIO 2 (Pin 3):** I2C SDA
- **GPIO 3 (Pin 5):** I2C SCL
- **Pin 6:** GND (common ground with all modules)
- **Pin 1:** 3.3V (for I2C pull-ups)

### USB Connections
- **USB 3.0 Port 1:** RGBD Camera (Intel RealSense or similar)
- **USB 3.0 Port 2:** Speakerphone
- **USB 2.0 Port 1:** Reserved

### HDMI Connection
- **HDMI Out:** To Head+Ears projector (video signal only)

### Power
- **5V via GPIO header** or **USB-C** (from Buck #1, 5V 5A)

### PCIe
- **Hailo AI Kit:** Via PCIe HAT

## Step 5: Cable Routing

### Best Practices
1. **Separate power and signal cables** (reduce EMI)
2. **Use cable ties** every 10cm along runs
3. **Strain relief** at all connection points
4. **Label both ends** of every cable
5. **Color coding:**
   - Red: Power (+)
   - Black: Ground (-)
   - Blue: I2C SDA
   - Yellow: I2C SCL
   - Other colors: Module-specific signals

### Physical Routing
- **I2C backbone:** Through torso center column
- **Power distribution:** Star topology from battery (torso base)
- **Motor cables:** Direct, thick gauge (16-18 AWG) from battery to base
- **HDMI:** From Pi (torso) up through neck to head

### Cable Management
- Use spiral wrap for cable bundles
- Reserve extra length at joints (neck, ears) for movement
- Secure moving cables with flexible mounts

## Step 6: Testing

### Pre-Power Checks
- [ ] Multimeter continuity test: All grounds connected
- [ ] Multimeter continuity test: No shorts between V+ and GND
- [ ] Visual inspection: All polarities correct
- [ ] Verify I2C pull-up resistors installed (2.2kΩ)

### Power-On Sequence
1. **Connect battery** (all modules should power on)
2. **Check buck converter outputs** (should all be 5.0V ±0.25V)
3. **Verify ESP32 power LEDs** (all 4 modules)
4. **Boot Raspberry Pi** (should boot normally)

### I2C Communication Test

On Raspberry Pi:

```bash
# Enable I2C
sudo raspi-config
# Interface Options → I2C → Enable

# Install tools
sudo apt-get install i2c-tools

# Scan I2C bus
i2cdetect -y 1
```

**Expected output:**
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- 08 09 0a 0b -- -- -- --
...
```

You should see:
- **0x08:** Head+Ears
- **0x09:** Neck
- **0x0A:** Torso
- **0x0B:** Base

### Module-Specific Tests

Run tests for each module:

```bash
# Head+Ears: Test eyes and servos
python3 tools/diagnostics/module_health_check.py --module head-ears

# Neck: Test servo movement
python3 tools/diagnostics/module_health_check.py --module neck

# Torso: Test display and printer
python3 tools/diagnostics/module_health_check.py --module torso

# Base: Test motors (with robot on blocks!)
python3 tools/diagnostics/odrive_diagnostic.py
```

## Troubleshooting

### No I2C Communication

**Symptom:** `i2cdetect` shows empty (no devices)

**Checks:**
1. Verify I2C enabled in `raspi-config`
2. Check pull-up resistors (2.2kΩ on SDA, SCL)
3. Measure voltage on SDA/SCL (should be 3.3V when idle)
4. Check common ground between Pi and modules
5. Try lower bus speed: Add to `/boot/config.txt`:
   ```
   dtparam=i2c_arm=on,i2c_arm_baudrate=100000
   ```

### Specific Module Not Responding

**Symptom:** `i2cdetect` shows 0x08, 0x09, 0x0A but missing 0x0B

**Checks:**
1. Check power to missing module (5V at ESP32)
2. Check ground connection
3. Verify firmware I2C address matches (e.g., `#define I2C_SLAVE_ADDRESS 0x0B`)
4. Flash latest firmware to module
5. Check for shorts on I2C pins

### Intermittent I2C Errors

**Symptom:** I2C communication works but occasional errors

**Solutions:**
1. Reduce cable length (< 1 meter)
2. Use twisted pair for SDA/SCL
3. Add capacitors (100μF) at module power inputs
4. Reduce I2C bus speed to 100kHz
5. Check for loose connections
6. Keep I2C cables away from motor power cables

### Power Issues

**Symptom:** Modules resetting, brownouts

**Solutions:**
1. Check buck converter output voltage (should be stable 5.0V)
2. Measure current draw (should be within converter limits)
3. Add bulk capacitors (1000μF) at power entry points
4. Upgrade to higher current buck converters if needed
5. Check battery voltage (should be > 11V under load)

## Wiring Documentation

Detailed wiring diagrams available at:
- **System-level:** `hardware/wiring/`
- **Per-module:** `hardware/modules/{module}/assembly/`

## Next Steps

After successful wiring and testing:

1. **Calibration:** Run calibration scripts for servos and sensors
2. **Firmware:** Flash latest firmware to all modules
3. **ROS2:** Launch ROS2 bringup to start all nodes
4. **Integration test:** Test full system (AI, navigation, expressions)

## References

- [I2C Specification (NXP)](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)
- [Wire Gauge Chart](https://www.powerstream.com/Wire_Size.htm)
- Module-specific assembly guides in `hardware/modules/{module}/assembly/`
