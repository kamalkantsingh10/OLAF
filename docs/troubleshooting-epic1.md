# Epic 1 Troubleshooting Guide

## Overview

This guide covers common issues encountered during Epic 1 (Foundation) development and testing, including hardware setup, firmware flashing, I2C communication, and ROS2 node operation.

**Epic 1 Components:**
- Raspberry Pi 5 (8GB RAM) running ROS2 Humble
- ESP32-S3-DevKitC-1 (Head Module)
- 2× GC9A01 Round TFT Displays (Eyes)
- I2C communication (Pi ↔ ESP32)
- SPI communication (ESP32 ↔ Displays)

---

## Table of Contents

1. [Hardware Issues](#hardware-issues)
2. [ESP32 Firmware Issues](#esp32-firmware-issues)
3. [I2C Communication Issues](#i2c-communication-issues)
4. [Display Issues](#display-issues)
5. [ROS2 Node Issues](#ros2-node-issues)
6. [Performance Issues](#performance-issues)
7. [General Debugging Tips](#general-debugging-tips)

---

## Hardware Issues

### ESP32 Not Detected by Computer

**Symptom:** ESP32 doesn't show up when running `pio device list` or in `/dev/ttyUSB*` or `/dev/ttyACM*`

**Possible Causes:**
1. USB cable is power-only (no data lines)
2. USB driver not installed
3. Faulty USB port or cable
4. ESP32 hardware failure

**Solutions:**

1. **Try a different USB cable** (many cheap cables are power-only):
   ```bash
   # Check if device appears
   lsusb | grep -i "CP210\|CH340\|FTDI"
   ```

2. **Install USB-to-UART drivers** (usually automatic on Linux):
   ```bash
   # For CP210x (ESP32-S3 common driver)
   sudo apt install linux-modules-extra-$(uname -r)
   ```

3. **Check USB permissions**:
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Log out and log back in, or:
   newgrp dialout
   ```

4. **Test with different USB port** or different computer

---

### Common Ground Not Connected

**Symptom:** I2C communication fails, ESP32 appears unstable, random resets

**Cause:** Missing ground connection between Raspberry Pi and ESP32

**Solution:**

**⚠️ CRITICAL:** Always connect GND between Pi and ESP32 for I2C communication.

```
Raspberry Pi GND (Pin 6/9/14) → ESP32 GND
```

Verify with multimeter: Continuity test between Pi GND and ESP32 GND should beep.

---

### Power Supply Issues

**Symptom:** ESP32 boots but crashes during display initialization, displays flicker, random resets

**Cause:** Insufficient current from power supply

**Current Requirements:**
- ESP32-S3: ~200mA idle, ~400mA active
- 2× GC9A01 displays: ~200mA combined (backlight on)
- **Total: ~600mA minimum**

**Solutions:**

1. **Use USB 2.0 or higher** (USB 2.0 provides 500mA, USB 3.0 provides 900mA)
2. **Use powered USB hub** if connecting multiple devices
3. **Use dedicated power supply** (5V, 1A minimum)
4. **Power ESP32 from Raspberry Pi 5V pin** (if Pi has sufficient power supply):
   ```
   Pi 5V (Pin 2/4) → ESP32 VIN
   Pi GND → ESP32 GND
   ```

---

## ESP32 Firmware Issues

### PlatformIO Build Fails

**Symptom:** `pio run` fails with compilation errors

**Common Causes & Solutions:**

#### 1. Library Dependencies Missing

```bash
# Error: TFT_eSPI.h: No such file or directory
```

**Solution:** PlatformIO should auto-install libraries. If not:
```bash
cd firmware/head
pio lib install
pio run
```

#### 2. Wrong Platform Version

```bash
# Error: USE_HSPI_PORT not defined
```

**Solution:** Update `platformio.ini`:
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32@^6.7.0  # Ensure >= 6.7.0
```

#### 3. Outdated PlatformIO Core

**Solution:**
```bash
pio upgrade
pio pkg update
```

---

### ESP32 Crashes on Boot (Guru Meditation Error)

**Symptom:** Serial monitor shows:
```
Guru Meditation Error: Core 0 panic'ed (StoreProhibited)
```

**Cause:** ESP32-S3 HSPI port misconfiguration

**Solution:**

Ensure `platformio.ini` has `USE_HSPI_PORT`:
```ini
build_flags =
    -DUSE_HSPI_PORT=1  # CRITICAL for ESP32-S3
    -DGC9A01_DRIVER=1
    # ... other flags
```

Then rebuild and reflash:
```bash
pio run -t upload
```

**Reference:** This is a known issue with ESP32-S3 and espressif32 platform >= 6.7.0

---

### Firmware Upload Fails

**Symptom:** `pio run -t upload` hangs at "Connecting..." or fails

**Solutions:**

1. **Hold BOOT button** while starting upload, release after "Connecting..." appears
2. **Check USB cable** (use data-capable cable, not power-only)
3. **Try different upload speed** in `platformio.ini`:
   ```ini
   upload_speed = 460800  # Default
   # Try slower: 115200
   ```

4. **Erase flash** if firmware is corrupted:
   ```bash
   pio run -t erase
   pio run -t upload
   ```

---

### Serial Monitor Shows No Output

**Symptom:** `pio device monitor` shows blank screen

**Solutions:**

1. **Check baud rate** matches firmware (default: 115200):
   ```bash
   pio device monitor -b 115200
   ```

2. **Press RESET button** on ESP32 after connecting serial monitor

3. **Verify USB connection** supports serial data (not just power)

---

## I2C Communication Issues

### ESP32 Not Detected on I2C Bus

**Symptom:** `sudo i2cdetect -y 1` does not show device at address `0x08`

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
(No 08 shown)
```

**Possible Causes:**

1. **I2C not enabled on Raspberry Pi**
2. **Common ground not connected** (most common!)
3. **SDA/SCL wires swapped or disconnected**
4. **ESP32 firmware not running I2C slave**
5. **Wrong I2C pins configured in firmware**

**Solutions:**

#### 1. Enable I2C on Raspberry Pi

```bash
sudo raspi-config
# Navigate: Interface Options → I2C → Enable
sudo reboot

# Verify I2C device exists
ls /dev/i2c-*
# Should show: /dev/i2c-1
```

#### 2. Verify Common Ground

**CRITICAL:** Use multimeter to verify continuity between Pi GND and ESP32 GND.

```
Raspberry Pi Pin 6 (GND) ↔ ESP32 GND → Multimeter should beep
```

#### 3. Check I2C Wiring

| Raspberry Pi | Signal | ESP32-S3 GPIO |
|--------------|--------|---------------|
| GPIO 2 (Pin 3) | SDA | GPIO 8 |
| GPIO 3 (Pin 5) | SCL | GPIO 9 |
| GND (Pin 6) | GND | GND |

**Verify pins with multimeter:**
- Pi GPIO2 to ESP32 GPIO8: Continuity
- Pi GPIO3 to ESP32 GPIO9: Continuity

#### 4. Check ESP32 Firmware

Connect serial monitor and verify:
```bash
pio device monitor

# Expected output:
✓ I2C slave initialized at 0x08
```

If not shown, reflash firmware:
```bash
pio run -t upload
```

#### 5. Test I2C with Loopback

**Temporary test:** Short SDA and SCL on Pi (disconnect ESP32 first):
```bash
# This should NOT be done with ESP32 connected
sudo i2cdetect -y 1
# You should see many addresses if shorted (proves I2C works)
```

---

### I2C Permission Denied

**Symptom:**
```
OSError: [Errno 13] Permission denied: '/dev/i2c-1'
```

**Cause:** User not in `i2c` group

**Solution:**

```bash
# Add user to i2c group
sudo usermod -a -G i2c $USER

# Apply immediately (or log out/in)
newgrp i2c

# Verify
groups
# Should include: i2c
```

---

### I2C Read/Write Timeouts

**Symptom:** ROS2 driver node logs show:
```
[WARN] I2C timeout reading from 0x08
[ERROR] Failed to communicate with head module
```

**Possible Causes:**

1. **ESP32 firmware crashed or hung**
2. **I2C clock speed too high for wire length**
3. **Electrical noise on I2C bus**
4. **Pull-up resistors missing/incorrect**

**Solutions:**

1. **Check ESP32 serial monitor** for crash messages:
   ```bash
   pio device monitor
   # Look for: Guru Meditation, Stack dump, Watchdog reset
   ```

2. **Shorten I2C wires** (<30cm recommended for 400kHz):
   ```
   Current length: 50cm → Too long
   Recommended: 15-20cm
   ```

3. **Reduce I2C clock speed** (firmware config):
   ```cpp
   // firmware/head/src/config.h
   constexpr uint32_t kI2cFrequency = 100000;  // Reduce to 100kHz
   ```

4. **Check pull-up resistors**:
   - Raspberry Pi has internal 1.8kΩ pull-ups (usually sufficient)
   - For long wires, add external 4.7kΩ resistors (SDA/SCL to 3.3V)

---

## Display Issues

### One or Both GC9A01 Displays Blank

**Symptom:** Displays have backlight on but show no content (all black or white)

**Possible Causes:**

1. **SPI wiring incorrect**
2. **CS (Chip Select) pins not connected**
3. **Display power insufficient**
4. **Wrong display driver configuration**

**Solutions:**

#### 1. Verify SPI Wiring

| GC9A01 Pin | ESP32-S3 GPIO | Shared/Individual |
|-----------|---------------|-------------------|
| SCL (CLK) | GPIO 12 | Shared |
| SDA (MOSI) | GPIO 11 | Shared |
| DC (A0) | GPIO 2 | Shared |
| RST | GPIO 4 | Shared |
| CS | GPIO 5 (left) / GPIO 15 (right) | Individual |
| VCC | 3.3V | Shared |
| GND | GND | Shared |

**Check with multimeter:** Verify continuity for each connection.

#### 2. Test Displays Individually

Modify firmware to test one display at a time:

```cpp
// Temporarily disable right eye
// digitalWrite(CS_RIGHT_EYE, HIGH);  // Keep high (disabled)

// Test left eye only
digitalWrite(CS_LEFT_EYE, LOW);
tft.fillScreen(TFT_RED);
delay(2000);
```

If one works and other doesn't → Check CS wiring for non-working display.

#### 3. Check Display Power

Each GC9A01 draws ~100mA. Verify ESP32 can provide 200mA total on 3.3V rail:
- **Use USB 2.0 or better** (500mA+)
- **Check voltage at display VCC pins** with multimeter (should be 3.3V ± 0.1V)

#### 4. Verify Driver Configuration

Check `platformio.ini`:
```ini
build_flags =
    -DGC9A01_DRIVER=1    # Correct driver
    -DUSE_HSPI_PORT=1    # ESP32-S3 specific
    -DTFT_WIDTH=240
    -DTFT_HEIGHT=240
```

---

### Displays Show Garbage/Noise

**Symptom:** Displays show random pixels, static, or corrupted graphics

**Possible Causes:**

1. **SPI wires too long or poor quality**
2. **SPI clock speed too high**
3. **Shared signal integrity issues**

**Solutions:**

1. **Shorten SPI wires** (<15cm for 20MHz SPI):
   ```
   Recommended: 10cm jumper wires
   Maximum: 20cm
   ```

2. **Reduce SPI clock speed** in `platformio.ini`:
   ```ini
   -DSPI_FREQUENCY=10000000  # Reduce to 10MHz (from 20MHz)
   ```

3. **Use twisted pairs** for SCL/MOSI or keep wires close together

4. **Add decoupling capacitors** (0.1µF near each display VCC/GND)

---

### Only One Eye Works

**Symptom:** Left eye displays correctly, right eye blank (or vice versa)

**Cause:** CS (Chip Select) pin not connected or configured incorrectly

**Solutions:**

1. **Check CS wiring**:
   - Left Eye CS → ESP32 GPIO 5
   - Right Eye CS → ESP32 GPIO 15

2. **Verify CS pin configuration** in firmware:
   ```cpp
   // config.h
   constexpr uint8_t CS_LEFT_EYE = 5;
   constexpr uint8_t CS_RIGHT_EYE = 15;

   // main.cpp setup()
   pinMode(CS_LEFT_EYE, OUTPUT);
   pinMode(CS_RIGHT_EYE, OUTPUT);
   digitalWrite(CS_LEFT_EYE, HIGH);  // Deselect initially
   digitalWrite(CS_RIGHT_EYE, HIGH);
   ```

3. **Test CS pins with multimeter**:
   - Set CS LOW in code → Multimeter should read 0V
   - Set CS HIGH in code → Multimeter should read 3.3V

---

## ROS2 Node Issues

### ROS2 Node Fails to Start

**Symptom:**
```bash
ros2 run orchestrator head_driver
# Error: ModuleNotFoundError: No module named 'orchestrator'
```

**Cause:** ROS2 workspace not built or not sourced

**Solution:**

```bash
# Build workspace
cd ~/olaf/ros2
colcon build --packages-select orchestrator

# Source install
source install/setup.bash

# Now run node
ros2 run orchestrator head_driver
```

**Add to ~/.bashrc** to auto-source:
```bash
echo "source ~/olaf/ros2/install/setup.bash" >> ~/.bashrc
```

---

### Node Starts But Can't Import smbus2

**Symptom:**
```
ModuleNotFoundError: No module named 'smbus2'
```

**Cause:** Python I2C library not installed

**Solution:**

```bash
pip3 install smbus2
# Or use system package:
sudo apt install python3-smbus
```

---

### Expression Commands Not Working

**Symptom:** Publishing to `/olaf/head/expression` but eyes don't change

**Debugging Steps:**

1. **Verify node running**:
   ```bash
   ros2 node list
   # Should show: /head_driver
   ```

2. **Check topic subscription**:
   ```bash
   ros2 topic info /olaf/head/expression
   # Should show: 1 subscription (head_driver)
   ```

3. **Monitor node logs**:
   ```bash
   ros2 run orchestrator head_driver --ros-args --log-level debug
   ```

4. **Test I2C directly**:
   ```bash
   # Bypass ROS2, test I2C command directly
   sudo i2cset -y 1 0x08 0x10 0x01  # Happy expression
   sudo i2cset -y 1 0x08 0x11 0x03  # Intensity 3
   ```

5. **Check expression format**:
   ```bash
   # Correct format: "emotion,intensity"
   ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'happy,3'"

   # INVALID (missing intensity):
   ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'happy'"
   ```

---

## Performance Issues

### Low Frame Rate (<30 FPS)

**Symptom:** Eyes animate slowly, choppy motion

**Causes & Solutions:**

1. **SPI clock too slow**: Increase in `platformio.ini`:
   ```ini
   -DSPI_FREQUENCY=20000000  # 20MHz
   ```

2. **Long SPI wires**: Use <15cm jumper wires

3. **Inefficient rendering code**: Profile with ESP32 serial timing logs

---

### I2C Latency >100ms

**Symptom:** Slow response to expression changes

**Causes & Solutions:**

1. **I2C clock too slow**: Increase to 400kHz:
   ```cpp
   constexpr uint32_t kI2cFrequency = 400000;  // 400kHz
   ```

2. **Retry logic excessive**: Reduce retry count in driver node

3. **Bus contention**: Ensure only one master (Pi) on I2C bus

---

## General Debugging Tips

### Systematic Debugging Process

1. **Test hardware layer first**:
   - Multimeter: Verify power, GND continuity
   - `lsusb`: Check ESP32 connected
   - `i2cdetect`: Check I2C communication

2. **Test firmware layer**:
   - Serial monitor: Verify firmware boots
   - LED blink test: Verify code running
   - I2C slave: Check initialized message

3. **Test ROS2 layer**:
   - `ros2 node list`: Verify node running
   - `ros2 topic echo`: Monitor status
   - Direct I2C test: Bypass ROS2

### Useful Commands

```bash
# Hardware detection
lsusb | grep -i "CP210\|CH340"
ls /dev/i2c-*
sudo i2cdetect -y 1

# ESP32 serial monitoring
pio device monitor -d firmware/head

# ROS2 debugging
ros2 node list
ros2 topic list
ros2 topic echo /olaf/head/status
ros2 run orchestrator head_driver --ros-args --log-level debug

# I2C direct testing
sudo i2cget -y 1 0x08 0x00  # Read module ID
sudo i2cget -y 1 0x08 0x02  # Read status
sudo i2cset -y 1 0x08 0x10 0x01  # Set happy expression
```

### Log File Locations

- **ROS2 logs**: `~/.ros/log/`
- **Firmware serial**: `pio device monitor` (real-time only)
- **System logs**: `dmesg | grep -i usb` (USB connection issues)

---

## Getting Help

If you encounter issues not covered in this guide:

1. **Check ESP32 serial output** (90% of issues show up here)
2. **Verify wiring** with multimeter (most common hardware issue)
3. **Test components independently** (isolate problem)
4. **Check GitHub Issues**: [OLAF Repository](https://github.com/your-repo/olaf)
5. **Review Story Documentation**:
   - [Story 1.3: Hardware Assembly](stories/1.3.head-module-hardware-assembly.md)
   - [Story 1.4: ESP32 Firmware](stories/1.4.head-module-esp32-firmware.md)
   - [Story 1.5: ROS2 Driver](stories/1.5.ros2-head-driver-node.md)

---

## Change Log

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-11-02 | v1.0 | Initial troubleshooting guide for Epic 1 | Gilfoyle (Dev Agent) |
