# Raspberry Pi I2C Setup Guide


## Overview

This guide walks you through configuring I2C communication on your ***Raspberry Pi*** 5 for OLAF module connectivity. I2C is the communication protocol used between the Raspberry Pi orchestrator and all four ESP32 modules (Head+Ears, Neck, Torso, Base).

**What you'll accomplish:**
- Enable I2C interface on Raspberry Pi
- Install I2C tools and Python libraries
- Verify I2C bus is accessible
- Prepare for module connectivity testing

---

## Prerequisites

- Raspberry Pi 5 with Raspberry Pi OS (Debian 12 Bookworm, 64-bit)
- Network connection (for package installation)
- Sudo privileges
- Terminal access (SSH or direct)

---

## Step 1: Enable I2C Interface

### Option A: Using raspi-config (Recommended)

```bash
sudo raspi-config
```

Navigate through the menu:
1. Select **"3 Interface Options"**
2. Select **"I5 I2C"**
3. Select **"Yes"** to enable
4. Select **"OK"**
5. Select **"Finish"**
6. **Reboot when prompted**

### Option B: Manual Configuration

```bash
# Edit boot config
sudo nano /boot/config.txt

# Add or uncomment this line:
dtparam=i2c_arm=on

# Save (Ctrl+X, Y, Enter) and reboot
sudo reboot
```

---

## Step 2: Verify I2C Kernel Module

After rebooting, verify the I2C kernel module is loaded:

```bash
# Check loaded modules
lsmod | grep i2c
```

**Expected output:**
```
i2c_dev                20480  0
i2c_bcm2835            16384  0
```

**If module not loaded:**
```bash
# Load module manually
sudo modprobe i2c_dev

# Make it load on boot
echo "i2c_dev" | sudo tee -a /etc/modules
```

---

## Step 3: Check I2C Device Files

```bash
# List I2C devices
ls -l /dev/i2c*
```

**Expected output:**
```
crw-rw---- 1 root i2c 89, 1 Dec 18 10:00 /dev/i2c-1
```

**Note:** Raspberry Pi 5 uses `/dev/i2c-1` for GPIO I2C (pins 3 and 5).

---

## Step 4: Configure User Permissions

Add your user to the `i2c` group to access I2C without sudo:

```bash
# Add current user to i2c group
sudo usermod -a -G i2c $USER

# Verify group membership
groups $USER
```

**Expected output should include:** `i2c`

**Important:** You must **log out and log back in** for group changes to take effect, or reboot:

```bash
# Option 1: Logout/login
exit  # Then SSH back in

# Option 2: Reboot
sudo reboot
```

**Verify permissions after re-login:**
```bash
groups  # Should show 'i2c' in the list
```

---

## Step 5: Install I2C Tools

```bash
# Update package list
sudo apt update

# Install i2c-tools
sudo apt install -y i2c-tools

# Verify installation
which i2cdetect
which i2cget
which i2cset

# Check version
i2cdetect -V
```

**Expected commands available:**
- `i2cdetect` - Scan I2C bus for devices
- `i2cget` - Read byte from I2C device register
- `i2cset` - Write byte to I2C device register
- `i2cdump` - Dump all registers from I2C device

---

## Step 6: Install Python smbus2 Library

```bash
# Install smbus2 (pure Python I2C library)
pip3 install smbus2

# Verify installation
python3 -c "import smbus2; print(f'smbus2 version: {smbus2.__version__}')"
```

**Expected output:**
```
smbus2 version: 0.4.3
```

---

## Step 7: Test I2C Bus Scan

Test that you can scan the I2C bus **without sudo**:

```bash
i2cdetect -y 1
```

**Expected output (no devices connected yet):**
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

**What this means:**
- The bus scan completed successfully
- No I2C devices are currently connected (all show `--`)
- Your permissions are correct (no "Permission denied" error)

---

## Step 8: Test Python I2C Access

Verify Python can access the I2C bus:

```bash
python3 << 'EOF'
import smbus2

try:
    bus = smbus2.SMBus(1)
    print("✓ I2C bus 1 opened successfully")
    bus.close()
    print("✓ Python I2C access confirmed")
except FileNotFoundError:
    print("✗ I2C bus not found - check raspi-config")
except PermissionError:
    print("✗ Permission denied - check i2c group membership")
EOF
```

**Expected output:**
```
✓ I2C bus 1 opened successfully
✓ Python I2C access confirmed
```

---

## Verification Checklist

Run through these quick tests to confirm everything works:

### ✅ Test 1: I2C Device File Exists
```bash
ls /dev/i2c-1
```
**Pass if:** File exists without errors

### ✅ Test 2: Bus Scan Without Sudo
```bash
i2cdetect -y 1
```
**Pass if:** Displays grid without permission errors

### ✅ Test 3: Python smbus2 Import
```bash
python3 -c "import smbus2; print('OK')"
```
**Pass if:** Prints "OK"

### ✅ Test 4: Group Membership
```bash
groups | grep i2c
```
**Pass if:** Shows "i2c" in output

---

## OLAF Module I2C Addresses

Once you start building modules, they will appear at these addresses:

| Address | Module | Components |
|---------|--------|------------|
| `0x08` | **Head+Ears** | OLED eyes, articulated ears, projector |
| `0x09` | **Neck** | 3-DOF servos, kickstand, mmWave sensors |
| `0x0A` | **Torso** | Heart display, thermal printer |
| `0x0B` | **Base** | Self-balancing, ODrive, IMU |

**When a module is connected, `i2cdetect` will show:**
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- 08 09 0a 0b -- -- -- --
```

See `config/i2c/addresses.yaml` for complete module details.

---

## Raspberry Pi I2C Pinout

### GPIO Pins for I2C

| Pin # | Function | GPIO | Description |
|-------|----------|------|-------------|
| 3 | **SDA** | GPIO 2 | I2C Data line |
| 5 | **SCL** | GPIO 3 | I2C Clock line |
| 6, 9, 14, 20, 25, 30, 34, 39 | **GND** | - | Ground (any) |

### Wiring to ESP32 Modules

All four ESP32 modules connect to the **same I2C bus**:

```
Raspberry Pi Pin 3 (SDA) ──[4.7kΩ to 3.3V]──┬─→ ESP32 Module 1 SDA
                                              ├─→ ESP32 Module 2 SDA
                                              ├─→ ESP32 Module 3 SDA
                                              └─→ ESP32 Module 4 SDA

Raspberry Pi Pin 5 (SCL) ──[4.7kΩ to 3.3V]──┬─→ ESP32 Module 1 SCL
                                              ├─→ ESP32 Module 2 SCL
                                              ├─→ ESP32 Module 3 SCL
                                              └─→ ESP32 Module 4 SCL

Raspberry Pi GND ───────────────────────────┬─→ ESP32 Module 1 GND
                                             ├─→ ESP32 Module 2 GND
                                             ├─→ ESP32 Module 3 GND
                                             └─→ ESP32 Module 4 GND
```

**Important notes:**
- **Pull-up resistors (4.7kΩ)** on Pi side only, not on each ESP32
- **Common ground** is essential - all modules must share GND with Pi
- **Bus length** should be under 1 meter for reliable 400kHz operation

---

## Troubleshooting

### Problem: `/dev/i2c-1` Not Found

**Symptoms:**
```bash
ls: cannot access '/dev/i2c-1': No such file or directory
```

**Solution:**
1. Enable I2C in raspi-config:
   ```bash
   sudo raspi-config → Interface Options → I2C → Enable
   ```
2. Reboot: `sudo reboot`
3. Verify: `ls /dev/i2c-1`

---

### Problem: Permission Denied on i2cdetect

**Symptoms:**
```bash
i2cdetect -y 1
# Error: Could not open file `/dev/i2c-1': Permission denied
```

**Solution:**
1. Add user to i2c group:
   ```bash
   sudo usermod -a -G i2c $USER
   ```
2. **Log out and log back in** (or reboot)
3. Verify: `groups | grep i2c`
4. Test: `i2cdetect -y 1` (should work without sudo)

---

### Problem: No Devices Detected (When Modules ARE Connected)

**Symptoms:**
- `i2cdetect -y 1` shows all `--` even with modules connected

**Check these:**

1. **Wiring connections:**
   ```bash
   # Verify connections:
   # Pi Pin 3 → All ESP32 SDA
   # Pi Pin 5 → All ESP32 SCL
   # Pi GND → All ESP32 GND
   ```

2. **Pull-up resistors:**
   - Need 4.7kΩ resistor from SDA to 3.3V
   - Need 4.7kΩ resistor from SCL to 3.3V
   - Resistors on Pi side only

3. **ESP32 power:**
   ```bash
   # Check ESP32 LEDs are lit
   # Verify 3.3V or 5V power supply connected
   ```

4. **ESP32 firmware:**
   - Ensure I2C slave firmware is flashed
   - Verify correct I2C address configured in firmware
   - Check firmware is running (Serial monitor shows output)

5. **I2C address conflicts:**
   - Each ESP32 must have unique address (0x08, 0x09, 0x0A, 0x0B)
   - Flash firmware one module at a time to test

---

### Problem: Bus Frozen / Hanging

**Symptoms:**
- `i2cdetect` hangs and never completes
- Commands time out

**Solutions:**

1. **Check for short circuits:**
   ```bash
   # Disconnect all modules
   # Test i2cdetect -y 1 with nothing connected
   # Reconnect modules one at a time
   ```

2. **Verify pull-up resistors:**
   - Should be 4.7kΩ (not too low, not too high)
   - Measure resistance: SDA to 3.3V, SCL to 3.3V

3. **Power cycle everything:**
   ```bash
   sudo reboot  # Restart Pi
   # Power cycle all ESP32 modules
   ```

4. **Reset I2C interface:**
   ```bash
   sudo rmmod i2c_bcm2835
   sudo modprobe i2c_bcm2835
   ```

---

## Next Steps

After completing this guide:

1. **Mark Story 0.3 complete** - I2C configuration is done
2. **Proceed to Story 0.4** - Setup Development Tools
3. **Ready for Epic 1** - Begin Head+Ears module breadboarding

When you start building modules in Epic 1, you'll use `i2cdetect` to verify each module appears at its expected address.

---

## Reference Files

- **I2C Addresses:** `config/i2c/addresses.yaml` - Complete module address map
- **Tech Stack:** `docs/architecture/tech-stack.md` - I2C specifications
- **PC Dev Setup:** `docs/guides/pc-development-setup.md` - Hybrid PC+Pi workflow

---

## Additional Resources

- [Raspberry Pi I2C Documentation](https://www.raspberrypi.com/documentation/computers/configuration.html#i2c)
- [i2c-tools Manual](https://manpages.ubuntu.com/manpages/focal/man8/i2cdetect.8.html)
- [smbus2 PyPI Page](https://pypi.org/project/smbus2/)
- [I2C Protocol Specification (NXP)](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)

---

**Guide Version:** 1.0
**Last Updated:** 2025-12-18
**Story:** 0.3 - Configure I2C Communication Tools
