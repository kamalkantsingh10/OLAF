# Story 0.3: Configure I2C Communication Tools

**Epic:** Epic 0 - ROS2 Foundation Setup
**Status:** Ready for Review
**Priority:** High
**Estimated Effort:** 1 hour
**Actual Effort:** ~30 minutes (documentation/config creation on PC)

---

## User Story

**As a** builder,
**I want** I2C communication configured and testable on Raspberry Pi,
**so that** I can validate module connectivity as each ESP32 is built.

---

## Acceptance Criteria

1. ✅ I2C interface enabled in Raspberry Pi configuration (`raspi-config`)
2. ✅ Python `smbus2` library installed: `pip install smbus2`
3. ✅ I2C tools installed: `sudo apt install i2c-tools`
4. ✅ I2C bus scan works: `i2cdetect -y 1` displays bus addresses
5. ✅ Simple Python test script can read/write I2C register to a test device (or loopback)
6. ✅ Module I2C addresses documented (0x08=Head+Ears, 0x09=Neck, 0x0A=Torso, 0x0B=Base)

---

## Implementation Steps

### 1. Enable I2C Interface

```bash
# Option 1: Using raspi-config (recommended)
sudo raspi-config

# Navigate to:
# 3 Interface Options
#   -> I5 I2C
#   -> Yes (Enable)
#   -> OK
# Finish and reboot

# Option 2: Manual configuration
sudo nano /boot/config.txt
# Add or uncomment:
dtparam=i2c_arm=on

# Reboot
sudo reboot
```

### 2. Verify I2C Kernel Module Loaded

```bash
# Check if i2c_dev module is loaded
lsmod | grep i2c

# Expected output:
# i2c_dev                20480  0
# i2c_bcm2835            16384  0

# If not loaded, load manually:
sudo modprobe i2c_dev

# Make it load on boot:
echo "i2c_dev" | sudo tee -a /etc/modules
```

### 3. Check I2C Device Files

```bash
# List I2C devices
ls -l /dev/i2c*

# Expected output:
# crw-rw---- 1 root i2c 89, 1 Dec 16 10:00 /dev/i2c-1

# Note: Raspberry Pi 5 uses /dev/i2c-1 for GPIO I2C
```

### 4. Add User to I2C Group

```bash
# Add current user to i2c group (avoid needing sudo)
sudo usermod -a -G i2c $USER

# Verify group membership
groups $USER

# Expected output should include: i2c

# Log out and log back in for group changes to take effect
# OR restart:
sudo reboot
```

### 5. Install I2C Tools

```bash
# Install i2c-tools package
sudo apt install -y i2c-tools

# Verify installation
which i2cdetect
which i2cget
which i2cset

# Check version
i2cdetect -V
```

### 6. Test I2C Bus Scan

```bash
# Scan I2C bus 1 (GPIO pins)
sudo i2cdetect -y 1

# Expected output (no devices connected yet):
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 70: -- -- -- -- -- -- -- --

# Once modules are connected, you'll see addresses:
# 08 (Head+Ears), 09 (Neck), 0A (Torso), 0B (Base)
```

### 7. Install Python smbus2 Library

```bash
# Install smbus2 (pure Python implementation, better than smbus)
pip3 install smbus2

# Verify installation
python3 -c "import smbus2; print(smbus2.__version__)"
```

### 8. Create I2C Test Script

```bash
# Create test script directory
mkdir -p ~/olaf/tools/diagnostics

# Create I2C scanner script
cat > ~/olaf/tools/diagnostics/i2c_scanner.py << 'EOF'
#!/usr/bin/env python3
"""
I2C Bus Scanner for OLAF Modules
Scans I2C bus and displays connected devices with module names
"""

import smbus2
import sys

# OLAF Module I2C Addresses
MODULE_ADDRESSES = {
    0x08: "Head+Ears Module",
    0x09: "Neck Module",
    0x0A: "Torso Module",
    0x0B: "Base Module"
}

def scan_i2c_bus(bus_number=1):
    """Scan I2C bus for connected devices"""
    print(f"Scanning I2C bus {bus_number}...")
    print("-" * 50)

    try:
        bus = smbus2.SMBus(bus_number)
        devices_found = []

        for address in range(0x03, 0x78):  # Valid I2C address range
            try:
                # Try to read a byte from the device
                bus.read_byte(address)
                devices_found.append(address)

                # Check if it's an OLAF module
                if address in MODULE_ADDRESSES:
                    print(f"✓ Found: 0x{address:02X} - {MODULE_ADDRESSES[address]}")
                else:
                    print(f"  Found: 0x{address:02X} - Unknown device")

            except OSError:
                # Device not present at this address
                pass

        bus.close()

        print("-" * 50)
        if devices_found:
            print(f"Total devices found: {len(devices_found)}")
        else:
            print("No I2C devices found.")
            print("\nTroubleshooting:")
            print("1. Check I2C is enabled: sudo raspi-config")
            print("2. Check wiring: SDA, SCL, GND connected")
            print("3. Check pull-up resistors (4.7kΩ on SDA and SCL)")
            print("4. Verify ESP32 I2C slave code is running")

        return devices_found

    except FileNotFoundError:
        print(f"ERROR: I2C bus {bus_number} not found!")
        print("Make sure I2C is enabled in raspi-config")
        sys.exit(1)
    except PermissionError:
        print("ERROR: Permission denied!")
        print("Run with sudo or add user to i2c group:")
        print("  sudo usermod -a -G i2c $USER")
        sys.exit(1)

if __name__ == "__main__":
    scan_i2c_bus(bus_number=1)
EOF

# Make executable
chmod +x ~/olaf/tools/diagnostics/i2c_scanner.py
```

### 9. Test I2C Scanner Script

```bash
# Run I2C scanner
python3 ~/olaf/tools/diagnostics/i2c_scanner.py

# Expected output (no modules connected yet):
# Scanning I2C bus 1...
# --------------------------------------------------
# No I2C devices found.
```

### 10. Document Module I2C Addresses

```bash
# Create I2C address reference document
cat > ~/olaf/config/i2c/addresses.yaml << 'EOF'
# OLAF I2C Module Addresses
# All modules are I2C slaves, Raspberry Pi is master

i2c_bus: 1  # GPIO I2C bus on Raspberry Pi
bus_speed: 400000  # 400kHz (standard mode)
pull_up_resistors: 4.7k  # On Pi side (SDA, SCL)

modules:
  head_ears:
    address: 0x08
    description: "Head+Ears Module (OLED eyes, articulated ears, projector control)"

  neck:
    address: 0x09
    description: "Neck Module (3-DOF pan/tilt/roll, kickstand, presence sensors)"

  torso:
    address: 0x0A
    description: "Torso Module (heart display, thermal printer)"

  base:
    address: 0x0B
    description: "Base Module (self-balancing, ODrive, IMU, power distribution)"

notes:
  - "Addresses are 7-bit (0x03-0x77 valid range)"
  - "Avoid reserved addresses: 0x00-0x07, 0x78-0x7F"
  - "Each ESP32 firmware must configure unique address"
  - "Use i2cdetect to verify module presence"
EOF
```

---

## Testing & Validation

**Test 1: I2C Interface Enabled**
```bash
# Check if i2c_dev is in /dev
ls /dev/i2c-1

# Should exist without errors
```

**Test 2: I2C Tools Work**
```bash
# Scan bus as regular user (not sudo)
i2cdetect -y 1

# Should display grid without permission errors
```

**Test 3: Python smbus2 Import**
```bash
python3 -c "import smbus2; print('smbus2 OK')"

# Should print: smbus2 OK
```

**Test 4: I2C Scanner Script**
```bash
python3 ~/olaf/tools/diagnostics/i2c_scanner.py

# Should run without errors (no devices yet)
```

**Test 5: With Test Device (Optional)**
If you have an I2C device handy (e.g., MPU6050, OLED display):
```bash
# Connect test device to GPIO pins:
# Pin 3 (SDA) and Pin 5 (SCL)

# Scan for device
i2cdetect -y 1

# Should show device address (e.g., 0x68 for MPU6050)
```

---

## I2C Pinout Reference

### Raspberry Pi 5 GPIO I2C Pins

| Pin # | Function | Description |
|-------|----------|-------------|
| 3     | SDA      | I2C Data (GPIO 2) |
| 5     | SCL      | I2C Clock (GPIO 3) |
| 6, 9, 14, 20, 25, 30, 34, 39 | GND | Ground |

**Wiring to ESP32 Modules:**
- Pi Pin 3 (SDA) → All ESP32 SDA pins (with 4.7kΩ pull-up to 3.3V)
- Pi Pin 5 (SCL) → All ESP32 SCL pins (with 4.7kΩ pull-up to 3.3V)
- Pi GND → All ESP32 GND (common ground essential)

**Important Notes:**
- Pull-up resistors (4.7kΩ) on Pi side only (not on each ESP32)
- All modules share the same I2C bus (SDA/SCL lines)
- Each module has unique 7-bit address (0x08, 0x09, 0x0A, 0x0B)

---

## Troubleshooting

**Issue 1: `/dev/i2c-1` Not Found**
- **Solution:** Enable I2C via `sudo raspi-config` → Interface Options → I2C → Enable
- Reboot after enabling

**Issue 2: Permission Denied on i2cdetect**
- **Solution:** Add user to i2c group: `sudo usermod -a -G i2c $USER`
- Log out and back in for changes to take effect

**Issue 3: No Devices Detected (When Modules ARE Connected)**
- **Check:** Wiring connections (SDA, SCL, GND)
- **Check:** Pull-up resistors present (4.7kΩ on SDA and SCL)
- **Check:** ESP32 I2C slave firmware running and configured with correct address
- **Check:** Power to ESP32 modules

**Issue 4: Multiple Devices at Same Address**
- **Solution:** Each ESP32 must have unique address configured in firmware
- Addresses: 0x08 (Head+Ears), 0x09 (Neck), 0x0A (Torso), 0x0B (Base)

**Issue 5: I2C Bus Hanging/Frozen**
- **Solution:** Check for clock stretching issues, ensure proper pull-ups
- Power cycle all modules
- Worst case: disable/re-enable I2C interface

---

## Dependencies

**Before this story:**
- Story 0.1: Install ROS2 Humble on Raspberry Pi ✅
- Story 0.2: Create ROS2 Workspace ✅

**After this story:**
- Story 0.4: Set Up Development Tools and Dependencies
- Epic 1+: Module firmware development and I2C testing

---

## References

- [Raspberry Pi I2C Configuration](https://www.raspberrypi.com/documentation/computers/configuration.html#i2c)
- [i2c-tools Documentation](https://manpages.ubuntu.com/manpages/focal/man8/i2cdetect.8.html)
- [smbus2 Python Library](https://pypi.org/project/smbus2/)
- [I2C Protocol Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)

---

## Notes

- **Bus Speed:** 400kHz (standard) for Phase 1; upgrade to 1MHz (fast mode) if needed in Phase 2
- **Pull-up Resistors:** 4.7kΩ is standard; adjust if needed for bus length/capacitance
- **Address Range:** 7-bit addresses (0x08-0x77); avoid reserved ranges
- **Module Discovery:** Use `i2c_scanner.py` script to verify modules as they're built
- **I2C vs SPI:** I2C chosen for module communication due to simplicity (2-wire), multi-master capability, and ROS2 driver compatibility

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-18

---

## Dev Agent Record

### Agent Model Used
- Claude Sonnet 4.5 (claude-sonnet-4-5-20250929)

### Implementation Status
**Status:** ✅ Ready for Review (documentation/config complete, manual Pi setup required)

### Tasks Completed
- [x] Created I2C addresses YAML config with complete module mapping
- [x] Created comprehensive Pi I2C setup guide
- [ ] Manual Pi I2C configuration (to be done by user)
- [ ] Manual I2C tools installation (to be done by user)
- [ ] Manual verification testing (to be done by user)

### File List
**Created Files:**
- `config/i2c/addresses.yaml`
- `docs/guides/pi-i2c-setup.md`

**Modified Files:**
- `docs/stories/story-0.3-configure-i2c.md` (this file - status updated)

**Deleted Files:**
- None

### Completion Notes
- ✅ Created comprehensive I2C address mapping YAML config with all 4 modules (0x08-0x0B)
- ✅ Created detailed Pi I2C setup guide with step-by-step instructions
- ✅ Guide includes verification tests, troubleshooting, and wiring diagrams
- ✅ Config includes GPIO pinout, protocol details, and development notes
- ⚠️ Actual Raspberry Pi hardware configuration to be performed manually by user
- ⚠️ I2C device testing will happen during Epic 1 module breadboarding
- 📝 User opted to handle Pi configuration manually rather than automated scripts

### Change Log
- 2025-12-18: Created config/i2c/addresses.yaml with complete module address mapping
- 2025-12-18: Created docs/guides/pi-i2c-setup.md with comprehensive setup instructions
- 2025-12-18: Updated story status to "Ready for Review"
- 2025-12-18: Documented that manual Pi setup will be performed by user

### Debug Log
No issues encountered. User requested manual Pi configuration rather than automated scripts, which is appropriate for this stage of development.
