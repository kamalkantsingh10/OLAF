# Story 3.9: Mount Torso Module to Robot Frame

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** Medium
**Estimated Effort:** 3-4 hours

---

## User Story

**As a** builder,
**I want** to mount the Torso module securely to the robot frame, connecting to Neck above and Base below,
**so that** the module is structurally integrated and all power/data connections are established.

---

## Acceptance Criteria

1. ✅ Torso module physically mounted to robot frame with secure bolted connections
2. ✅ Top mounting interface connects to Neck module (M5 bolts, metal bracket)
3. ✅ Bottom mounting interface connects to Base module (M5 bolts, metal bracket)
4. ✅ Power connection established: Base module supplies 5V/5A to Pi 5 via USB-C
5. ✅ I2C bus connections verified: All modules communicate (0x08, 0x09, 0x0A, 0x0B)
6. ✅ Cable routing is clean and strain-relieved
7. ✅ Module is stable (no wobbling) and can support Neck + Head+Ears weight (~2kg)
8. ✅ All ROS2 drivers functional after mounting
9. ✅ Thermal management verified: Pi stays under 80°C under load

---

## Implementation Steps

### 1. Prepare Mounting Hardware

**Components needed:**
- 4× M5 bolts (25mm length) for top Neck connection
- 4× M5 bolts (20mm length) for bottom Base connection
- 8× M5 lock washers
- 8× M5 nuts
- 2× Metal mounting plates (80mm × 60mm × 3mm steel) - installed in Story 3.5
- Rubber grommets for cable routing holes
- Zip ties for cable management
- Heat-shrink tubing for cable protection

### 2. Verify Torso Module Components

**Pre-mounting checklist:**

```bash
# On Raspberry Pi (inside Torso)
ssh olaf@olaf-pi5.local

# Verify all subsystems functional
ros2 run torso_driver torso_driver_node

# In another terminal, test commands
ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: 'beat:255,0,0:128'}"
ros2 topic pub --once /torso/print std_msgs/msg/String "{data: 'Test print before mounting'}"

# Verify I2C devices visible
i2cdetect -y 1
# Should see Torso ESP32 at 0x09

# Verify Hailo AI Kit
hailortcli fw-control identify

# Check temperature
vcgencmd measure_temp
# Should be reasonable (40-60°C idle)

# Shutdown Pi for mounting
sudo shutdown -h now
```

### 3. Position Torso Module on Frame

**Mounting sequence:**

```bash
# 1. Place Torso module in vertical position
#    - Front of module (heart display) facing forward
#    - Printer output slot on left side for accessibility
#    - Top metal bracket aligned with Neck module bottom mounting points

# 2. Dry fit check:
#    - Verify bolt holes align with Neck and Base mounting points
#    - Check clearance for cables (power, I2C)
#    - Ensure ventilation holes not blocked by frame

# 3. Mark cable routing paths on enclosure
#    - Top: I2C cable bundle to Neck (and through to Head+Ears)
#    - Bottom: Power cable from Base, I2C cable to Base
```

### 4. Connect to Neck Module (Top)

**Mechanical connection:**

```bash
# 1. Align Torso top metal bracket with Neck bottom mounting plate
# 2. Insert M5 bolts through:
#    Neck bottom plate -> Torso top bracket -> Kitchen bin plastic
# 3. Add lock washer and nut on inside of Torso
# 4. Tighten bolts in cross pattern (prevent warping)
# 5. Verify connection is rigid (no play)

# Torque specification: 3-4 Nm (hand-tight with allen key)
```

**Cable connections (top):**

```bash
# Route I2C cable from Pi inside Torso to Neck module
# Cable path: Pi GPIO -> Torso PCB -> Through top grommet -> Neck PCB

# I2C wiring:
#   Pi GPIO 2 (SDA) -> Torso PCB SDA -> Neck PCB SDA -> Head+Ears PCB SDA
#   Pi GPIO 3 (SCL) -> Torso PCB SCL -> Neck PCB SCL -> Head+Ears PCB SCL
#   GND -> Common ground across all modules

# Use 4-conductor cable (SDA, SCL, 3.3V, GND)
# Cable gauge: 22 AWG minimum
# Length: ~300mm with slack for strain relief
```

### 5. Connect to Base Module (Bottom)

**Mechanical connection:**

```bash
# 1. Align Torso bottom metal bracket with Base top mounting plate
# 2. Insert M5 bolts through:
#    Base top plate -> Torso bottom bracket -> Kitchen bin plastic
# 3. Add lock washer and nut on inside of Torso
# 4. Tighten bolts in cross pattern
# 5. Verify connection is rigid

# Test weight bearing:
#    - Module should support full robot weight above (Neck + Head+Ears ~2kg)
#    - No flexing or creaking sounds
```

**Cable connections (bottom):**

```bash
# Power cable: Base 5V/5A buck converter -> Pi USB-C
#   - Use high-quality USB-C cable (rated for 5A/27W)
#   - Route through bottom grommet
#   - Secure with zip tie near connector (strain relief)
#   - Length: ~200mm

# I2C cable: Base PCB -> Torso PCB -> Pi
#   - 4-conductor cable (SDA, SCL, 3.3V, GND)
#   - Route through same bottom grommet
#   - Length: ~200mm

# Cable bundle organization:
#   - Group power and I2C cables together
#   - Use cable sleeve or spiral wrap
#   - Secure to Torso enclosure wall with adhesive clips
```

### 6. Verify Structural Integrity

**Mechanical tests:**

```bash
# Test 1: Wobble test
#   - Gently push Torso left/right
#   - No movement at mounting points
#   - Kitchen bin should not flex

# Test 2: Weight test
#   - Apply downward force on top (simulating Neck + Head weight)
#   - No creaking or bolt loosening
#   - Metal brackets should not bend

# Test 3: Cable strain test
#   - Gently pull on cables near connectors
#   - Cables should have slack (not taut)
#   - Grommets prevent cable chafing

# If any test fails:
#   - Tighten bolts
#   - Add reinforcement (additional metal brackets)
#   - Redistribute weight
```

### 7. Power On and Test All Modules

**System integration test:**

```bash
# 1. Power on Base module (supplies power to all modules)
# 2. Wait for Pi to boot (30-60 seconds)

# 3. SSH into Pi
ssh olaf@olaf-pi5.local

# 4. Scan I2C bus - verify all modules present
i2cdetect -y 1

# Expected output:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- 08 09 0a 0b -- -- -- --
# 10:          -- -- -- -- -- -- -- -- -- -- -- -- --

# Addresses:
#   0x08: Head+Ears
#   0x09: Torso (ESP32)
#   0x0A: Neck
#   0x0B: Base

# 5. Launch all ROS2 driver nodes
cd ~/olaf/ros2
source install/setup.bash

ros2 launch olaf_bringup all_drivers.launch.py
# (Or launch each driver individually for debugging)

# 6. Verify all nodes running
ros2 node list

# Expected:
#   /head_ears_driver
#   /torso_driver
#   /neck_driver
#   /base_driver

# 7. Test each module with ROS2 commands
# Head+Ears: Blink eyes
ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: 'blink'}"

# Torso: Heart animation
ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: 'beat:255,0,0:150'}"

# Torso: Print message
ros2 topic pub --once /torso/print std_msgs/msg/String "{data: 'Torso mounted successfully!'}"

# Neck: Deploy kickstand
ros2 topic pub --once /neck/kickstand std_msgs/msg/String "{data: 'deploy'}"

# Neck: Tilt head
ros2 topic pub --once /neck/tilt std_msgs/msg/Int16 "{data: 15}"

# Base: Test motor response (if safe)
# ros2 topic pub --once /base/cmd_vel geometry_msgs/msg/Twist ...
# (Full base testing in Story 4.12)
```

### 8. Cable Management and Finalization

**Clean up installation:**

```bash
# 1. Organize all cables inside Torso enclosure
#    - Use adhesive cable clips to secure to walls
#    - Route cables away from moving parts (fan)
#    - Keep cables away from heat sources (Pi CPU, Hailo)

# 2. External cable routing (top and bottom)
#    - Use cable sleeve or spiral wrap
#    - Secure with zip ties every 100mm
#    - Prevent cables from snagging on frame edges

# 3. Label cables (optional but recommended)
#    - Power: Red label
#    - I2C: Blue label
#    - Ground: Black label

# 4. Take photos for documentation
#    - Front, back, left, right views
#    - Close-ups of mounting points
#    - Cable routing details
```

### 9. Thermal Management Verification

**Monitor temperatures under load:**

```bash
# SSH into Pi
ssh olaf@olaf-pi5.local

# Install monitoring tools
sudo apt install htop lm-sensors

# Run stress test (AI + CPU load)
stress --cpu 4 --timeout 300s &
hailortcli benchmark yolov5m.hef &

# Monitor temperature in real-time
watch -n 1 vcgencmd measure_temp

# Expected:
#   Idle: 40-50°C
#   Under load: 60-75°C
#   Max acceptable: 80°C

# If temperature exceeds 80°C:
#   - Check fan is spinning (listen for noise, feel airflow)
#   - Add more ventilation holes to enclosure
#   - Verify heatsink thermal paste properly applied
#   - Consider upgrading to higher CFM fan
```

### 10. Document Final Configuration

**Create `modules/torso/mounting.md`:**

```markdown
# Torso Module Mounting Documentation

## Mounting Points
- **Top:** 4× M5 bolts to Neck module bottom plate
- **Bottom:** 4× M5 bolts to Base module top plate
- **Torque:** 3-4 Nm (hand-tight)

## Cable Connections

### Top (to Neck)
- I2C: 4-conductor cable (SDA, SCL, 3.3V, GND)
- Length: 300mm

### Bottom (from Base)
- Power: USB-C cable, 5V/5A
- I2C: 4-conductor cable
- Length: 200mm each

## Module Addresses (I2C)
- 0x08: Head+Ears
- 0x09: Torso ESP32
- 0x0A: Neck
- 0x0B: Base

## Weight
- Torso module: ~3.5kg (enclosure + Pi + components)
- Supports: Neck (0.5kg) + Head+Ears (1.5kg) = 2kg above

## Thermal Performance
- Idle: 45°C
- Load: 70°C
- Fan: 40mm × 10mm, 5V PWM

## Photos
[Include mounting photos here]
```

---

## Testing & Validation

**Test 1: Structural Stability**
```bash
# Apply 5kg downward force on top
# No flexing, bolts remain tight
```

**Test 2: All Module Communication**
```bash
i2cdetect -y 1
# All 4 addresses present: 0x08, 0x09, 0x0A, 0x0B
```

**Test 3: ROS2 Integration**
```bash
ros2 node list
# All driver nodes running

ros2 topic list
# All module topics available
```

**Test 4: Power Distribution**
```bash
# Pi running + all ESP32s powered
# No voltage drops or brownouts
vcgencmd measure_volts
# Should show 5.0-5.2V
```

**Test 5: Thermal Under Load**
```bash
# 5-minute stress test
# Temperature stays below 80°C
```

---

## Troubleshooting

**Issue 1: Module Wobbles**
- **Solution:** Tighten bolts, add lock washers, reinforce metal brackets

**Issue 2: I2C Device Missing**
- **Solution:** Check cable connections, verify module powered, check I2C address conflicts

**Issue 3: Pi Not Booting**
- **Solution:** Check USB-C power cable, verify 5V/5A from Base, check SD card seated

**Issue 4: ROS2 Nodes Crash**
- **Solution:** Check system resources, verify all I2C devices present, check logs

**Issue 5: Overheating**
- **Solution:** Verify fan spinning, add ventilation, reduce CPU load

**Issue 6: Cable Strain**
- **Solution:** Add slack to cables, use strain relief clips, reroute cables

---

## Dependencies

**Before this story:**
- Story 3.5: Assemble Torso Enclosure ✅
- Story 3.7: Create Torso ROS2 Driver Node ✅
- Story 3.8: Install and Configure Raspberry Pi 5 in Torso Module ✅
- Story 2.9: Mount Neck Module ✅ (for top connection)
- Story 4.12: Mount Base Module ✅ (for bottom connection - can be done in parallel)

**After this story:**
- Torso module fully integrated
- Epic 5: End-to-End Demo

---

## References

- [Mechanical Mounting Best Practices](https://www.mcmaster.com/bolts/)
- [I2C Bus Cable Length Limits](https://www.ti.com/lit/an/slva689/slva689.pdf)
- [Cable Management Techniques](https://www.cablematters.com/blog/Cable-Management)

---

## Notes

- **Mounting Order:** Typically Neck first, then Torso, then Base. Can be adjusted based on workspace.
- **Kitchen Bin Strength:** Metal reinforcement bars critical for supporting weight. Test before mounting.
- **Cable Slack:** Always leave 50-100mm extra cable for strain relief and future maintenance.
- **I2C Cable Length:** Keep under 1 meter total to avoid signal degradation. Use shielded cable if >500mm.
- **Power Cable Rating:** USB-C cable must support 5A. Many cheap cables only rated for 3A.
- **Vibration:** Consider adding rubber washers at mounting points to dampen vibration from motors.
- **Future Enhancements:** Add quick-disconnect connectors for easier module removal, RGB status LEDs visible from outside.

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
