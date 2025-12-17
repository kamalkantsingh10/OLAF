# Story 4.12: Mount Base Module and Connect All Modules

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** to mount the Base module and establish all mechanical and electrical connections to upper modules,
**so that** the complete robot is assembled and operational.

---

## Acceptance Criteria

1. ✅ Base platform mounted as lowest module
2. ✅ Torso module bolted to Base platform (top mounting points)
3. ✅ Power distribution: Battery powers all modules (36V → buck converters → Pi/ESP32s)
4. ✅ I2C bus: All modules connected (0x08, 0x09, 0x0A, 0x0B)
5. ✅ All ROS2 driver nodes launch successfully
6. ✅ Robot balances autonomously on two wheels
7. ✅ All subsystems tested: head moves, heart animates, kickstand deploys, robot moves
8. ✅ Full robot documented with photos

---

## Implementation Steps

### 1. Mount Base PCB to Platform

```bash
# Install M3 standoffs (10mm) to platform
# Bolt Base PCB onto standoffs
# Connect battery via XT60 connector
# Connect ODrive via XT60 connector
# Connect buck converter outputs to Pi and ESP32s
```

### 2. Install Battery on Platform

```bash
# Center battery on platform for weight balance
# Secure with Velcro straps or metal bracket
# Connect battery to Base PCB XT60 input
# Verify polarity before connecting!
```

### 3. Mount Torso to Base

```bash
# Align Torso bottom mounting bracket with Base platform top
# Use M5 bolts (×4) to secure connection
# Route power cable from Base to Torso (USB-C to Pi)
# Route I2C cable from Base PCB to Torso Pi
```

### 4. Establish I2C Bus Connections

```bash
# I2C bus topology (all modules on same bus):
#   Pi GPIO 2/3 (SDA/SCL)
#     ├─ Head+Ears ESP32 (0x08)
#     ├─ Torso ESP32 (0x09)
#     ├─ Neck ESP32 (0x0A)
#     └─ Base ESP32 (0x0B)

# Verify all devices present:
i2cdetect -y 1
# Expected: 0x08, 0x09, 0x0A, 0x0B
```

### 5. Test Power Distribution

```bash
# Power on battery (master switch)
# Verify voltages:
#   Battery: 36-42V
#   Pi USB-C: 5.0V
#   ESP32s: 12V (to onboard regulators) or 5V direct
# All modules should boot (LEDs light up)
```

### 6. Launch All ROS2 Drivers

```bash
# Create master launch file: olaf_bringup.launch.py
ros2 launch olaf_bringup all_drivers.launch.py

# Verify all nodes running:
ros2 node list
# Expected:
#   /head_ears_driver
#   /torso_driver
#   /neck_driver
#   /base_driver
```

### 7. Test Full System Integration

```bash
# Test 1: Enable balancing
ros2 service call /base/enable_balance std_srvs/srv/SetBool "{data: true}"
# Robot should balance upright

# Test 2: Head/ears animation
ros2 topic pub --once /head_ears/eyes std_msgs/msg/String "{data: 'blink'}"

# Test 3: Heart display
ros2 topic pub --once /torso/heart std_msgs/msg/String "{data: 'beat:255,0,0:128'}"

# Test 4: Kickstand
ros2 topic pub --once /neck/kickstand std_msgs/msg/String "{data: 'deploy'}"
ros2 topic pub --once /neck/kickstand std_msgs/msg/String "{data: 'retract'}"

# Test 5: Movement
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 8. Verify Mechanical Stability

```bash
# Check all mounting bolts tight
# Verify no cable strain
# Check weight distribution (robot shouldn't tip)
# Test emergency stop (accessible and functional)
```

### 9. Calibrate Sensors and Actuators

```bash
# IMU: Verify pitch reads 0° when robot upright
# Motors: Verify positive velocity moves forward
# Servos: Verify full range of motion (no binding)
# Display/printer: Verify accessible and functional
```

### 10. Document Complete Assembly

**Create `docs/assembly/complete_robot.md`:**

```markdown
# OLAF Complete Assembly Documentation

## Specifications
- Height: ~1.2m (with head)
- Width: 0.3m (wheelbase)
- Weight: ~17kg total
- Battery: 36V 4.4Ah (45min runtime)

## Module Stack (bottom to top):
1. Base: Motors, battery, balancing control
2. Torso: Pi 5, heart display, printer
3. Neck: Kickstand, tilt servo
4. Head+Ears: Eyes (OLEDs), ears (servos), camera

## Power Distribution
- 36V battery → Base platform
- Base buck converters → 5V (Pi), 12V (ESP32s)
- Total power: ~200W typical, 780W peak

## I2C Network
- All modules: 0x08 (Head), 0x09 (Torso), 0x0A (Neck), 0x0B (Base)
- Bus speed: 400kHz
- Total cable length: <1m

## ROS2 Topics
- `/cmd_vel`: Robot movement commands
- `/odom`: Robot odometry
- `/head_ears/*`: Eye/ear control
- `/torso/*`: Heart/printer control
- `/neck/*`: Kickstand/tilt control
- `/base/*`: IMU, battery status

## Startup Sequence
1. Power on battery (master switch)
2. Wait for Pi boot (~30s)
3. SSH into Pi: `ssh olaf@olaf-pi5.local`
4. Launch drivers: `ros2 launch olaf_bringup all_drivers.launch.py`
5. Enable balancing: `ros2 service call /base/enable_balance ...`
6. Robot ready!

## Emergency Procedures
- E-stop button: Immediately cuts motor power
- Low battery: Robot disables automatically at 32V
- Tilt limit: Robot disables at 30° tilt
- Manual shutdown: `ros2 service call /base/enable_balance ... false`

## Photos
[Insert photos of complete robot from multiple angles]
```

---

## Testing & Validation

**Test 1: All Modules Powered**
```bash
# All LEDs on, no smoke, correct voltages
```

**Test 2: I2C Communication**
```bash
# All 4 module addresses detected
```

**Test 3: Full System Balance**
```bash
# Robot balances for >60 seconds
```

**Test 4: All Subsystems Functional**
```bash
# Eyes, heart, kickstand, movement all working
```

---

## Troubleshooting

**Issue 1: Robot Won't Balance**
- **Solution:** Check IMU orientation, verify motor directions, enable balancing service

**Issue 2: Module Not Detected on I2C**
- **Solution:** Check wiring, verify module powered, check I2C address conflicts

**Issue 3: Power Supply Issues**
- **Solution:** Check battery voltage, verify buck converter outputs, check fuses

---

## Dependencies

**Before this story:**
- Story 4.11: Create Base ROS2 Driver Node ✅
- Story 3.9: Mount Torso Module ✅
- Story 2.9: Mount Neck Module ✅
- Story 1.8: Mount Head+Ears Module ✅

**After this story:**
- Epic 5: End-to-End Demo

---

## Notes

- **Complete Robot:** All mechanical and electrical systems integrated
- **Safety:** Always have E-stop accessible during testing
- **Balance First:** Ensure balancing works before testing movement
- **Cable Management:** Clean routing prevents snagging and damage
- **Future:** Add bumpers, emergency stop on remote, status display on torso

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
