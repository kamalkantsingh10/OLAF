# Story 2.9: Mount Neck Module to Robot Frame

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** Medium
**Estimated Effort:** 3-4 hours

---

## User Story

**As a** builder,
**I want** Neck module physically mounted to robot frame with all cables connected,
**so that** the module is integrated into the robot structure and can support the head.

---

## Acceptance Criteria

1. ✅ Module mounted securely to robot frame (connecting Torso to Head+Ears)
2. ✅ Servos positioned to allow full pan/tilt/roll range of motion
3. ✅ Kickstand mechanism can deploy and retract without obstruction
4. ✅ Presence sensors positioned for 360° coverage
5. ✅ I2C cable connected to Pi
6. ✅ Power cables connected to Base module power distribution
7. ✅ Cable management ensures no loose wires interfering with neck movement
8. ✅ Module responds to ROS2 commands when powered
9. ✅ Mechanical load test: neck can support weight of Head+Ears module without sagging

---

## Implementation Steps

### 1. Prepare Mounting Hardware

**Hardware:**
- M4 or M5 screws (×2) for Torso connection (bottom of Neck)
- M4 screws (×2) for Head+Ears connection (top of Neck)
- Cable ties for cable management
- Heat shrink or cable sleeving

### 2. Mount to Torso Module

```bash
# Bottom of Neck enclosure has mounting holes
# Align with top of Torso module
# Secure with M5 screws
# Verify stable, no wobble
```

### 3. Connect Head+Ears Module

```bash
# Top of Neck (roll servo mounting point) connects to Head+Ears
# Use M4 screws through roll-head connector
# Verify weight distributed properly
# Test pan/tilt/roll motion with head attached
```

### 4. Route Cables

**Power:**
- 7.4V for servos from Base power distribution
- 5V for logic from Base power distribution
- Route along frame, zip tie every 10cm

**I2C:**
- From Pi (Torso) to Neck PCB
- Bundle with power cables

**Sensors:**
- Internal to Neck module (already connected to PCB)

### 5. Test Kickstand Deployment

```bash
# Deploy kickstand
# Verify clears Torso and Base modules
# Contacts floor at proper angle
# Supports full robot weight
```

### 6. Functional Test

```bash
# Power on robot
# Launch neck driver: ros2 run neck_driver neck_driver_node
# Test pan/tilt/roll: ros2 topic pub /neck/position ...
# Test kickstand: ros2 topic pub /neck/kickstand ...
# Verify presence sensors: ros2 topic echo /neck/presence
```

### 7. Load Test

```bash
# With Head+Ears mounted:
# Command neck through full range of motion
# Verify no sagging, binding, or servo stalls
# Check for mechanical interference
```

---

## Testing & Validation

**Test 1: Range of Motion**
- Pan ±90° (smooth, no binding)
- Tilt +60° / -30° (full range)
- Roll ±20° (head tilts side-to-side)

**Test 2: Kickstand Stability**
- Deploy kickstand, turn off balancing
- Robot remains upright on kickstand alone
- No tipping or instability

**Test 3: Sensor Coverage**
- Walk around robot 360°
- Verify front sensor detects front hemisphere
- Rear sensor detects rear

---

## Troubleshooting

**Issue: Neck Sags Under Head Weight**
- **Solution:** Increase servo torque settings, check mechanical design for flex, consider stronger servos

**Issue: Kickstand Doesn't Reach Floor**
- **Solution:** Adjust kickstand leg length, change deployment angle, lower Neck mounting position

---

## Dependencies

**Before this story:**
- Story 2.5: Enclosure and servo mounts printed ✅
- Story 2.6: Kickstand mechanism assembled ✅
- Story 2.8: ROS2 driver node created ✅
- Torso module available for mounting

**After this story:**
- Epic 3: Torso Module Build
- Epic 5: End-to-End Demo

---

## Notes

- **Weight Balance:** Neck servos must support ~800g head. If servos struggle, reduce head weight or upgrade servos.
- **Cable Routing:** Leave slack for neck movement. Cables too tight will restrict motion.
- **Kickstand Critical:** Enables stationary mode for demos, charging, and maintenance without requiring balancing.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
