# Story 4.8: Configure ODrive for Hoverboard Motors

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** to configure ODrive for reliable control of hoverboard motors with hall sensors,
**so that** the motors can be commanded accurately for balancing and movement.

---

## Acceptance Criteria

1. ✅ ODrive S1 connected to both hoverboard motors
2. ✅ Motor parameters configured (pole pairs, resistance, inductance)
3. ✅ Hall sensor encoder configured and calibrated
4. ✅ Both motors calibrate successfully without errors
5. ✅ Closed-loop velocity control working smoothly
6. ✅ Configuration saved to ODrive non-volatile memory
7. ✅ Velocity commands respond accurately (±5% of setpoint)
8. ✅ Emergency stop tested (motors stop within 100ms)

---

## Implementation Steps

### 1. Connect ODrive to Motors and Power

```bash
# Wiring (from Story 4.2):
#   M0 (left motor): Phase A/B/C + Hall VCC/GND/A/B/C
#   M1 (right motor): Phase A/B/C + Hall VCC/GND/A/B/C
#   Power: 36V battery via XT60, 15A fuse

# Connect ODrive USB to computer
odrivetool
```

### 2. Configure Motor Parameters

```python
# Motor 0 (left)
odrv0.axis0.motor.config.pole_pairs = 15
odrv0.axis0.motor.config.calibration_current = 10.0
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4.0
odrv0.axis0.motor.config.requested_current_range = 25.0
odrv0.axis0.motor.config.current_lim = 20.0
odrv0.axis0.motor.config.torque_constant = 8.27 / 150  # 8.27V/krpm

# Motor 1 (right) - same config
odrv0.axis1.motor.config.pole_pairs = 15
odrv0.axis1.motor.config.calibration_current = 10.0
odrv0.axis1.motor.config.resistance_calib_max_voltage = 4.0
odrv0.axis1.motor.config.requested_current_range = 25.0
odrv0.axis1.motor.config.current_lim = 20.0
odrv0.axis1.motor.config.torque_constant = 8.27 / 150
```

### 3. Configure Hall Sensor Encoders

```python
# Motor 0 encoder
odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 90  # 15 pole pairs × 6
odrv0.axis0.encoder.config.calib_scan_distance = 150
odrv0.axis0.encoder.config.bandwidth = 100

# Motor 1 encoder
odrv0.axis1.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis1.encoder.config.cpr = 90
odrv0.axis1.encoder.config.calib_scan_distance = 150
odrv0.axis1.encoder.config.bandwidth = 100
```

### 4. Configure Controllers

```python
# Velocity control mode for both axes
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.vel_limit = 20.0  # rev/s
odrv0.axis0.controller.config.vel_gain = 0.15
odrv0.axis0.controller.config.vel_integrator_gain = 0.3

odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis1.controller.config.vel_limit = 20.0
odrv0.axis1.controller.config.vel_gain = 0.15
odrv0.axis1.controller.config.vel_integrator_gain = 0.3

# Save configuration
odrv0.save_configuration()
odrv0.reboot()
```

### 5. Run Motor Calibration

```python
# Reconnect after reboot
odrivetool

# Calibrate motor 0
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
# Wait ~5 seconds, motor will beep/twitch

# Check for errors
dump_errors(odrv0)
# Expected: no errors

# Check calibration success
odrv0.axis0.motor.is_calibrated
# Expected: True

# Repeat for motor 1
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
dump_errors(odrv0)
odrv0.axis1.motor.is_calibrated
```

### 6. Run Encoder Calibration

```python
# Calibrate hall sensors motor 0
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
# Wait ~3 seconds, motor spins slowly

odrv0.axis0.encoder.is_ready
# Expected: True

# Repeat for motor 1
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
odrv0.axis1.encoder.is_ready

# Save calibration
odrv0.save_configuration()
```

### 7. Test Closed-Loop Control

```python
# Enter closed-loop control
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Test motor 0 velocity
odrv0.axis0.controller.input_vel = 2.0  # 2 rev/s
# Motor should spin smoothly

# Read actual velocity
odrv0.axis0.encoder.vel_estimate
# Should be close to 2.0 (±0.1)

# Stop
odrv0.axis0.controller.input_vel = 0

# Test motor 1
odrv0.axis1.controller.input_vel = -2.0  # Opposite direction
odrv0.axis1.controller.input_vel = 0

# Exit closed loop
odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis1.requested_state = AXIS_STATE_IDLE
```

### 8. Enable Startup Sequence

```python
# Auto-enable closed loop on power-up
odrv0.axis0.config.startup_encoder_index_search = False
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_motor_calibration = False
odrv0.axis0.config.startup_closed_loop_control = True

odrv0.axis1.config.startup_encoder_index_search = False
odrv0.axis1.config.startup_encoder_offset_calibration = False
odrv0.axis1.config.startup_motor_calibration = False
odrv0.axis1.config.startup_closed_loop_control = True

odrv0.save_configuration()
odrv0.reboot()
```

### 9. Test Emergency Stop

```bash
# Enter closed loop
# Command motor spin
odrv0.axis0.controller.input_vel = 5.0

# Press physical E-stop button
# Expected: Power cuts, motors stop within 100ms

# Release E-stop
# ODrive reboots, enters closed loop automatically (if startup enabled)
```

### 10. Document Configuration

**Create `modules/base/odrive_config.md`:**

```markdown
# ODrive Configuration for OLAF

## Hardware
- ODrive S1
- Motors: Hoverboard BLDC (15 pole pairs)
- Encoders: Hall sensors (90 CPR)

## Motor Parameters
- Pole pairs: 15
- Current limit: 20A
- Velocity limit: 20 rev/s (~200 RPM)

## Control Gains
- Velocity gain: 0.15
- Velocity integrator gain: 0.3

## Startup Behavior
- Auto-enters closed loop control on power-up
- No recalibration needed (saved to memory)

## Commands
- Velocity control: Set `input_vel` (rev/s)
- Example: `odrv0.axis0.controller.input_vel = 5.0`

## Troubleshooting
- Check errors: `dump_errors(odrv0)`
- Recalibrate: `requested_state = AXIS_STATE_MOTOR_CALIBRATION`
```

---

## Testing & Validation

**Test 1: Motor Calibration Success**
```bash
odrv0.axis0.motor.is_calibrated == True
odrv0.axis1.motor.is_calibrated == True
```

**Test 2: Velocity Tracking**
```bash
# Command 5 rev/s
# Actual velocity within ±0.25 rev/s (±5%)
```

**Test 3: Startup Behavior**
```bash
# Power cycle ODrive
# Enters closed loop automatically
```

---

## Troubleshooting

**Issue 1: Calibration Fails (ERROR_MOTOR_FAILED)**
- **Solution:** Reduce calibration current to 5A, check phase wiring, verify battery voltage >30V

**Issue 2: Encoder Not Ready**
- **Solution:** Check hall sensor wiring, verify 5V power to halls, swap hall A/B/C wires

**Issue 3: Motor Vibrates/Cogs**
- **Solution:** Increase velocity gain, check motor resistance, reduce current limit

**Issue 4: Velocity Tracking Poor**
- **Solution:** Tune PID gains (increase vel_gain), check encoder CPR, verify no mechanical binding

---

## Dependencies

**Before this story:**
- Story 4.7: Assemble Base Platform ✅

**After this story:**
- Story 4.9: Develop Base ESP32 Firmware with 200Hz Balancing PID

---

## References

- [ODrive Hoverboard Guide](https://docs.odriverobotics.com/v/latest/guides/hoverboard.html)
- [ODrive Configuration Reference](https://docs.odriverobotics.com/v/latest/manual/configuration.html)

---

## Notes

- **Hall Sensors Critical:** Enable smooth low-speed control required for balancing
- **Calibration:** Only needed once, saved to ODrive memory
- **Velocity Units:** ODrive uses rev/s. For wheel: 1 rev/s ≈ 0.6 m/s linear speed (6.5" wheel)
- **Current Limits:** 20A per motor safe for hoverboard motors, don't exceed 25A continuously

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
