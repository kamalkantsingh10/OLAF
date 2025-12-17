# Story 4.10: Fine-Tune Self-Balancing PID Parameters

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** Medium
**Estimated Effort:** 3-4 hours

---

## User Story

**As a** builder,
**I want** to systematically tune PID gains for optimal balancing performance,
**so that** the robot balances stably, recovers quickly from disturbances, and doesn't oscillate.

---

## Acceptance Criteria

1. ✅ Ziegler-Nichols or manual tuning method applied
2. ✅ P, I, D gains optimized for minimal oscillation
3. ✅ Robot balances for >60 seconds without falling
4. ✅ Recovers from 10° push within 2 seconds
5. ✅ Settling time <1 second after disturbance
6. ✅ No steady-state error (stays within ±2° of upright)
7. ✅ Final gains documented in firmware and docs
8. ✅ Performance validated with multiple test runs

---

## Implementation Steps

### 1. Start with Initial Conservative Gains

```cpp
// In firmware:
PIDController balancePID(10.0, 0.0, 0.0, DT);  // Low P, zero I and D
```

### 2. Tune Proportional Gain (P)

```bash
# Step 1: Increase P until robot starts to oscillate
# P=10: Robot falls (too low)
# P=20: Robot stays up but drifts
# P=40: Robot balances but has steady-state error
# P=60: Robot oscillates slightly
# P=80: Robot oscillates strongly (critical gain Kc)

# Step 2: Set P to 60-70% of critical gain
# P_final ≈ 0.6 × 80 = 48

# Test: Robot should balance without oscillation
```

### 3. Tune Integral Gain (I)

```bash
# Purpose: Eliminate steady-state error (robot leaning slightly)

# Start with I = P / 10 = 4.8 ≈ 5
# Increase gradually if steady-state error remains
# Stop if robot becomes sluggish or oscillates

# Final I ≈ 5-10
```

### 4. Tune Derivative Gain (D)

```bash
# Purpose: Reduce overshoot and dampen oscillations

# Start with D = P / 20 = 2.4 ≈ 2
# Increase D to reduce overshoot
# Too much D causes jittery behavior

# Final D ≈ 2-5
```

### 5. Test Disturbance Rejection

```bash
# Test 1: Push robot forward (10° tilt)
#   - Should recover within 2 seconds
#   - No excessive overshoot

# Test 2: Push robot backward
#   - Same recovery performance

# Test 3: Add weight on top (500g)
#   - Robot should still balance
#   - May need slight I gain increase
```

### 6. Optimize for Different Scenarios

```bash
# Scenario A: Empty robot (lightweight)
#   Best gains: P=50, I=5, D=2

# Scenario B: Full robot (with Torso + Head, ~15kg total)
#   Best gains: P=60, I=8, D=3
#   Higher gains needed for more inertia

# Use I2C commands to switch gain sets dynamically
```

### 7. Fine-Tune Control Loop Timing

```bash
# If robot still oscillates:
#   - Verify control loop exactly 200Hz
#   - Check IMU filter bandwidth (21Hz recommended)
#   - Reduce serial debug output (slows loop)

# Measure actual loop frequency:
static unsigned long lastPrint = 0;
static int loopCount = 0;

loopCount++;
if (millis() - lastPrint > 1000) {
    Serial.print("Loop frequency: ");
    Serial.println(loopCount);
    loopCount = 0;
    lastPrint = millis();
}
// Expected output: 200 Hz
```

### 8. Test Long-Duration Stability

```bash
# Test: Balance for 5 minutes continuously
# Metrics:
#   - Average pitch angle: ±1°
#   - Max deviation: <5°
#   - No drift over time

# If drift occurs: Increase I gain slightly
```

### 9. Document Final Gains

**Update `modules/base/firmware/src/main.cpp`:**

```cpp
// Optimized PID gains (tuned 2025-12-17)
// Robot mass: ~15kg with full assembly
// Wheel diameter: 6.5 inches
PIDController balancePID(55.0, 7.0, 2.5, DT);
```

**Create `modules/base/pid_tuning.md`:**

```markdown
# PID Tuning Results

## Final Gains
- P (Proportional): 55.0
- I (Integral): 7.0
- D (Derivative): 2.5

## Performance Metrics
- Balance duration: >5 minutes
- Recovery time: 1.5 seconds (from 10° push)
- Steady-state error: ±1.5°
- Oscillation: None (critically damped)

## Tuning Process
1. Started with P=10, increased to P=80 (oscillation)
2. Set P=55 (70% of critical)
3. Added I=7 to eliminate drift
4. Added D=2.5 to reduce overshoot

## Robot Configuration
- Total mass: 15kg
- Battery: 36V 4.4Ah
- Wheels: 6.5" hoverboard
- Center of gravity: 350mm above ground

## Notes
- Gains may need adjustment if payload changes significantly
- IMU must be mounted flat and centered
- Control loop must maintain 200Hz
```

### 10. Validate with Multiple Test Runs

```bash
# Run 5 test sessions:
#   Session 1: 60 second balance test
#   Session 2: Disturbance rejection (10 pushes)
#   Session 3: 5 minute endurance test
#   Session 4: Forward/backward movement
#   Session 5: Full speed test (max velocity)

# Log all results
# Success criteria: <5% failure rate
```

---

## Testing & Validation

**Test 1: Balance Duration**
```bash
# Robot balances for >60 seconds without intervention
```

**Test 2: Disturbance Recovery**
```bash
# Recovers from 10° push in <2 seconds
# No overshoot >5°
```

**Test 3: Steady-State Error**
```bash
# Average pitch within ±2° over 60 seconds
```

**Test 4: No Oscillation**
```bash
# Pitch angle smooth, no high-frequency oscillations
```

---

## Troubleshooting

**Issue 1: Robot Oscillates at 1-2 Hz**
- **Solution:** Reduce P gain by 20%, increase D gain

**Issue 2: Robot Drifts Slowly to One Side**
- **Solution:** Increase I gain, check IMU calibration, verify motor directions matched

**Issue 3: Robot Sluggish, Doesn't Respond Quickly**
- **Solution:** Increase P gain, reduce D gain

**Issue 4: Robot Jerky/Jittery**
- **Solution:** Reduce D gain, add low-pass filter to gyro data

---

## Dependencies

**Before this story:**
- Story 4.9: Develop Base ESP32 Firmware ✅

**After this story:**
- Story 4.11: Create Base ROS2 Driver Node

---

## References

- [PID Tuning Guide](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
- [Balancing Robot PID Tuning](https://www.researchgate.net/publication/320307286_PID_Controller_Design_for_Two-Wheeled_Self-Balancing_Robot)

---

## Notes

- **Safety:** Always have E-stop ready during tuning
- **Patience:** Good tuning takes time, expect multiple iterations
- **Environment:** Smooth floor critical, test on same surface consistently
- **Battery:** Voltage affects motor response, tune with fully charged battery
- **Future:** Implement gain scheduling (different gains for different speeds), adaptive control

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
