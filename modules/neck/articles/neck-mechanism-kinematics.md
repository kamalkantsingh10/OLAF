# Neck Mechanism Kinematics

**Author:** Design Documentation
**Date:** 2025-12-25
**Status:** Design Verified

---

## Overview

The Neck module provides expressive 3-DOF head movement using a parallel linkage mechanism with 3× STS3215 servos. The design enables both traditional pan/tilt motions and WALL-E-style expressive head tilts for natural interaction.

---

## Mechanical Architecture

### Parallel Linkage Design

**Configuration:**
- **1× Base Servo (Yaw):** Rotates entire upper assembly for horizontal head panning
- **2× Parallel Tilt Servos (Pitch/Roll):** Drive linkage rods to control head pitch and roll independently

**Physical Layout:**
```
                    [Head Platform]
                         /|\
                        / | \
                       /  |  \
     [Linkage Rod]----/   |   \----[Linkage Rod]
                    /     |     \
         [Tilt Servo L]   |   [Tilt Servo R]
                    \     |     /
                     \    |    /
                      \   |   /
                   [Base Platform]
                         |
                   [Yaw Servo]
                         |
                   [Torso Mount]
```

---

## Degrees of Freedom

### DOF 1: Yaw (Pan) - Horizontal Rotation

**Servo:** ID 1 (Base servo, bottom position)
**Range:** ±75° from center (configurable in `config.yaml`)
**Motion:** Entire head rotates left/right
**Control:** Single servo angle commands

```
Yaw Angle = Base Servo Position
  -75° = Look far left
    0° = Look forward
  +75° = Look far right
```

### DOF 2: Pitch (Nod) - Vertical Tilt

**Servos:** ID 2 + ID 3 (Both tilt servos moving **synchronized**)
**Range:** ±45° from center (configurable)
**Motion:** Head nods up/down (looking up/down)
**Control:** Both servos commanded to **same angle**

```
Pitch Angle = Left Tilt Servo = Right Tilt Servo
  +45° = Look up (both servos rotate together)
    0° = Level
  -45° = Look down (both servos rotate together)
```

**Example Commands:**
```cpp
// Look up 30°
servos.WritePos(SERVO_TILT_LEFT, centerPos + pitchOffset, speed);
servos.WritePos(SERVO_TILT_RIGHT, centerPos + pitchOffset, speed);

// Look down 20°
servos.WritePos(SERVO_TILT_LEFT, centerPos - pitchOffset, speed);
servos.WritePos(SERVO_TILT_RIGHT, centerPos - pitchOffset, speed);
```

### DOF 3: Roll (Side Tilt) - Expressive Head Tilt

**Servos:** ID 2 + ID 3 (Tilt servos moving **differentially**)
**Range:** ±30° (estimated, depends on linkage geometry)
**Motion:** Head tilts to side (WALL-E/Pixar-style curiosity, focusing on faces)
**Control:** Servos commanded to **different angles**

```
Roll Angle = (Left Tilt Servo - Right Tilt Servo) / 2
  Left +20°, Right -20° → Roll left ~30°
  Left  -5°, Right  +5° → Roll right ~7.5°
  Left   0°, Right   0° → No roll (level)
```

**Example Commands:**
```cpp
// Tilt head left (curious look)
servos.WritePos(SERVO_TILT_LEFT, centerPos + rollOffset, speed);
servos.WritePos(SERVO_TILT_RIGHT, centerPos - rollOffset, speed);

// Tilt head right (confusion)
servos.WritePos(SERVO_TILT_LEFT, centerPos - rollOffset, speed);
servos.WritePos(SERVO_TILT_RIGHT, centerPos + rollOffset, speed);
```

---

## Control Modes

### Mode 1: Synchronized (Pure Pitch)

**Use Case:** Traditional nodding (agreement, looking up/down)

```cpp
float pitch = 30.0;  // degrees

int leftAngle = pitchToServoPos(pitch);
int rightAngle = pitchToServoPos(pitch);

servos.WritePos(SERVO_TILT_LEFT, leftAngle, 800);
servos.WritePos(SERVO_TILT_RIGHT, rightAngle, 800);

// Result: Pure vertical tilt, no side roll
```

### Mode 2: Differential (Pitch + Roll)

**Use Case:** Expressive tilts (curiosity, focusing on faces, confusion)

```cpp
float pitch = 10.0;  // degrees (slight down)
float roll = 25.0;   // degrees (tilt left)

int leftAngle = pitchToServoPos(pitch + roll);
int rightAngle = pitchToServoPos(pitch - roll);

servos.WritePos(SERVO_TILT_LEFT, leftAngle, 800);
servos.WritePos(SERVO_TILT_RIGHT, rightAngle, 800);

// Result: Looking slightly down with head tilted left (WALL-E style)
```

### Mode 3: Combined (Yaw + Pitch + Roll)

**Use Case:** Natural gaze tracking, face-to-face interaction

```cpp
float yaw = 45.0;    // Look right
float pitch = -15.0; // Slightly down
float roll = 10.0;   // Slight curious tilt

servos.WritePos(SERVO_YAW, yawToServoPos(yaw), 1000);
servos.WritePos(SERVO_TILT_LEFT, pitchToServoPos(pitch + roll), 800);
servos.WritePos(SERVO_TILT_RIGHT, pitchToServoPos(pitch - roll), 800);

// Result: Looking at person to the right, slightly below, with curious tilt
```

---

## Servo Specifications

| Servo ID | Function | Model | Position | Motion Range | Notes |
|----------|----------|-------|----------|--------------|-------|
| **1** | Yaw (Pan) | STS3215 | Base (bottom) | ±75° | Rotates entire upper assembly |
| **2** | Tilt Left | STS3215 | Left linkage | ±45° (sync), ±30° (differential) | Parallel linkage driver |
| **3** | Tilt Right | STS3215 | Right linkage | ±45° (sync), ±30° (differential) | Parallel linkage driver |
| **4** | Kickstand | STS3215 | Neck base | 0°-90° | Shared bus, independent function |

**Servo Details:**
- **Model:** Feetech STS3215
- **Torque:** 30 kg·cm @ 12V
- **Speed:** 0.09 sec/60° @ 12V
- **Resolution:** 4096 positions (0-4095, 12-bit)
- **Center Position:** 2048
- **Communication:** Serial bus (daisy-chain), half-duplex UART
- **Voltage:** 6-12V (optimal: 12V for full torque)

---

## Configuration

### config.yaml (to be created)

```yaml
# modules/neck/config.yaml
neck_kinematics:
  # Servo IDs
  servo_yaw: 1
  servo_tilt_left: 2
  servo_tilt_right: 3
  servo_kickstand: 4

  # Motion limits (degrees from center)
  yaw_range: 75        # ±75° horizontal rotation
  pitch_range: 45      # ±45° vertical tilt (synchronized mode)
  roll_range: 30       # ±30° side tilt (differential mode, estimated)

  # Servo center positions (factory default: 2048)
  center_yaw: 2048
  center_tilt_left: 2048
  center_tilt_right: 2048

  # Safety limits (prevent mechanical binding)
  min_servo_pos: 512    # Absolute minimum (reserve margin)
  max_servo_pos: 3584   # Absolute maximum (reserve margin)

  # Motion speeds (0-3000, units: steps per second)
  speed_yaw: 1000       # Moderate panning speed
  speed_pitch: 800      # Smooth nodding
  speed_roll: 600       # Gentle expressive tilts

  # Kickstand configuration
  kickstand_retracted: 512   # Fully up (stored position)
  kickstand_deployed: 3500   # Fully down (supporting robot)
  kickstand_speed: 1000      # Deployment/retraction speed
```

---

## Kinematic Calculations

### Angle to Servo Position Conversion

**Formula:**
```
ServoPosition = CENTER_POS + (Angle_degrees / MAX_ANGLE_degrees) * SERVO_RANGE
```

**Where:**
- `CENTER_POS = 2048` (12-bit center)
- `SERVO_RANGE = 2048` (half of 4096 total positions)
- `MAX_ANGLE_degrees` = configured range (75° for yaw, 45° for pitch)

**Example (Yaw):**
```cpp
// Convert yaw angle (-75° to +75°) to servo position
int yawToServoPos(float yaw_deg) {
    const int CENTER = 2048;
    const float MAX_YAW = 75.0;
    const int RANGE = 2048;

    float normalized = yaw_deg / MAX_YAW;  // -1.0 to +1.0
    int offset = (int)(normalized * RANGE);
    return CENTER + offset;
}

// Example: yaw = +45° → servoPos = 2048 + (45/75)*2048 = 3276
```

**Example (Pitch with Roll):**
```cpp
// Convert pitch+roll to left/right servo positions
void pitchRollToServoPos(float pitch_deg, float roll_deg,
                         int& left_pos, int& right_pos) {
    const int CENTER = 2048;
    const float MAX_PITCH = 45.0;
    const int RANGE = 2048;

    // Left servo: pitch + roll
    float left_angle = pitch_deg + roll_deg;
    left_pos = CENTER + (int)((left_angle / MAX_PITCH) * RANGE);

    // Right servo: pitch - roll
    float right_angle = pitch_deg - roll_deg;
    right_pos = CENTER + (int)((right_angle / MAX_PITCH) * RANGE);
}

// Example: pitch = +10°, roll = +20° (tilt left)
// → left = 2048 + ((10+20)/45)*2048 = 3413
// → right = 2048 + ((10-20)/45)*2048 = 1592
```

---

## Mechanical Advantages

### Parallel Linkage Benefits

1. **2× Torque for Pitch:**
   - Two servos share load when tilting head up/down
   - Each servo provides 30 kg·cm → Combined 60 kg·cm for pitch
   - Essential for heavy head module (OLED eyes, camera, sensors)

2. **Stable Platform:**
   - Parallel geometry prevents unwanted twist/rotation
   - Head platform remains level during pure pitch motion
   - Reduces vibration during movement

3. **Expressive Motion:**
   - Differential control enables natural head tilts
   - Mimics human/animal curiosity behaviors
   - Enhances personality and emotional expressiveness

4. **Mechanical Simplicity:**
   - No complex gearboxes or linkages
   - Direct drive from servos via simple rods
   - Easy to calibrate and maintain

### Design Trade-offs

**Advantages:**
- High torque capacity
- Expressive 3-DOF motion
- Reliable mechanical design
- Standard servos (no custom parts)

**Disadvantages:**
- Requires synchronized control algorithm
- More complex kinematics than single-servo tilt
- Differential mode needs careful calibration
- Slight coupling between pitch and roll (linkage geometry dependent)

---

## Expressive Motion Examples

### 1. Curiosity (WALL-E Style)

```cpp
// Tilt head slightly, look at object of interest
yaw = 30;     // Turn toward object
pitch = -10;  // Look slightly down
roll = 15;    // Curious head tilt

// Result: "What's that?" expression
```

### 2. Agreement (Nodding)

```cpp
// Synchronized pitch, no roll
for (int i = 0; i < 3; i++) {
    pitch = -20;  // Nod down
    delay(300);
    pitch = +10;  // Nod up
    delay(300);
}
roll = 0;  // No side tilt

// Result: "Yes!" nodding motion
```

### 3. Confusion

```cpp
// Alternate side tilts with slight yaw
for (int i = 0; i < 2; i++) {
    yaw = -15; roll = -20;  // Tilt left, look left
    delay(400);
    yaw = +15; roll = +20;  // Tilt right, look right
    delay(400);
}

// Result: "Huh?" confused head shaking
```

### 4. Attention (Looking at Person)

```cpp
// Track face detected by camera
yaw = face_x_offset;    // Pan to center face horizontally
pitch = face_y_offset;  // Tilt to center face vertically
roll = face_x_offset * 0.3;  // Slight tilt toward face (engaged look)

// Result: Natural face-to-face eye contact with personality
```

---

## Calibration Procedure

### Step 1: Find Mechanical Limits

1. Power servos with 12V supply
2. Manually move each servo to physical extremes
3. Note servo positions before mechanical binding/strain
4. Record safe limits with 5° margin

**Example:**
```
Yaw Servo:
  Physical left limit: 820
  Physical right limit: 3275
  Safe limits: 900 to 3196 (±70° practical range)

Tilt Servos (synchronized):
  Physical up limit: 3150
  Physical down limit: 945
  Safe limits: 1048 to 3048 (±45° practical range)
```

### Step 2: Set Center Positions

1. Command all servos to center (2048)
2. Verify head is level and facing forward
3. If misaligned, adjust center positions in config
4. Re-test until centered

### Step 3: Test Motion Ranges

1. Command full range sweeps for each DOF
2. Verify smooth motion with no binding
3. Check linkage rods are parallel during pitch
4. Adjust config limits if needed

### Step 4: Validate Expressive Motions

1. Test synchronized pitch (nodding)
2. Test differential roll (side tilts)
3. Test combined motions (yaw + pitch + roll)
4. Verify WALL-E-style expressions work naturally

---

## Future Enhancements

### Inverse Kinematics (Story 2.7+)

**Goal:** Given target head orientation (yaw, pitch, roll), calculate servo angles automatically

```cpp
// High-level API
setHeadOrientation(yaw_deg, pitch_deg, roll_deg);

// Internal calculation:
// servo1_pos = yawToServoPos(yaw);
// (servo2_pos, servo3_pos) = pitchRollToServoPos(pitch, roll);
```

### Expression Presets (Story 2.8+)

**Goal:** Pre-programmed expressive motions for personality coordination

```cpp
enum Expression {
    NEUTRAL,
    CURIOUS,      // Slight tilt + forward lean
    CONFUSED,     // Side-to-side tilts
    EXCITED,      // Rapid small movements
    THINKING,     // Upward gaze + slight tilt
    GREETING      // Nod sequence
};

expressEmotion(CURIOUS, INTENSITY_MEDIUM);
```

### Face Tracking Integration (Story 2.9+)

**Goal:** Real-time head orientation to track detected faces

```cpp
// Camera detects face at (x, y) in frame
// Convert to yaw/pitch angles
// Add subtle roll for engagement
// Smooth servo motion (no jerky movements)
```

---

## References

- [Feetech STS3215 Datasheet](http://www.feetechrc.com/en_product.html)
- [SCServo Library](https://github.com/ftservo/SCServo)
- [WALL-E Animation Reference](https://www.pixar.com/wall-e) (Expressive head tilts)
- [Parallel Linkage Kinematics](https://en.wikipedia.org/wiki/Parallel_manipulator)

---

## Change Log

| Date | Author | Changes |
|------|--------|---------|
| 2025-12-25 | Design Team | Initial kinematics documentation |

---

**Status:** ✅ Design verified with mechanical mockup
**Next Steps:** Create `config.yaml`, implement servo control firmware
