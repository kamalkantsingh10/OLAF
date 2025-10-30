# Data Models

## I2C Register Map (Standard - All Modules)

```cpp
// shared/i2c_registers.h
// Common registers (0x00-0x0F)

#define REG_MODULE_ID           0x00  // Read-only: Module identification
#define REG_FIRMWARE_VERSION    0x01  // Read-only: Firmware version
#define REG_STATUS              0x02  // Read-only: Module status byte
#define REG_ERROR_CODE          0x03  // Read-only: Last error code
#define REG_COMMAND             0x04  // Write: Command trigger
#define REG_COMMAND_ARG1        0x05  // Write: Command argument 1-4
#define REG_COMMAND_ARG2        0x06
#define REG_COMMAND_ARG3        0x07
#define REG_COMMAND_ARG4        0x08

// Status byte bit flags
#define STATUS_READY            0x01
#define STATUS_BUSY             0x02
#define STATUS_ERROR            0x04
#define STATUS_OTA_MODE         0x08
#define STATUS_CALIBRATING      0x10
```

## Head Module Register Map (I2C 0x08)

```cpp
// Expression Control (0x10-0x1F)
#define REG_EXPRESSION_TYPE     0x10  // Emotion type (0x00-0x06)
#define REG_EXPRESSION_INTENSITY 0x11 // Intensity level (1-5)
#define REG_BLINK_TRIGGER       0x12

// Sensor Data (0x20-0x2F)
#define REG_PRESENCE_DETECTED   0x20
#define REG_PRESENCE_DISTANCE   0x21
#define REG_PRESENCE_ANGLE      0x22

// Expression types
#define EXPR_NEUTRAL  0x00
#define EXPR_HAPPY    0x01
#define EXPR_CURIOUS  0x02
#define EXPR_THINKING 0x03
#define EXPR_CONFUSED 0x04
#define EXPR_SAD      0x05
#define EXPR_EXCITED  0x06
```

## Base Module Register Map (I2C 0x0C)

```cpp
// Movement Control (0x10-0x1F)
#define REG_TARGET_VELOCITY     0x10  // float32 (4 bytes)
#define REG_TARGET_TURN_RATE    0x14  // float32 (4 bytes)
#define REG_MOVEMENT_MODE       0x18

// Balancing Status (0x20-0x2F)
#define REG_BALANCE_STATE       0x20
#define REG_PITCH_ANGLE         0x21  // float32
#define REG_PITCH_RATE          0x25  // float32
#define REG_KICKSTAND_STATE     0x29

// Odometry (0x30-0x4F)
#define REG_ODOMETRY_X          0x30  // float32
#define REG_ODOMETRY_Y          0x34  // float32
#define REG_ODOMETRY_THETA      0x38  // float32

// States
#define STATE_RELAXED         0x00
#define STATE_TRANSITIONING   0x01
#define STATE_BALANCING       0x02
#define STATE_EMERGENCY_STOP  0x03
```

## Body Module Register Map (I2C 0x0A)

```cpp
// Heart Display Control (0x10-0x1F)
#define REG_HEART_EMOTION_TYPE     0x10  // Emotion type (0x00-0x06)
#define REG_HEART_INTENSITY        0x11  // Intensity level (1-5)
#define REG_HEART_RATE_BPM         0x12  // Heart rate override (BPM, 0=auto)
#define REG_HEART_COLOR_R          0x13  // Heart color RGB (0-255 each)
#define REG_HEART_COLOR_G          0x14
#define REG_HEART_COLOR_B          0x15

// Projector Control (0x20-0x2F)
#define REG_PROJECTOR_POWER        0x20  // 0=off, 1=on
#define REG_PROJECTOR_STATUS       0x21  // Power state feedback

// LED Control (0x30-0x3F)
#define REG_LED_MODE               0x30  // LED pattern mode
#define REG_LED_COLOR_R            0x31  // LED color RGB
#define REG_LED_COLOR_G            0x32
#define REG_LED_COLOR_B            0x33
#define REG_LED_BRIGHTNESS         0x34  // 0-255

// Emotion types (same as Head Module for consistency)
#define EMOTION_NEUTRAL  0x00
#define EMOTION_HAPPY    0x01
#define EMOTION_CURIOUS  0x02
#define EMOTION_THINKING 0x03
#define EMOTION_CONFUSED 0x04
#define EMOTION_SAD      0x05
#define EMOTION_EXCITED  0x06
```

## ROS2 Custom Messages

```python
# interfaces/msg/Expression.msg
string emotion              # 'happy', 'curious', etc.
uint8 intensity             # 1-5
uint8 duration_ms
bool synchronize

# interfaces/msg/BalanceStatus.msg
Header header
string state
float32 pitch_deg
float32 pitch_rate_dps
bool kickstand_deployed
float32 motor_current_left_a
float32 motor_current_right_a

# interfaces/msg/ModuleStatus.msg
Header header
string module_name
uint8 i2c_address
bool online
uint8 firmware_version_major
uint8 firmware_version_minor
string error_message
uint32 uptime_seconds
```

---
