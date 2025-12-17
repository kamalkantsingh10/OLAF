# Story 2.1: Breadboard Neck Components and Test Connectivity

**Epic:** Epic 2 - Neck Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** all Neck components breadboarded and individually tested,
**so that** I can verify functionality before committing to PCB design.

---

## Acceptance Criteria

1. ✅ ESP32 development board connected to breadboard with power supply
2. ✅ 4× Feetech STS3215 servos wired on single UART bus (daisy-chained): 3 for pan/tilt/roll + 1 for kickstand
3. ✅ 2× presence sensors (mmWave or PIR) wired and providing detection signals
4. ✅ Servo daisy-chain communication tested: all 4 servos respond to unique IDs
5. ✅ Presence sensors tested: can detect human presence in their coverage zones
6. ✅ Component-level test script confirms: all servos move to commanded positions, kickstand deploys/retracts, presence sensors report detection state
7. ✅ Servo ID configuration documented (e.g., Pan=ID1, Tilt=ID2, Roll=ID3, Kickstand=ID4)

---

## Implementation Steps

### 1. Set Up ESP32 Development Board

```bash
cd ~/olaf/modules/neck
mkdir -p firmware/breadboard_test
cd firmware/breadboard_test
pio init --board esp32dev
```

**Power:**
- ESP32 5V pin → Breadboard 5V rail
- ESP32 GND pin → Breadboard GND rail
- USB connection to laptop for programming

### 2. Wire Feetech STS3215 Servo Daisy Chain

**Servo Configuration:**
- All 4 servos on single UART bus for simpler wiring
- Servos share data line, each has unique ID
- Power: 6-8.4V (nominal 7.4V for STS3215)

**Wiring Diagram:**
```
ESP32 GPIO32 (UART TX/RX) → Logic Level Converter (3.3V ↔ 5V TTL)
                          ↓
                    Servo ID=1 (Pan) DATA IN → DATA OUT
                          ↓
                    Servo ID=2 (Tilt) DATA IN → DATA OUT
                          ↓
                    Servo ID=3 (Roll) DATA IN → DATA OUT
                          ↓
                    Servo ID=4 (Kickstand) DATA IN → DATA OUT

Power: 7.4V supply (2S LiPo or regulated) → All servos VCC
GND: Common ground (ESP32, servos, power supply)
```

**Power Supply:**
- Voltage: 6-8.4V (7.4V nominal for STS3215)
- Current: 5A minimum (4 servos × 1A peak + margin)
- Use 2S LiPo battery or bench supply with current limiting

**Logic Level Converter:**
- ESP32 uses 3.3V logic, Feetech servos use 5V TTL
- Bidirectional level shifter (e.g., BSS138-based)
- Connections: ESP32 3.3V side ↔ Servo 5V side

### 3. Configure Servo IDs

**Using Feetech Debug Tool (Optional but Recommended):**

```bash
# Download FD software from Feetech website
# Connect single servo to USB-to-TTL adapter
# Use FD software to:
# 1. Detect servo
# 2. Set ID (1, 2, 3, or 4)
# 3. Verify baud rate (1000000 or 115200)
# 4. Test movement
# Repeat for all 4 servos
```

**Without Debug Tool (Firmware-Based):**

```cpp
// Set servo IDs programmatically (one at a time)
// Connect only ONE servo at a time to avoid conflicts
#include <SCServo.h>

SMS_STS servos;
HardwareSerial ServoSerial(1); // GPIO32

void setup() {
    Serial.begin(115200);
    ServoSerial.begin(1000000, SERIAL_8N1, 32, 32); // Half-duplex
    servos.pSerial = &ServoSerial;

    // Broadcast mode: change ID of connected servo
    int newID = 1; // Change this for each servo (1, 2, 3, 4)
    servos.unLockEprom(254); // Unlock all servos
    servos.writeByte(254, SMS_STS_ID, newID);
    servos.LockEprom(newID);

    Serial.println("Servo ID set to: " + String(newID));
    Serial.println("Power cycle servo and test");
}

void loop() {
    // Test: Move servo to center
    servos.WritePos(newID, 2048, 1000);
    delay(2000);
}
```

### 4. Test Servo Daisy Chain Communication

**Test Code:**

```cpp
#include <SCServo.h>

SMS_STS servos;
HardwareSerial ServoSerial(1);

#define SERVO_PAN 1
#define SERVO_TILT 2
#define SERVO_ROLL 3
#define SERVO_KICKSTAND 4

void setup() {
    Serial.begin(115200);
    ServoSerial.begin(1000000, SERIAL_8N1, 32, 32);
    servos.pSerial = &ServoSerial;

    Serial.println("Neck Servo Test - 4 Servos Daisy-Chained");

    // Test each servo individually
    testServo(SERVO_PAN, "Pan");
    testServo(SERVO_TILT, "Tilt");
    testServo(SERVO_ROLL, "Roll");
    testServo(SERVO_KICKSTAND, "Kickstand");
}

void testServo(int id, const char* name) {
    Serial.print("Testing ");
    Serial.print(name);
    Serial.print(" (ID ");
    Serial.print(id);
    Serial.println(")...");

    // Read current position
    int pos = servos.ReadPos(id);
    Serial.print("  Current position: ");
    Serial.println(pos);

    // Move to center
    servos.WritePos(id, 2048, 1000);
    delay(1500);

    // Move to extremes
    servos.WritePos(id, 3072, 1000); // +1024 from center
    delay(1500);
    servos.WritePos(id, 1024, 1000); // -1024 from center
    delay(1500);

    // Return to center
    servos.WritePos(id, 2048, 1000);
    delay(1500);

    Serial.println("  Test complete\n");
}

void loop() {
    // Coordinated movement demo
    static unsigned long lastMove = 0;
    static int step = 0;

    if (millis() - lastMove > 2000) {
        lastMove = millis();

        switch(step) {
            case 0: // Look left
                servos.WritePos(SERVO_PAN, 3072, 800);
                Serial.println("Look left");
                break;
            case 1: // Look right
                servos.WritePos(SERVO_PAN, 1024, 800);
                Serial.println("Look right");
                break;
            case 2: // Look up
                servos.WritePos(SERVO_TILT, 3072, 800);
                Serial.println("Look up");
                break;
            case 3: // Look down
                servos.WritePos(SERVO_TILT, 1024, 800);
                Serial.println("Look down");
                break;
            case 4: // Roll left
                servos.WritePos(SERVO_ROLL, 3072, 800);
                Serial.println("Roll left");
                break;
            case 5: // Roll right
                servos.WritePos(SERVO_ROLL, 1024, 800);
                Serial.println("Roll right");
                break;
            case 6: // Deploy kickstand
                servos.WritePos(SERVO_KICKSTAND, 3500, 1000);
                Serial.println("Kickstand down");
                break;
            case 7: // Retract kickstand
                servos.WritePos(SERVO_KICKSTAND, 512, 1000);
                Serial.println("Kickstand up");
                break;
        }

        step = (step + 1) % 8;
    }
}
```

**Upload and Test:**
```bash
pio run --target upload
pio device monitor

# Expected output:
# - Each servo responds when addressed by ID
# - Smooth movement through range
# - No jitter or communication errors
```

### 5. Wire and Test Presence Sensors

**Sensor Options:**

**Option A: PIR Sensors (Simple, Cheap)**
- HC-SR501 PIR modules (×2)
- Detection range: 3-7m
- Output: Digital HIGH when motion detected

**Option B: mmWave Sensors (Accurate, Through-Wall)**
- RCWL-0516 microwave radar modules (×2)
- Detection range: 5-9m
- Output: Digital HIGH when presence detected
- Can detect through enclosure material

**Wiring (Both Types Similar):**
```
Sensor 1 (Front):
  VCC → 5V
  GND → GND
  OUT → ESP32 GPIO25

Sensor 2 (Rear):
  VCC → 5V
  GND → GND
  OUT → ESP32 GPIO26
```

**Test Code:**

```cpp
#define SENSOR_FRONT 25
#define SENSOR_REAR 26

void setup() {
    Serial.begin(115200);
    pinMode(SENSOR_FRONT, INPUT);
    pinMode(SENSOR_REAR, INPUT);

    Serial.println("Presence Sensor Test");
    Serial.println("Wave hand in front of sensors...");
}

void loop() {
    static bool lastFront = false;
    static bool lastRear = false;

    bool frontDetected = digitalRead(SENSOR_FRONT);
    bool rearDetected = digitalRead(SENSOR_REAR);

    if (frontDetected != lastFront) {
        Serial.print("Front sensor: ");
        Serial.println(frontDetected ? "DETECTED" : "Clear");
        lastFront = frontDetected;
    }

    if (rearDetected != lastRear) {
        Serial.print("Rear sensor: ");
        Serial.println(rearDetected ? "DETECTED" : "Clear");
        lastRear = rearDetected;
    }

    delay(50);
}
```

### 6. Integrated Test: Servos + Sensors

**Combined Test:**

```cpp
// React to presence detection with neck movement
void loop() {
    bool frontDetected = digitalRead(SENSOR_FRONT);
    bool rearDetected = digitalRead(SENSOR_REAR);

    if (frontDetected) {
        // Look forward and down (attention pose)
        servos.WritePos(SERVO_PAN, 2048, 500);  // Center
        servos.WritePos(SERVO_TILT, 1500, 500); // Slight down
        Serial.println("Presence in front - looking");
    } else if (rearDetected) {
        // Turn around (pan 180°)
        servos.WritePos(SERVO_PAN, 3800, 1000);
        Serial.println("Presence behind - turning");
    } else {
        // Neutral position
        servos.WritePos(SERVO_PAN, 2048, 500);
        servos.WritePos(SERVO_TILT, 2048, 500);
        servos.WritePos(SERVO_ROLL, 2048, 500);
    }

    delay(100);
}
```

### 7. Document Servo Range Limits

**Calibration:**

```bash
# For each servo, find mechanical limits:
# 1. Move servo to extreme positions slowly
# 2. Note position values before binding/strain
# 3. Set software limits with safety margin

# Example ranges (will vary by mechanical design):
# Pan: 1024 (left) to 3072 (right), center 2048
# Tilt: 1500 (down) to 3000 (up), center 2250
# Roll: 1800 (left) to 2300 (right), center 2048
# Kickstand: 512 (retracted/up) to 3500 (deployed/down)

# Document in modules/neck/wiring.md
```

---

## Testing & Validation

**Test 1: Individual Servo Response**
```bash
# Each servo must respond to its ID without affecting others
# Upload test code, verify serial output shows correct servo moving
```

**Test 2: Daisy Chain Reliability**
```bash
# Send 100 position commands to each servo
# Verify: 100% response rate, no timeouts or errors
```

**Test 3: Concurrent Servo Movement**
```bash
# Command all 4 servos simultaneously to different positions
# Verify: All servos reach target positions smoothly
# Measure current draw: should be <4A peak
```

**Test 4: Presence Detection Coverage**
```bash
# Walk around breadboard setup at various distances (1m, 3m, 5m)
# Verify: Front sensor detects front hemisphere, rear detects rear
# Check for false positives (should be minimal)
```

**Test 5: Kickstand Load Test**
```bash
# Deploy kickstand (servo to extended position)
# Apply downward force (simulate robot weight, ~5-10kg)
# Verify: Servo holds position without slipping
# If slipping, increase servo torque or redesign linkage
```

---

## Troubleshooting

**Issue 1: Servo Not Responding**
- **Symptom:** One or more servos don't move when commanded
- **Solution:**
  - Check servo ID with FD software
  - Verify power supply voltage (6-8.4V)
  - Check data line connection (TX/RX on GPIO32)
  - Test servo individually (disconnect others)
  - Verify baud rate matches (1000000 or 115200)

**Issue 2: Servo Jitter or Vibration**
- **Symptom:** Servos vibrate when idle or at target position
- **Solution:**
  - Add bulk capacitor (1000µF) to servo power rail
  - Reduce PID gains in servo (use FD software)
  - Check for electrical noise on data line
  - Ensure common ground between ESP32 and servo power

**Issue 3: Communication Errors in Daisy Chain**
- **Symptom:** Only first servo works, or intermittent failures
- **Solution:**
  - Check daisy chain wiring (DATA OUT → DATA IN)
  - Verify all servos have unique IDs
  - Reduce baud rate (try 115200 instead of 1000000)
  - Add 1kΩ pull-up resistor on data line
  - Check cable quality (use short, low-resistance wires)

**Issue 4: Presence Sensors False Triggers**
- **Symptom:** Sensors trigger randomly without presence
- **Solution:**
  - For PIR: Increase trigger delay, shield from drafts/sunlight
  - For mmWave: Adjust sensitivity potentiometer (if available)
  - Add debouncing in software (require HIGH for 500ms)
  - Check sensor power supply (should be stable 5V)

**Issue 5: ESP32 Resets When Servos Move**
- **Symptom:** ESP32 crashes when multiple servos actuate
- **Solution:**
  - Use separate power supplies (5V for ESP32, 7.4V for servos)
  - Common ground is critical
  - Add 1000µF capacitor to servo power supply
  - Limit number of servos moving simultaneously

**Issue 6: Kickstand Servo Insufficient Torque**
- **Symptom:** Servo can't hold robot weight or slips
- **Solution:**
  - Verify servo is STS3215 (higher torque model)
  - Increase gear ratio in mechanical linkage
  - Use metal servo gears instead of plastic
  - Consider dual-servo kickstand design (Story 2.6)

---

## Dependencies

**Before this story:**
- Story 0.4: Set Up Development Tools and Dependencies ✅
- ESP32 development board available
- Feetech STS3215 servos purchased (×4)
- Presence sensors purchased (×2)
- Power supply (7.4V, 5A for servos)

**After this story:**
- Story 2.2: Design Neck Custom PCB in Fritzing

---

## References

- [Feetech STS3215 Datasheet](http://www.feetechrc.com/en_product.html)
- [SCServo Library Documentation](https://github.com/ftservo/SCServo)
- [PIR Sensor Tutorial](https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor)
- [RCWL-0516 mmWave Sensor](https://github.com/jdesbonnet/RCWL-0516)
- [Half-Duplex UART ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html)

---

## Notes

- **Servo Power:** STS3215 operates at 6-8.4V. Using 7.4V (2S LiPo) is optimal for torque and speed.
- **Daisy Chain Advantage:** Single data line simplifies wiring, but requires unique IDs and proper configuration.
- **ID Configuration:** Once set, servo IDs persist in EEPROM. Document IDs clearly for PCB design.
- **Kickstand Servo:** ID=4 used for kickstand (moved from Base to Neck module per architecture update).
- **Presence Sensors:** mmWave recommended over PIR for better through-wall detection and fewer false positives.
- **Current Budget:** 4 servos × 1A peak = 4A. Use 5A supply for margin. Measure actual current in Story 2.4.
- **Servo Range:** STS3215 position range is 0-4095 (12-bit). Center position is 2048. Map to 0-270° mechanical rotation.
- **Testing Duration:** Run breadboard test for several hours to identify flaky connections before PCB design.

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
