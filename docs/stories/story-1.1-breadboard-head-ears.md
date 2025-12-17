# Story 1.1: Breadboard Head+Ears Components and Test Connectivity

**Epic:** Epic 1 - Head+Ears Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** all Head+Ears components breadboarded and individually tested,
**so that** I can verify functionality before committing to PCB design.

---

## Acceptance Criteria

1. ✅ ESP32 development board connected to breadboard with power supply
2. ✅ 2× OLED displays (128×64, SPI) wired and displaying test images
3. ✅ 2× ear servo buses wired (4 servos total: 2-DOF per ear, Feetech UART)
4. ✅ Floor projector power circuit breadboarded (GPIO → optocoupler → relay/MOSFET)
5. ✅ Floor projector focus servo wired (linear servo via PWM/UART)
6. ✅ OAK-D-Pro camera connected to separate test system (Pi) and verified working
7. ✅ Component-level test script confirms: OLEDs display animations, servos move to commanded positions, projector power switches on/off, focus servo adjusts

---

## Implementation Steps

### 1. Set Up ESP32 Development Board

```bash
# Install PlatformIO if not already done (from Story 0.4)
pip install platformio

# Create test project
cd ~/olaf/modules/head-ears
mkdir -p firmware/breadboard_test
cd firmware/breadboard_test
pio init --board esp32dev
```

**Wiring:**
- ESP32 5V pin → Breadboard 5V rail
- ESP32 GND pin → Breadboard GND rail
- USB connection to laptop for programming and power

### 2. Wire and Test OLED Displays (SPI)

**Components:**
- 2× OLED displays (128×64, SSD1306 or SH1106, SPI interface)
- Jumper wires

**Wiring for OLED 1 (Left Eye):**
- VCC → 3.3V
- GND → GND
- SCK (Clock) → ESP32 GPIO18 (VSPI SCK)
- MOSI (Data) → ESP32 GPIO23 (VSPI MOSI)
- DC (Data/Command) → ESP32 GPIO21
- CS (Chip Select) → ESP32 GPIO5
- RST (Reset) → ESP32 GPIO16

**Wiring for OLED 2 (Right Eye):**
- VCC → 3.3V
- GND → GND
- SCK → ESP32 GPIO18 (shared)
- MOSI → ESP32 GPIO23 (shared)
- DC → ESP32 GPIO22
- CS → ESP32 GPIO17
- RST → ESP32 GPIO16 (shared)

**Test Code:**

```cpp
// platformio.ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    adafruit/Adafruit GFX Library
    adafruit/Adafruit SSD1306

// src/main.cpp
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_DC1 21
#define OLED_CS1 5
#define OLED_RESET 16
#define OLED_DC2 22
#define OLED_CS2 17

Adafruit_SSD1306 leftEye(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC1, OLED_RESET, OLED_CS1);
Adafruit_SSD1306 rightEye(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC2, OLED_RESET, OLED_CS2);

void setup() {
  Serial.begin(115200);

  if(!leftEye.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("Left eye OLED failed");
  }
  if(!rightEye.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("Right eye OLED failed");
  }

  leftEye.clearDisplay();
  rightEye.clearDisplay();

  // Draw circles (eyes)
  leftEye.fillCircle(64, 32, 20, SSD1306_WHITE);
  rightEye.fillCircle(64, 32, 20, SSD1306_WHITE);

  leftEye.display();
  rightEye.display();
}

void loop() {
  // Blink animation
  delay(2000);
  leftEye.clearDisplay();
  rightEye.clearDisplay();
  leftEye.display();
  rightEye.display();

  delay(200);
  leftEye.fillCircle(64, 32, 20, SSD1306_WHITE);
  rightEye.fillCircle(64, 32, 20, SSD1306_WHITE);
  leftEye.display();
  rightEye.display();
}
```

**Test:**
```bash
pio run --target upload
pio device monitor
# Verify: Both OLEDs show circles, then blink
```

### 3. Wire and Test Ear Servos (Feetech STS3215, UART)

**Components:**
- 4× Feetech STS3215 servos (2 per ear, 2-DOF each)
- Servo power supply (6-8.4V, 2A minimum)
- Logic level converter (3.3V ESP32 ↔ 5V servo logic)

**Servo UART Protocol:**
- Half-duplex serial communication
- Each servo has unique ID (1-4)
- Baud rate: 1000000 (1Mbps) or 115200

**Wiring for Left Ear Bus:**
- Servo 1 (ID=1) & Servo 2 (ID=2) daisy-chained
- Data line → Logic level converter → ESP32 GPIO26 (UART1 TX/RX via direction control)
- Servos VCC → 6V power supply
- Servos GND → Common GND

**Wiring for Right Ear Bus:**
- Servo 3 (ID=3) & Servo 4 (ID=4) daisy-chained
- Data line → Logic level converter → ESP32 GPIO27 (UART2 TX/RX)
- Servos VCC → 6V power supply
- Servos GND → Common GND

**Test Code:**

```cpp
// lib_deps += sct3215/Feetech SCS
#include <SCServo.h>

SMS_STS servos;
HardwareSerial SerialLeft(1);  // UART1
HardwareSerial SerialRight(2); // UART2

void setup() {
  Serial.begin(115200);

  // Left ear servos (ID 1, 2)
  SerialLeft.begin(1000000, SERIAL_8N1, 26, 26); // RX=TX=GPIO26
  servos.pSerial = &SerialLeft;

  // Test left ear servos
  servos.WritePos(1, 2048, 1000); // Servo 1 to center, 1000ms
  servos.WritePos(2, 2048, 1000); // Servo 2 to center
  delay(1500);

  // Right ear servos (ID 3, 4)
  SerialRight.begin(1000000, SERIAL_8N1, 27, 27);
  servos.pSerial = &SerialRight;

  servos.WritePos(3, 2048, 1000); // Servo 3 to center
  servos.WritePos(4, 2048, 1000); // Servo 4 to center
}

void loop() {
  // Ear wiggle animation
  servos.pSerial = &SerialLeft;
  servos.WritePos(1, 3072, 500); // Up
  servos.WritePos(2, 3072, 500);
  delay(600);

  servos.WritePos(1, 1024, 500); // Down
  servos.WritePos(2, 1024, 500);
  delay(600);
}
```

**Note:** Feetech servos require half-duplex UART. You may need a direction control circuit (GPIO to enable TX/RX switching) or use a library that handles this automatically.

### 4. Wire and Test Projector Power Circuit

**Components:**
- Optocoupler (PC817 or similar)
- MOSFET or relay (for switching projector power, 12V/5V depending on projector)
- Resistors (220Ω for LED, 10kΩ pull-down for MOSFET gate)

**Wiring:**
- ESP32 GPIO25 → 220Ω resistor → Optocoupler LED anode
- Optocoupler LED cathode → GND
- Optocoupler transistor collector → MOSFET gate
- Optocoupler transistor emitter → GND
- MOSFET source → GND
- MOSFET drain → Projector power supply negative
- Projector power supply positive → Projector VCC
- 10kΩ resistor: MOSFET gate to GND (pull-down)

**Test Code:**

```cpp
#define PROJECTOR_POWER_PIN 25

void setup() {
  Serial.begin(115200);
  pinMode(PROJECTOR_POWER_PIN, OUTPUT);
  digitalWrite(PROJECTOR_POWER_PIN, LOW); // Off
}

void loop() {
  Serial.println("Projector ON");
  digitalWrite(PROJECTOR_POWER_PIN, HIGH);
  delay(3000);

  Serial.println("Projector OFF");
  digitalWrite(PROJECTOR_POWER_PIN, LOW);
  delay(3000);
}
```

**Test:**
- Use multimeter to verify MOSFET switching (drain voltage changes)
- Connect LED in series with projector power line to visualize switching

### 5. Wire and Test Projector Focus Servo

**Components:**
- Linear servo or standard servo with focus mechanism attachment
- Power supply (5-6V)

**Wiring:**
- Servo signal → ESP32 GPIO32 (PWM capable)
- Servo VCC → 5V
- Servo GND → GND

**Test Code:**

```cpp
#include <ESP32Servo.h>

Servo focusServo;
#define FOCUS_SERVO_PIN 32

void setup() {
  Serial.begin(115200);
  focusServo.attach(FOCUS_SERVO_PIN);
  focusServo.write(90); // Center position
}

void loop() {
  Serial.println("Focus: Near");
  focusServo.write(0);
  delay(2000);

  Serial.println("Focus: Far");
  focusServo.write(180);
  delay(2000);

  Serial.println("Focus: Center");
  focusServo.write(90);
  delay(2000);
}
```

### 6. Test OAK-D-Pro Camera

**On Raspberry Pi (separate test):**

```bash
# Install DepthAI
pip install depthai

# Run demo
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"

# Test RGB camera
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python/examples
python3 ColorCamera/rgb_preview.py
```

**Verify:**
- Camera detected via USB
- RGB feed displays
- Depth map functional (test depth demo)

### 7. Create Integrated Test Script

```cpp
// Comprehensive test combining all components
// Create test sequences:
// 1. OLED eye animations (open, blink, look left/right)
// 2. Ear movements (wiggle, perk up, flatten)
// 3. Projector power cycle
// 4. Focus servo sweep
// 5. All components running simultaneously

void testSequence() {
  Serial.println("=== HEAD+EARS MODULE TEST ===");

  // Eyes blink
  Serial.println("Test 1: Eye blink");
  blinkEyes(3);

  // Ears wiggle
  Serial.println("Test 2: Ear wiggle");
  wiggleEars(2);

  // Projector power
  Serial.println("Test 3: Projector power");
  testProjectorPower();

  // Focus servo
  Serial.println("Test 4: Focus control");
  testFocusServo();

  Serial.println("=== TEST COMPLETE ===");
}
```

---

## Testing & Validation

**Test 1: OLED Display**
```bash
# Upload test code
pio run --target upload
# Expected: Both OLEDs display graphics, blink animation at 30+ FPS
```

**Test 2: Ear Servos**
```bash
# Expected: All 4 servos respond to commands, smooth motion, no jitter
# Measure current draw: should be <2A for all servos combined
```

**Test 3: Projector Power Circuit**
```bash
# Use multimeter to verify switching
# Expected: MOSFET drain voltage toggles between 0V and projector supply voltage
```

**Test 4: OAK-D-Pro Camera (on Pi)**
```bash
python3 depthai-python/examples/ColorCamera/rgb_preview.py
# Expected: Video stream at 30 FPS, depth map functional
```

**Test 5: Integrated System**
```bash
# Run all components simultaneously for 5 minutes
# Monitor temperature (ESP32, servos, power supply)
# Check for voltage sag on power rails
# Verify no component failures
```

---

## Troubleshooting

**Issue 1: OLED Not Displaying**
- **Symptom:** Blank screen on one or both OLEDs
- **Solution:**
  - Check wiring (SPI pins, VCC, GND)
  - Verify I2C address if using I2C OLEDs (use I2C scanner)
  - Try different OLED library (SSD1306 vs SH1106)
  - Check power supply (should be 3.3V, not 5V for most OLEDs)

**Issue 2: Servos Not Responding**
- **Symptom:** Servos don't move or only some move
- **Solution:**
  - Check servo IDs with Feetech debug tool
  - Verify baud rate (1000000 or 115200)
  - Ensure half-duplex UART is configured correctly
  - Check power supply (servos need 6-8.4V, 2A minimum)
  - Add 1000µF capacitor across servo power supply for stability

**Issue 3: ESP32 Resets When Servos Move**
- **Symptom:** ESP32 crashes or resets when commanding servos
- **Solution:**
  - Separate power supplies for ESP32 (5V USB) and servos (6V dedicated)
  - Common ground between supplies is critical
  - Add bulk capacitor (1000µF) to servo power supply
  - Reduce number of servos moving simultaneously

**Issue 4: Projector Power Circuit Doesn't Switch**
- **Symptom:** MOSFET doesn't turn on/off
- **Solution:**
  - Check optocoupler orientation (LED polarity)
  - Verify GPIO output (use Serial.println or multimeter)
  - MOSFET may need higher gate voltage (use logic-level MOSFET like IRLZ44N)
  - Check pull-down resistor on gate (10kΩ to GND)

**Issue 5: OAK-D-Pro Not Detected**
- **Symptom:** `depthai` doesn't find camera
- **Solution:**
  - Check USB cable (use USB 3.0 port and cable)
  - Update DepthAI library: `pip install --upgrade depthai`
  - Verify permissions: `sudo usermod -aG plugdev $USER` (logout/login)
  - Try different USB port
  - Check camera power LED (should be lit)

**Issue 6: Servo Daisy Chain Communication Fails**
- **Symptom:** Only first servo responds
- **Solution:**
  - Verify servo wiring: Data line must connect through all servos in series
  - Check servo IDs are unique (1, 2, 3, 4)
  - Use Feetech configuration tool to set IDs if needed
  - Ensure all servos on same bus have same baud rate

---

## Dependencies

**Before this story:**
- Story 0.4: Set Up Development Tools and Dependencies ✅
- ESP32 development board available
- Jumper wires, breadboards, power supplies
- Head+Ears components purchased (OLEDs, servos, projector, camera)

**After this story:**
- Story 1.2: Design Head+Ears Custom PCB in Fritzing

---

## References

- [Adafruit GFX Library](https://learn.adafruit.com/adafruit-gfx-graphics-library)
- [SSD1306 OLED Tutorial](https://randomnerdtutorials.com/esp32-ssd1306-oled-display-arduino-ide/)
- [Feetech STS3215 Documentation](http://www.feetechrc.com/en_product.html)
- [ESP32Servo Library](https://github.com/madhephaestus/ESP32Servo)
- [OAK-D-Pro Documentation](https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9095.html)
- [DepthAI Python Examples](https://github.com/luxonis/depthai-python/tree/main/examples)

---

## Notes

- **Power Budget:** ESP32 (500mA), OLEDs (200mA total), Servos (2A peak), Projector (varies) → Use 5V/3A for ESP32+OLEDs, separate 6V/3A for servos
- **Servo Selection:** Feetech STS3215 chosen for UART daisy-chaining capability and compact size
- **OLED Refresh Rate:** Target 30 FPS for smooth eye animations (33ms per frame)
- **Camera Mounting:** OAK-D-Pro will mount in Head+Ears enclosure but USB runs to Pi in Torso
- **Projector Control:** Focus servo may need custom mechanical linkage (design in Story 1.5)
- **Testing Duration:** Breadboard testing should run for several hours to identify flaky connections
- **Component Substitutions:** If Feetech servos unavailable, can use Dynamixel XL-320 (also UART daisy-chainable)

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
