# Story 3.1: Breadboard Torso Components and Test Connectivity

**Epic:** Epic 3 - Torso Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 3-5 hours

---

## User Story

**As a** builder,
**I want** all Torso components breadboarded and individually tested,
**so that** I can verify functionality before committing to PCB design.

---

## Acceptance Criteria

1. ✅ ESP32 development board connected to breadboard with power supply
2. ✅ 2.8" square display (SPI) wired and displaying test animations (beating heart pattern)
3. ✅ Thermal printer wired (UART or parallel interface) and printing test output
4. ✅ Component-level test script confirms: display animates smoothly, printer outputs text and simple graphics

---

## Implementation Steps

### 1. Set Up ESP32

```bash
cd ~/olaf/modules/torso
mkdir -p firmware/breadboard_test
cd firmware/breadboard_test
pio init --board esp32dev
```

### 2. Wire 2.8" Square Display (SPI)

**Display: 2.8" 320×320 IPS LCD (ST7789 driver typical)**

**Wiring:**
```
Display → ESP32
VCC → 3.3V
GND → GND
SCK → GPIO18 (SPI SCK)
MOSI → GPIO23 (SPI MOSI)
DC → GPIO21
CS → GPIO5
RST → GPIO16
BL (Backlight) → 3.3V (or PWM for brightness control)
```

**Test Code:**

```cpp
// platformio.ini
lib_deps =
    bodmer/TFT_eSPI @ ^2.5.0

// src/main.cpp
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

void setup() {
    Serial.begin(115200);
    tft.init();
    tft.setRotation(0); // Portrait mode
    tft.fillScreen(TFT_BLACK);

    // Draw heart shape
    drawHeart(160, 160, 50, TFT_RED);
}

void loop() {
    // Pulsing heart animation
    static int scale = 40;
    static int dir = 1;

    tft.fillScreen(TFT_BLACK);
    drawHeart(160, 160, scale, TFT_RED);

    scale += dir * 2;
    if (scale > 60 || scale < 40) dir *= -1;

    delay(100);
}

void drawHeart(int x, int y, int size, uint16_t color) {
    // Simple heart shape using filled circles and triangle
    tft.fillCircle(x - size/3, y - size/4, size/3, color);
    tft.fillCircle(x + size/3, y - size/4, size/3, color);
    tft.fillTriangle(
        x - size*2/3, y - size/6,
        x + size*2/3, y - size/6,
        x, y + size,
        color
    );
}
```

### 3. Wire Thermal Printer

**Printer: Mini thermal printer (e.g., CSN-A2-T, Adafruit printer)**

**Wiring (UART):**
```
Printer TX → ESP32 GPIO17 (RX)
Printer RX → ESP32 GPIO16 (TX)
Printer VCC → 5-9V power supply (separate, high current)
Printer GND → Common GND
```

**Test Code:**

```cpp
// lib_deps += adafruit/Adafruit Thermal Printer Library

#include <Adafruit_Thermal.h>
#include <HardwareSerial.h>

HardwareSerial printerSerial(2); // UART2
Adafruit_Thermal printer(&printerSerial);

void setup() {
    Serial.begin(115200);
    printerSerial.begin(19200, SERIAL_8N1, 17, 16); // RX=17, TX=16

    printer.begin();
    printer.println("OLAF Torso Module");
    printer.println("Thermal Printer Test");
    printer.println();
    printer.setSize('L');
    printer.println("Heart Beat!");
    printer.feed(2);

    Serial.println("Print complete");
}

void loop() {
    // Print on command
    if (Serial.available()) {
        String text = Serial.readStringUntil('\n');
        printer.println(text);
        printer.feed(1);
    }
}
```

### 4. Integrated Test

```cpp
// Combined: Display heart + print on button press
void loop() {
    // Animate heart on display
    animateHeart();

    // Check for print trigger (e.g., GPIO button or serial command)
    if (digitalRead(BUTTON_PIN) == LOW) {
        printer.println("Heart beat recorded!");
        printer.feed(1);
        delay(500); // Debounce
    }
}
```

---

## Testing & Validation

**Test 1: Display Animation**
```bash
# Upload code
# Expected: Heart pulses smoothly at ~10 FPS
# Check brightness, color, no flickering
```

**Test 2: Printer Output**
```bash
# Send serial command: "Hello OLAF"
# Expected: Text prints on thermal paper
# Check: Clear text, proper formatting, paper feeds
```

**Test 3: Power Consumption**
```bash
# Measure current:
# - ESP32 + Display: ~200-300mA @ 3.3V
# - Printer (idle): ~100mA
# - Printer (printing): ~1.5A peak
# Total: Design for 2A @ 5V
```

---

## Troubleshooting

**Issue: Display Shows Garbage or Blank**
- **Solution:** Check SPI wiring, verify display driver (ST7789 vs ILI9341), configure TFT_eSPI library (User_Setup.h)

**Issue: Printer Doesn't Respond**
- **Solution:** Verify baud rate (19200 or 9600), check TX/RX crossover, ensure separate 5-9V power supply with adequate current

**Issue: Printer Prints Gibberish**
- **Solution:** Check baud rate match, verify serial polarity (TX↔RX), test with Adafruit example code

---

## Dependencies

**Before this story:**
- Story 0.4: Development tools installed ✅
- 2.8" display purchased
- Thermal printer purchased

**After this story:**
- Story 3.2: Design Torso Custom PCB

---

## References

- [TFT_eSPI Library](https://github.com/Bodmer/TFT_eSPI)
- [Adafruit Thermal Printer Guide](https://learn.adafruit.com/mini-thermal-receipt-printer)

---

## Notes

- **Display:** 2.8" square (320×320) is unusual size. Verify before ordering. 2.4" or 3.5" round displays also work.
- **Printer Power:** Thermal printers draw high current (1-2A) during printing. Use separate 5-9V supply, not ESP32 regulator.
- **Heart Animation:** Simple pulsing for Phase 1. Phase 2 can add variable rhythms (calm, excited, etc.).

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
