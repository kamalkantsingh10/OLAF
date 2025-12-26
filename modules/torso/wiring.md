# Torso Module Wiring

**Module:** Torso (I2C Address: 0x0A)
**MCU:** ESP32-S3-DevKitC-1 (N16R8)
**Design:** Backplane-compatible ([backplane-pinout.md](../../docs/architecture/backplane-pinout.md))

---

## 1. Components

| Item | Qty | Specification | Notes |
|------|-----|---------------|-------|
| ESP32-S3-DevKitC-1 (N16R8) | 1 | 16MB Flash, 8MB PSRAM | I2C slave at 0x0A |
| ILI9341 2.8" TFT Touch | 1 | 320×240 RGB, SPI, 65K colors | Heart-shaped display |
| EM5820/QR701 Thermal Printer | 1 | Mini 58mm, ESC/POS, UART | 12V power required |
| HLK-LD2461 mmWave Sensor | 1 | 24GHz 2T4R, UART @ 256000 baud | Rear-facing, 5V |

---

## 2. Power Requirements

| Rail | Component | Current | Notes |
|------|-----------|---------|-------|
| **12V** | Thermal Printer | 200mA idle, 2A peak | Separate 12V supply required |
| **5V** | ESP32-S3 | 80-500mA | VIN pin |
| **5V** | ILI9341 TFT | 100-150mA | Display + backlight |
| **5V** | HLK-LD2461 | 330-400mA | mmWave sensor |
| **Total 5V** | All logic | ~640mA typical, 1.5A peak | - |

**CRITICAL:** Thermal printer needs 12V external supply (2A capacity). Never connect to ESP32.

---

## 3. Pin Assignments

### Backplane Pins (J1 - Standardized)

| GPIO | Function | Connection | Protocol |
|------|----------|------------|----------|
| 8 | I2C_SDA | Raspberry Pi GPIO 2 | I2C Slave @ 400kHz |
| 9 | I2C_SCL | Raspberry Pi GPIO 3 | I2C Slave @ 400kHz |
| 17 | UART1_TX | Thermal Printer RX | UART @ 19200 baud |
| 18 | UART1_RX | Thermal Printer TX | UART @ 19200 baud |
| 38 | UART2_TX | mmWave Sensor RX | UART @ 256000 baud |
| 39 | UART2_RX | mmWave Sensor TX | UART @ 256000 baud |

### Module-Specific Pins (J3)

| GPIO | Function | Connection | Protocol |
|------|----------|------------|----------|
| 13 | SPI_MOSI | TFT SDI/MOSI | SPI @ 40MHz |
| 14 | SPI_SCK | TFT SCK | SPI Clock |
| 21 | TFT_DC | TFT D/C | Data/Command select |
| 47 | TFT_CS | TFT CS | Chip Select |
| 2 | TFT_RST | TFT Reset | Hardware reset |
| 42 | TFT_BL | TFT Backlight | PWM (optional) |

---

## 4. Wiring Diagram

```
                   ESP32-S3 (0x0A)
                         │
        ┌────────────────┼────────────────┐
        │                │                │
    I2C (8/9)      UART1 (17/18)    UART2 (38/39)
        │                │                │
        ▼                ▼                ▼
  Raspberry Pi    Thermal Printer   HLK-LD2461
   (GPIO 2/3)       (12V power)      (5V power)
        │                │                │
        └────────── GND ─┴────────────────┘

              SPI Display (13/14/21/47/2/42)
                         │
                    ILI9341 TFT
                    (5V power)
```

---

## 5. Connections

### A. I2C to Raspberry Pi

| ESP32 | Pi | Notes |
|-------|-----|-------|
| GPIO 8 | GPIO 2 (Pin 3) | SDA, 4.7kΩ pull-up |
| GPIO 9 | GPIO 3 (Pin 5) | SCL, 4.7kΩ pull-up |
| GND | GND (Pin 6) | Common ground |

### B. ILI9341 TFT Display (SPI)

| ESP32 | TFT | Notes |
|-------|-----|-------|
| GPIO 13 | MOSI | SPI data |
| GPIO 14 | SCK | SPI clock |
| GPIO 21 | DC | Data/Command |
| GPIO 47 | CS | Chip select |
| GPIO 2 | RST | Reset |
| GPIO 42 | BL | Backlight (PWM) |
| 5V | VCC | Display power |
| GND | GND | Ground |

### C. Thermal Printer (UART)

| ESP32 | Printer | Notes |
|-------|---------|-------|
| GPIO 17 | RX | Commands |
| GPIO 18 | TX | Feedback |
| GND | GND | Signal ground only |
| - | VCC | **12V external (2A)** |

**Protocol:** 19200 baud, ESC/POS commands

### D. HLK-LD2461 mmWave Sensor (Rear)

| ESP32 | Sensor | Notes |
|-------|--------|-------|
| GPIO 38 | RX | UART2 TX |
| GPIO 39 | TX | UART2 RX |
| 5V | VCC | 330mA typical |
| GND | GND | Ground |

**Protocol:** 256000 baud, binary data

---

## 6. Testing

### Test 1: TFT Display
```bash
cd modules/torso/firmware
pio run -t upload -t monitor
# Expected: Heart animation on display
```

### Test 2: Thermal Printer
```bash
# Send serial: "Hello OLAF"
# Expected: Text prints on thermal paper
```

### Test 3: mmWave Sensor
```bash
# Walk behind breadboard
# Expected: Serial shows hex data stream
```

### Test 4: I2C
```bash
# On Raspberry Pi
i2cdetect -y 1
# Expected: Device at 0x0A
```

---

## 7. Notes

- **TFT requires 5V** (not 3.3V) for VCC
- **Printer requires 12V** external supply - never connect to ESP32
- **mmWave requires 5V** (330mA) - not 3.3V
- **Rear sensor** complements front sensor on Neck for 360° coverage

---


