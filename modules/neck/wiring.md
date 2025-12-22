# Neck Module Wiring

**Module:** Neck (I2C Address: 0x09)
**MCU:** ESP32-S3-DevKitC-1 (N16R8)
**Design:** Backplane-compatible ([backplane-pinout.md](../../docs/architecture/backplane-pinout.md))

---

## 1. Hardware Requirements

### Components

| Item | Qty | Specification |
|------|-----|---------------|
| ESP32-S3-DevKitC-1 (N16R8) | 1 | 16MB Flash, 8MB PSRAM |
| WS2812B LED Strip | 1 | 8 LEDs, 5V, single data line |
| Feetech Serial Bus Controller | 1 | STSC series, UART interface |
| Feetech STS3215 Servo | 4 | 30 kg·cm (IDs: 1=Pan, 2=Tilt, 3=Roll, 4=Kickstand) |
| HLK-LD2461 mmWave Sensor | 1 | 24GHz 2T4R radar, UART @ 256000 baud, front-facing |
| 470Ω Resistor | 1 | 1/4W, for LED data line |
| 100µF Capacitor | 1 | 16V electrolytic, for LED power |

### Power Requirements

| Rail | Load | Typical | Peak | Source |
|------|------|---------|------|--------|
| 5V | ESP32 (80mA) + LEDs (120mA) + HLK-LD2461 (330mA) | 530mA | 1.5A | **External 5V @ 2A recommended** |
| 6-12V | 4× Servos (motor power) | 1-2A | 4A+ | **Separate supply (NOT ESP32)** |

---

## 2. Pin Assignments

### Backplane Pins (J1 - Standardized)

| GPIO | Function | External Connection | Protocol |
|------|----------|---------------------|----------|
| 8 | I2C_SDA | Raspberry Pi GPIO 2 | I2C Slave @ 400kHz |
| 9 | I2C_SCL | Raspberry Pi GPIO 3 | I2C Slave @ 400kHz |
| 17 | UART_TX | Servo Controller RX | UART @ 115200 baud |
| 18 | UART_RX | Servo Controller TX | UART @ 115200 baud |
| 16 | EXP_GPIO | - | Available |
| 7 | ADC_IN | - | Available (ADC1_CH6) |

### Module-Specific Pins (J3)

| GPIO | Function | External Connection | Protocol |
|------|----------|---------------------|----------|
| 1 | LED_DATA | WS2812B DIN (via 470Ω) | Single-wire @ 800kHz |
| 38 | MMWAVE_TX | mmWave sensor RX | UART2 TX @ 115200 baud |
| 39 | MMWAVE_RX | mmWave sensor TX | UART2 RX @ 115200 baud |
| 48 | ONBOARD_LED | Built-in RGB LED | Output (heartbeat) |
| 43 | USB_TX | Debug console | UART0 (reserved) |
| 44 | USB_RX | Debug console | UART0 (reserved) |

### Available for Future Use
GPIO 2, 20, 21, 40, 41, 42, 47

### Reserved (Do Not Use)
- **GPIO 26-37:** Internal flash (26-32) and PSRAM (33-37)
- **GPIO 0, 3, 45, 46:** Strapping pins (boot mode)

---

## 3. Wiring Diagram

```
                        ┌─────────────────────────────────────┐
                        │   ESP32-S3-DevKitC-1 (N16R8)        │
                        │         (I2C Slave: 0x09)           │
                        └─────────────────────────────────────┘
                                      │
        ┌─────────────────────────────┼─────────────────────────────┐
        │                             │                             │
        │ I2C SLAVE (GPIO 8/9)        │ UART1 (GPIO 17/18)         │ UART2 (GPIO 38/39)
        │                             │                             │
        ▼                             ▼                             ▼
┌───────────────┐          ┌──────────────────────┐      ┌──────────────────┐
│ Raspberry Pi  │          │ Serial Bus Servo Ctrl│      │  HLK-LD2461      │
│               │          │                      │      │ (Front, 2T4R)    │
│ GPIO 2 (SDA) ─┼──[4.7kΩ]│                      │      │                  │
│ GPIO 3 (SCL) ─┼──[4.7kΩ]│  RX ←── GPIO 17 (TX) │      │ RX ←── GPIO 38   │
│ GND ──────────┼──────────│  TX ──→ GPIO 18 (RX) │      │ TX ──→ GPIO 39   │
└───────────────┘          │  GND ────────────────┤      │ VCC ←── 5V       │
                           │                      │      │ GND ────────────┐│
                           │  Serial Bus Out      │      └─────────────────┘│
                           └──────┬───────────────┘                         │
                                  │                                         │
                    ┌─────────────┴──────────────┐                         │
                    │                            │                         │
              ┌─────▼─────┐  ┌──────▼──────┐  ┌─▼────────┐  ┌──────▼─────┐│
              │ Servo ID1 │  │ Servo ID2   │  │Servo ID3 │  │ Servo ID4  ││
              │   (Pan)   │  │  (Tilt)     │  │ (Roll)   │  │(Kickstand) ││
              └───────────┘  └─────────────┘  └──────────┘  └────────────┘│
                    ▲                                                       │
                    │ Motor Power (6-12V, 4A+)                             │
              ┌─────┴──────┐                                               │
              │ Servo PSU  │                                               │
              │ (ISOLATED) │                                               │
              └────────────┘                                               │
                                                                            │
   ┌────────────────────────────────────────────────────────────────────────┘
   │
   │  WS2812B LED Strip (GPIO 1)
   │  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
   │  │ LED0 │ LED1 │ LED2 │ LED3 │ LED4 │ LED5 │ LED6 │ LED7 │
   │  │ Bat  │ ROS2 │ Head │ Neck │Torso │ Base │  AI  │ Nav  │
   │  └───┬──┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
   │      │
   │      │ DIN ←── GPIO 1 ──[470Ω]──
   │      │ VCC ←── 5V ──[100µF]──
   │      │ GND ──────────────────────────────┐
   │                                          │
   │                                          │
   └────────── Common GND ────────────────────┴──────────────────┐
                    ▲                                            │
              ┌─────┴──────┐                                     │
              │  5V Supply │                                     │
              │  (1.5A+)   │─────────────────────────────────────┘
              └────────────┘
```

**Legend:**
- `──` Wire connection
- `[470Ω]` Resistor (470 ohms)
- `[100µF]` Capacitor (100 microfarads)
- `[4.7kΩ]` Pull-up resistor (4.7 kilo-ohms)
- `←──` / `──→` Signal direction

---

## 4. Wiring Instructions

Follow these instructions to connect the Neck module components. Work systematically through each section, verifying connections before moving to the next. The I2C and UART connections use low-power signals, while the WS2812 LEDs and servos require careful power management.

### A. I2C to Raspberry Pi

This connection allows the Raspberry Pi to control the Neck module as an I2C slave device. The Pi will send commands to control LEDs, servos, and read sensor data. Pull-up resistors are critical for reliable I2C communication - verify these are present (usually on the Pi side).

| ESP32 Pin | Pi Pin | Wire Gauge | Notes |
|-----------|--------|------------|-------|
| GPIO 8 | GPIO 2 (Pin 3) | 24 AWG | 4.7kΩ pull-up required |
| GPIO 9 | GPIO 3 (Pin 5) | 24 AWG | 4.7kΩ pull-up required |
| GND | GND (Pin 6) | 22 AWG | Common ground |

### B. WS2812B LED Strip

The 8-LED status strip provides visual feedback for system health. Each LED represents a different component or status (battery, modules, AI, navigation). The 470Ω resistor protects the data line from voltage spikes, while the 100µF capacitor smooths power delivery during LED color changes. These components are essential for reliable operation - don't skip them.

**Important:** WS2812 LEDs must be powered from 5V (not 3.3V). The ESP32 GPIO signal (3.3V) is sufficient for data, but the LEDs themselves need 5V to function.

```
ESP32 GPIO 1 ──[470Ω]── WS2812 DIN
ESP32 5V ──────┬──────── WS2812 VCC
            [100µF]
ESP32 GND ─────┴──────── WS2812 GND
```

**LED Index Assignment:**
- 0: Battery/Power | 1: ROS2 Orchestrator | 2: Head+Ears (0x08) | 3: Neck (0x09)
- 4: Torso (0x0A) | 5: Base (0x0B) | 6: AI/Cloud | 7: Navigation/SLAM

### C. Serial Bus Servo Controller

The servo controller manages 4× STS3215 servos (neck pan/tilt/roll and kickstand) via a single UART connection. The servos are daisy-chained on the controller's serial bus, each with a unique ID (1-4).

**CRITICAL:** Servo motors draw 1-2 amps each and require 6-12V power from a **separate external supply**. Never connect servo motor power to the ESP32 - this will destroy the board. Only the signal lines (TX/RX) and ground connect to the ESP32.

| ESP32 Pin | Controller Pin | Wire Gauge | Notes |
|-----------|----------------|------------|-------|
| GPIO 17 | RX | 24 AWG | Command data |
| GPIO 18 | TX | 24 AWG | Position feedback |
| GND | GND | 22 AWG | Signal ground only |
| - | Motor Power | 18-20 AWG | **6-12V external supply** |

Servos connect to controller serial bus (daisy-chain).

### D. mmWave Presence Sensor (Front) - HLK-LD2461

The HLK-LD2461 is an advanced 24GHz mmWave radar sensor with 2T4R antenna configuration (2 transmit, 4 receive) for high-precision multi-target tracking. It detects human presence, tracks trajectories, and monitors up to 5 people simultaneously via UART communication. An identical HLK-LD2461 sensor (rear-facing) will be installed on the Torso module to provide full 360° coverage tracking up to 10 total targets (5 front + 5 rear).

**HLK-LD2461 Specifications:**
- **Antenna Config:** 2T4R (2 transmit, 4 receive antennas)
- **Power:** 5V DC (260-400mA, typically 330mA)
- **Communication:** UART @ 256000 baud (faster than LD2450/LD2412)
- **Detection Range (Motion):** 0-8 meters
- **Detection Range (Stationary):** 0-6 meters
- **Multi-target Tracking:** Up to 5 simultaneous targets
- **Frequency:** 24.00-24.25 GHz (ISM band)
- **Tracking:** Real-time trajectory and position data

| ESP32 Pin | Sensor Pin | Wire Gauge | Notes |
|-----------|------------|------------|-------|
| GPIO 38 | RX | 24 AWG | UART2 TX (ESP32 transmit to sensor) |
| GPIO 39 | TX | 24 AWG | UART2 RX (ESP32 receive from sensor) |
| 5V | VCC | 22 AWG | Check sensor spec (usually 5V) |
| GND | GND | 22 AWG | - |

**Protocol:** UART @ 256000 baud (HLK-LD2461 default)
**Note:** 2nd HLK-LD2461 sensor (rear-facing) will be on Torso module for full 360° coverage
**Data Format:** Custom binary protocol - see [LD2461 datasheet](https://drive.google.com/drive/folders/14_KgZpL4Th2LTRuq_W0X_ePjHu_Z6lcS) for packet structure

### E. Power Distribution

Proper power distribution is critical for reliable operation. The system uses two separate power supplies: a 5V rail for logic and sensors, and a higher voltage (6-12V) rail exclusively for servo motors. **Never mix these power supplies** - connecting servo power to the ESP32 will cause permanent damage.

All grounds must be connected together (star-ground topology) to provide a common reference voltage. However, power rails remain isolated - only grounds are shared.

**5V Supply Requirements:** **Recommended 2A capacity** to handle ESP32 (500mA peak), LEDs (480mA peak), and HLK-LD2461 sensor (400mA peak). The LD2461 draws significantly more power than simpler sensors due to its 2T4R antenna array and advanced processing. Use a quality regulated supply to avoid voltage drops under load.

**Servo Supply:** Minimum 4A capacity at 6-12V. Servos draw 1-2A each during movement. Check your servo specifications for exact voltage requirements.

```
5V Supply (2A recommended)
  ├─ ESP32 VIN
  ├─ WS2812 VCC (add 100µF capacitor)
  └─ HLK-LD2461 VCC (260-400mA peak!)

Servo Supply (6-12V, 4A+) ──> Servo Controller Motor Power
                           └─ GND to common ground
```

---

## 5. Testing & Validation

### Step 1: I2C Communication
```bash
# On Raspberry Pi
i2cdetect -y 1
# Expected: Device detected at 0x09
```

### Step 2: LED Functionality
Upload firmware, run LED test pattern.
**Expected:** All 8 LEDs light up in sequence.

### Step 3: Servo Control
Send UART position command to servo ID 1.
**Expected:** Pan servo moves, position feedback received.

### Step 4: mmWave Sensor Communication
Send query command via UART2, read sensor response.
**Expected:** Sensor responds with presence detection data.

---
