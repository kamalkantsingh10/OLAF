# Standardized Module Backplane - Pin Allocation

**Design:** LEFT side (J1) = Common backplane, RIGHT side (J3) = Module-specific

---

## 1. Pin Allocation Table (J1 - Left Side Backplane)

| Physical Pin | GPIO | Function | Type | Connected To | Notes |
|--------------|------|----------|------|--------------|-------|
| **POWER** |
| J1-1 | 3.3V | Power 3.3V | Power | 3.3V devices | From ESP32 regulator |
| J1-2 | 3.3V | Power 3.3V | Power | 3.3V devices | Duplicate |
| J1-21 | 5V | Power 5V | Power | ESP32 VIN | External 5V input |
| J1-22 | GND | Ground | Ground | Common ground | Multiple GND available |
| **I2C (Raspberry Pi)** |
| J1-12 | 8 | I2C_SDA | I/O | Pi GPIO2 | Primary I2C data |
| J1-15 | 9 | I2C_SCL | I/O | Pi GPIO3 | Primary I2C clock |
| **UART1 (Serial Bus)** |
| J1-10 | 17 | UART1_TX | Output | Device RX | UART1 transmit (e.g., servo controller) |
| J1-11 | 18 | UART1_RX | Input | Device TX | UART1 receive (position feedback) |
| **UART2 (Sensors)** |
| J1-16 | 38 | UART2_TX | Output | Sensor RX | UART2 transmit (e.g., mmWave, GPS) |
| J1-17 | 39 | UART2_RX | Input | Sensor TX | UART2 receive (sensor data) |
| **EXPANSION** |
| J1-9 | 16 | EXP_GPIO | I/O | General purpose | Digital I/O |
| J1-7 | 7 | ADC_IN | Input | Analog sensor | ADC1_CH6, 0-3.3V |
| **DEBUG** |
| J1-3 | RST | Reset | Input | Reset button | Active LOW |

---

## 2. Available Interfaces (J3 - Right Side Free for Module-Specific Use)

### J3 Header - All Available for Module Logic

| GPIO Range | Pins Available | Suggested Use Cases |
|------------|----------------|---------------------|
| **General GPIO** | 1, 2, 21, 42, 47 | SPI CS, PWM servos, relays, LED control (WS2812 data) |
| **ADC Capable** | 20 (ADC2_CH9) | Secondary analog input if needed |
| **I2C Capable** | 40, 41 | Secondary I2C bus, additional sensors |
| **USB Capable** | 19 (USB_D-), 20 (USB_D+) | USB device implementation |
| **Built-in LED** | 48 | Onboard RGB status LED |
| **Debug UART** | 43 (TX), 44 (RX) | USB serial (don't use for other purposes) |

---

## 3. Interface Usage Guidelines

### UART1 vs UART2

**UART1 (GPIO 17/18) - Primary Serial Bus:**
- **Purpose:** Controllers, actuators, high-level devices
- **Examples:** Servo controllers, motor controllers, display modules
- **Typical Baud:** 115200 (standard for most controllers)
- **Use When:** Device controls multiple actuators or requires command/response

**UART2 (GPIO 38/39) - Sensor Serial Bus:**
- **Purpose:** Sensors, data acquisition, monitoring devices
- **Examples:** mmWave sensors (HLK-LD2461), GPS modules, environmental sensors
- **Typical Baud:** 9600-256000 (varies by sensor)
- **Use When:** Device provides continuous data stream or periodic measurements

### Module Examples

**Neck Module:**
- UART1 → Servo Controller (4× STS3215 servos)
- UART2 → HLK-LD2461 mmWave sensor @ 256000 baud

**Base Module:**
- UART1 → ODrive motor controller
- UART2 → GPS module @ 9600 baud

**Torso Module:**
- UART1 → Thermal printer (ESC/POS commands)
- UART2 → HLK-LD2461 mmWave sensor @ 256000 baud

### GPIO Allocation Summary

**J1 Backplane (Standardized):**
- GPIO 7, 8, 9, 16, 17, 18, 38, 39 (reserved for backplane)

**J3 Module-Specific (Free):**
- GPIO 1, 2, 19, 20, 21, 40, 41, 42, 47, 48 (available)

**Reserved (Do Not Use):**
- GPIO 0, 3, 45, 46 (strapping pins)
- GPIO 26-37 (internal flash/PSRAM)
- GPIO 43, 44 (USB serial debug)

---

**Last Updated:** 2025-12-22
**Change:** Added UART2 (GPIO 38/39) to standardized backplane for sensor communication
