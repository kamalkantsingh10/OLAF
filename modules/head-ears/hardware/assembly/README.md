# Head+Ears Module Assembly Guide

Assembly instructions for the Head+Ears module (I2C Address: 0x08).

## Components

### Electronics
- 1× ESP32-WROOM-32 development board
- 2× OLED displays (128×64, SSD1306, SPI)
- 2× Feetech servo motors (for articulated ears)
- 1× Floor projector (pico projector with HDMI input)
- 1× PC817 optocoupler (or similar, for projector power control)
- 1× Linear servo (for projector focus adjustment)
- 1× Buck converter (5V, 3A)
- Resistors: 2.2kΩ (I2C pull-ups), 220Ω (optocoupler current limiting)
- Wiring: 22-24 AWG stranded wire

### Mechanical
- 3D printed parts (see `../mechanical/` directory)
- M3 screws and nuts
- Servo brackets and horns
- Cable ties

## Wiring Diagrams

### Overview
![Head+Ears Wiring Overview](wiring-diagram.png)

### ESP32 Pin Assignments

| Pin | Function | Connection |
|-----|----------|------------|
| **I2C (to Raspberry Pi)** |
| GPIO 21 | SDA | I2C bus (with 2.2kΩ pull-up) |
| GPIO 22 | SCL | I2C bus (with 2.2kΩ pull-up) |
| **OLED Displays (SPI)** |
| GPIO 5 | Left OLED CS | Left eye chip select |
| GPIO 17 | Left OLED DC | Left eye data/command |
| GPIO 16 | Left OLED RST | Left eye reset |
| GPIO 18 | Right OLED CS | Right eye chip select |
| GPIO 19 | Right OLED DC | Right eye data/command |
| GPIO 23 | Right OLED RST | Right eye reset |
| GPIO 18 | MOSI (SPI) | Shared SPI data |
| GPIO 19 | SCK (SPI) | Shared SPI clock |
| **Ear Servos (UART)** |
| GPIO 32 | Left Ear RX | Left servo UART RX |
| GPIO 33 | Left Ear TX | Left servo UART TX |
| GPIO 27 | Right Ear RX | Right servo UART RX |
| GPIO 14 | Right Ear TX | Right servo UART TX |
| **Projector Control** |
| GPIO 25 | Projector Power | Optocoupler input (projector power switch) |
| GPIO 26 | Focus Servo | PWM signal to linear servo |
| **Power** |
| 5V | VIN | From buck converter |
| GND | GND | Common ground |

### Projector Control Circuit

**Power Control (Optocoupler):**
```
ESP32 GPIO25 ──[ 220Ω ]──┬── PC817 Input (Pin 1)
                         │
                         └── PC817 Input (Pin 2) ── GND

PC817 Output (Pin 4) ────────── Projector Power Switch (+)
PC817 Output (Pin 3) ────────── Projector Power Switch (-)

Projector AC Adapter ──→ Optocoupler Switch ──→ Projector DC Input
```

**Focus Control (Linear Servo):**
```
Linear Servo:
  - Red: 5V
  - Brown: GND
  - Orange: GPIO26 (PWM signal)
```

### OLED Eye Wiring

**Left Eye (SSD1306):**
```
ESP32          OLED
GPIO 5    →    CS
GPIO 17   →    DC
GPIO 16   →    RST
GPIO 18   →    MOSI (D1)
GPIO 19   →    SCK (D0)
3.3V      →    VCC
GND       →    GND
```

**Right Eye:** Same connections, but CS/DC/RST use GPIOs 18, 19, 23

### Ear Servo Wiring

Each Feetech servo has 4 wires:
- **Red:** 5V power
- **Black:** Ground
- **Yellow:** UART RX (to ESP32 TX)
- **Green:** UART TX (to ESP32 RX)

## Assembly Steps

### 1. Prepare 3D Printed Parts
- [ ] Print all parts from `../mechanical/` directory
- [ ] Remove supports and sand surfaces
- [ ] Test fit all parts together

### 2. Install ESP32 and Buck Converter
- [ ] Mount ESP32 to base plate
- [ ] Mount buck converter nearby
- [ ] Connect 5V input (from torso power distribution)
- [ ] Verify 5V output with multimeter

### 3. Wire OLED Displays
- [ ] Solder wires to OLED modules (use color-coded wires)
- [ ] Connect to ESP32 SPI pins (see table above)
- [ ] Test displays (upload test firmware)
- [ ] Mount OLEDs in eye sockets

### 4. Install Ear Servos
- [ ] Mount servos in ear base brackets
- [ ] Connect servo wires to ESP32 UART pins
- [ ] Connect servo power (5V, GND)
- [ ] Test servo movement (calibration script)
- [ ] Attach ear panels to servo horns

### 5. Projector Control Circuit
- [ ] Solder optocoupler circuit on perfboard or PCB
- [ ] Connect GPIO25 to optocoupler input (with 220Ω resistor)
- [ ] Connect optocoupler output to projector power switch
- [ ] Mount linear servo in projector focus mechanism
- [ ] Connect linear servo to GPIO26

### 6. I2C Connection to Raspberry Pi
- [ ] Run 4-wire cable (SDA, SCL, GND, 3.3V) to Raspberry Pi
- [ ] Use JST-XH 4-pin connector (recommended)
- [ ] Add 2.2kΩ pull-up resistors on SDA/SCL (if first module)

### 7. HDMI Connection
- [ ] Connect HDMI cable from Raspberry Pi to projector
- [ ] Secure cable with strain relief

### 8. Final Assembly
- [ ] Route all cables neatly
- [ ] Use cable ties to secure bundles
- [ ] Close enclosure (top cover)
- [ ] Label module: "HEAD+EARS / 0x08"

## Testing Checklist

### Power Test
- [ ] Apply 5V power
- [ ] Check ESP32 power LED lights up
- [ ] Verify 5V on all component power lines
- [ ] Check for shorts or hot components

### I2C Communication Test
- [ ] Run `i2cdetect -y 1` on Raspberry Pi
- [ ] Verify module appears at address 0x08
- [ ] Send test I2C commands (register read/write)

### OLED Display Test
- [ ] Upload eye animation firmware
- [ ] Verify both eyes display correctly
- [ ] Test different expressions (blink, look left/right)

### Ear Servo Test
- [ ] Upload servo test firmware
- [ ] Calibrate servo positions (use `tools/calibration/servo_calibrator.py`)
- [ ] Test full range of motion (both ears, both DOF)

### Projector Control Test
- [ ] Send I2C command to turn projector ON
- [ ] Verify projector powers up
- [ ] Send focus adjustment commands
- [ ] Verify linear servo moves focus mechanism
- [ ] Test HDMI video output (display test pattern)

## Troubleshooting

**OLEDs not displaying:**
- Check SPI wiring (MOSI, SCK, CS, DC, RST)
- Verify 3.3V power to OLEDs
- Check firmware pin definitions match wiring

**Ear servos not responding:**
- Check UART wiring (TX ↔ RX crossover)
- Verify 5V power to servos
- Check baud rate in firmware (usually 115200 or 1000000)

**Projector won't turn on:**
- Check optocoupler connections
- Test optocoupler with multimeter (continuity when GPIO25 HIGH)
- Verify projector power adapter is working

**I2C not detected:**
- Check I2C wiring (SDA, SCL, GND)
- Verify pull-up resistors (2.2kΩ)
- Check I2C address in firmware (should be 0x08)
- Try lower I2C bus speed (100kHz vs 400kHz)

## Calibration

After assembly, run calibration scripts:

```bash
# Calibrate ear servos
python3 tools/calibration/servo_calibrator.py --module head-ears

# Test OLED animations
cd firmware/modules/head-ears
pio run --target upload

# Test projector control
python3 tools/diagnostics/test_projector.py
```

## Maintenance

- **Cable inspection:** Check for wear on moving parts (ears, neck connection)
- **Servo health:** Recalibrate servos if movement becomes jerky
- **OLED burn-in prevention:** Vary eye expressions, use screensaver mode
- **Projector focus:** Recalibrate if focus drifts over time

## Photos

(Add assembly photos here as you build)

- `step1-esp32-mounted.jpg`
- `step2-oleds-installed.jpg`
- `step3-servos-attached.jpg`
- `complete-front-view.jpg`
- `complete-internal-wiring.jpg`

## BOM (Bill of Materials)

See `../pcb/bom.csv` for complete parts list with suppliers and part numbers.
