# Epic 5: Core Torso & Power System - Complete Build

**Epic Goal:** Complete the core torso and power infrastructure by 3D printing torso panels and power enclosure (with Epic 2 feedback incorporated), assembling complete torso housing with Raspberry Pi mounting and heart LCD display (1.53" Round TFT 360×360 ST77916 QSPI, Body Module I2C 0x0A), and building robust power system with 36V hoverboard battery, BMS, buck converters, charging circuit, power distribution harness, and safety features (fuses, emergency cutoff, voltage monitoring).

**Duration:** 2-3 weeks (Weeks 11-13)

**Prerequisites:**
- Epic 1 (Foundation & I2C Communication) - I2C architecture validated
- Epic 2 (Complete OnShape Design & Community Feedback) - All STLs exported and feedback incorporated
- Epic 3 (Complete Head Assembly) - Head module completed for eventual integration

**Value Delivered:** Complete power infrastructure enabling autonomous operation, integrated heart display creating emotional body language, professional component housing for Raspberry Pi and electronics, safe charging system for extended runtime, thermal-tested power distribution ready for full robot integration in Epic 7.

**Architecture Focus:** This epic establishes the Body Module (I2C 0x0A) controlling heart LCD display, implements robust 36V power architecture with multiple voltage rails (36V→12V for motors, 36V→5V for electronics), validates BMS and safety systems, and creates mechanical housing for orchestrator (Raspberry Pi) and power components.

**Hardware Note:** Body Module requires new ESP32-S3 with QSPI interface for high-resolution heart display (360×360 ST77916). Power system uses temporary bench power for Epic 1-3, now transitions to onboard battery system.

---

## Story 5.1: 3D Print Core Torso Panels & Power Enclosure

**As a** hardware builder,
**I want** all torso components 3D printed with Epic 2 feedback incorporated,
**so that** I have all physical parts ready for assembly.

### Acceptance Criteria:

1. **STL Files Verified:**
   - Torso STLs from Epic 2 design downloaded from OnShape
   - Files reviewed for completeness:
     - `torso-main-chassis.stl` (main structural frame)
     - `torso-front-panel.stl` (front panel with heart LCD cutout)
     - `torso-back-panel.stl` (back panel with access door)
     - `torso-side-left.stl`, `torso-side-right.stl` (side panels)
     - `heart-lcd-bezel.stl` (decorative bezel for heart display)
     - `heart-lcd-mount.stl` (LCD mounting bracket)
     - `raspberry-pi-mount.stl` (Pi mounting plate with standoffs)
     - `power-enclosure-base.stl` (battery compartment base)
     - `power-enclosure-lid.stl` (battery compartment lid with ventilation)
     - `buck-converter-mount.stl` (mounting bracket for converters)
     - `power-distribution-board-mount.stl` (custom power board mounting)
     - `cable-channels-torso.stl` (cable routing guides)
     - `component-bay-dividers.stl` (organization dividers for electronics)

2. **3D Printing Settings:**
   - Material: PETG (heat resistance for power components, strength for structural)
   - Layer height: 0.2mm (0.15mm for LCD mount precision)
   - Infill: 30% for structural parts (chassis, power enclosure), 20% for panels
   - Supports: Auto-generated where needed (LCD bezel, cable channels)
   - Print orientation optimized for strength and minimal supports

3. **Printing Execution:**
   - All components printed successfully
   - No warping, layer separation, or print failures
   - Support material removed cleanly
   - Screw holes drilled/cleaned to proper size (M3 for electronics, M4 for structural)
   - Power enclosure lid ventilation holes clear (heat dissipation)

4. **Quality Verification:**
   - Heart LCD cutout fits 1.53" round display (40mm diameter)
   - Raspberry Pi mount fits Pi 4B dimensions with clearance for cables
   - Battery compartment dimensions fit 36V hoverboard battery pack
   - Buck converter mounts fit common DC-DC converter modules (LM2596, XL4015)
   - Cable channels accommodate 14AWG power wiring
   - Structural integrity: Chassis can support head assembly weight (test with substitute weight)

5. **Post-Processing:**
   - Support scars sanded smooth
   - Functional surfaces (mounting holes, LCD bezel) checked for fit
   - Test-fit major components: Pi, battery (if available), converters
   - Heat-sensitive areas (near converters): Verify PETG thermal resistance

6. **Documentation:**
   - Print time logged: `hardware/3d-models/print-log.md`
   - Photos of printed parts: `hardware/3d-models/Body/assembly-parts-photos/`
   - Any print issues noted for future reprints

**Dependencies:** Epic 2 complete (STLs exported, community feedback incorporated)

**Estimated Effort:** 12-16 hours print time (spread over 3-4 days) + 3-4 hours post-processing

**Note:** Power enclosure is largest print - plan printer schedule accordingly. Battery compartment may require multi-part design if exceeds printer bed size.

---

## Story 5.2: Heart LCD Display Module - Hardware Assembly

**As a** hardware builder,
**I want** the heart LCD display physically mounted and wired to ESP32-S3,
**so that** I have working Body Module hardware ready for firmware.

### Acceptance Criteria:

1. **Component Preparation:**
   - 1.53" Round TFT LCD 360×360 (ST77916 driver, QSPI interface) acquired
   - ESP32-S3-WROOM-2 (N16R8) for Body Module
   - M2.5 screws for LCD mounting
   - 15cm jumper wires for QSPI connections

2. **LCD Mounting:**
   - Heart LCD mounted in `heart-lcd-mount.stl` bracket
   - Bracket secured to `torso-front-panel.stl` with M3 screws
   - `heart-lcd-bezel.stl` decorative bezel attached (covers mounting hardware)
   - Display centered in torso front panel cutout
   - Display aligned: Vertical orientation, parallel to front panel

3. **QSPI Wiring (ESP32-S3 → ST77916 LCD):**
   - **Power:**
     - VCC → 3.3V (ESP32-S3)
     - GND → GND
   - **QSPI Interface (high-speed display):**
     - CS (Chip Select) → GPIO10
     - SCK (Clock) → GPIO12
     - SDA0 (Data 0) → GPIO11
     - SDA1 (Data 1) → GPIO13
     - SDA2 (Data 2) → GPIO14
     - SDA3 (Data 3) → GPIO15
     - DC (Data/Command) → GPIO16
     - RST (Reset) → GPIO17
     - BL (Backlight) → GPIO18 (PWM for brightness control)

4. **ESP32-S3 Mounting:**
   - ESP32-S3 secured inside torso chassis (accessible compartment)
   - USB port access maintained for firmware updates
   - QSPI wiring organized and secured (keep wires <15cm to reduce interference)
   - I2C wiring prepared (to Raspberry Pi, installed in Story 5.4):
     - SDA → GPIO21
     - SCL → GPIO22
     - I2C address configured in firmware: **0x0A** (Body Module)

5. **Power Testing:**
   - Connect ESP32 to bench 5V supply
   - ESP32 boots: Blue LED on, serial output visible
   - LCD backlight turns on (verify GPIO18 controls brightness)
   - No shorts: Multimeter check on all power rails before power-on

6. **Mechanical Fit Verification:**
   - LCD bezel flush with front panel
   - No flex or wobble in LCD mount
   - ESP32 accessible for USB firmware updates
   - Cable routing allows front panel to close without pinching wires

7. **Documentation:**
   - Wiring photos: `hardware/wiring/body-module-assembly.jpg`
   - QSPI pinout documented: `firmware/body/README.md`
   - Component locations diagram

**Dependencies:** Story 5.1 (torso parts printed)

**Estimated Effort:** 4-6 hours

**Note:** ST77916 QSPI interface enables 60 FPS animation for smooth heart beating. Firmware integration in Story 5.5.

---

## Story 5.3: Power System Assembly - Battery, BMS & Distribution

**As a** hardware builder,
**I want** complete power system assembled with battery, BMS, converters, and safety features,
**so that** Olaf can operate autonomously with safe power delivery.

### Acceptance Criteria:

1. **Components Acquired:**
   - **Battery:** 36V 4.4Ah hoverboard battery pack (10S2P 18650 cells) with integrated BMS
   - **Buck Converters:**
     - 36V→12V converter (10A rated) for future motor use (ODrive in Epic 6)
     - 36V→5V converter (10A rated) for Raspberry Pi, ESP32s, servos
   - **Safety Components:**
     - Main power switch (rocker switch, 10A rated)
     - Emergency cutoff button (normally-closed push button)
     - Fuse holder + fuses: 10A for 12V rail, 10A for 5V rail
     - Voltage monitor module (displays battery voltage)
   - **Wiring:**
     - 14AWG wire (red/black) for 36V main lines
     - 16AWG wire for 12V/5V distribution
     - XT60 connectors (battery → distribution board)
     - Anderson Powerpole connectors (distribution → loads)
   - **Charging:**
     - 36V (42V max) lithium battery charger (2A recommended)
     - Barrel jack connector for charging port

2. **Battery Installation:**
   - 36V battery pack placed in `power-enclosure-base.stl`
   - Battery secured with velcro straps (prevents movement during balancing)
   - BMS wires organized: Balance leads, charge/discharge protection
   - Battery compartment lid allows ventilation (lithium safety)

3. **Power Distribution Board Assembly:**
   - Custom power distribution board (perfboard or PCB):
     ```
     [36V Battery] → [Main Switch] → [Emergency Cutoff] → [Fuse Block] → [Buck Converters]
                                                                         ↓
                                                           12V Rail (future motors)
                                                           5V Rail (Pi, ESP32s, servos)
     ```
   - All connections soldered or screw-terminal secured
   - Voltage monitor connected across battery terminals (always-on, low current draw)
   - Fuses installed in holders (not directly soldered for easy replacement)

4. **Buck Converter Configuration:**
   - **36V→12V Converter:**
     - Input: 36V from battery (post-fuse)
     - Output adjusted: 12.0V ±0.2V (measured with multimeter)
     - Load tested: 5A dummy load (resistor bank or test motor)
     - Output stable: No voltage drop under load
   - **36V→5V Converter:**
     - Input: 36V from battery (post-fuse)
     - Output adjusted: 5.0V ±0.1V (critical for Pi/ESP32 stability)
     - Load tested: 3A dummy load (simulate Pi + 2 ESP32s)
     - Ripple tested: <50mV ripple with oscilloscope (if available)

5. **Safety System Testing:**
   - **Main Power Switch:**
     - OFF: Voltage monitor shows battery voltage, all other rails dead
     - ON: All rails energized, voltage monitor displays 36V
   - **Emergency Cutoff:**
     - Button pressed: All power rails immediately disconnected
     - Button released: Power restored (normally-closed circuit)
   - **Fuse Protection:**
     - Intentional short test: Fuse blows, prevents damage
     - Fuse replacement verified

6. **Output Wiring:**
   - **5V Rail Distribution:**
     - Raspberry Pi: Anderson Powerpole connector → GPIO header 5V pins (or USB-C PD if Pi 5)
     - ESP32 Head Module: JST connector → VIN (5V)
     - ESP32 Ears+Neck Module: JST connector → VIN (5V)
     - ESP32 Body Module: JST connector → VIN (5V)
     - Servo controllers (Epic 3 ears+neck): Anderson Powerpole → 5V (separate line, high current)
   - **12V Rail:**
     - Placeholder wiring for Epic 6 (ODrive motor controller)
   - All connectors labeled with voltage rail and amperage

7. **Thermal Testing:**
   - **30-Minute Load Test:**
     - Connect all Epic 3 hardware (Pi, 3 ESP32s, 7 servos) to 5V rail
     - Run head expressions continuously (Story 3.10 test)
     - Monitor temperatures:
       - Buck converters: <60°C (measure with IR thermometer or thermocouple)
       - Battery: <40°C (lithium safety limit)
       - Wiring: No warm spots (indicates high resistance joints)
     - Voltage stability: 5V rail remains 4.9-5.1V throughout test
   - **Thermal Shutdown Test:**
     - Overload 5V rail: 12A load (exceeds 10A rating)
     - Verify buck converter thermal protection kicks in (output cuts off, no damage)
     - Converter recovers after cooling (automatic restart or manual reset)

8. **Charging System:**
   - Barrel jack charging port installed in torso back panel
   - Charger connected: BMS indicates charging (LED or voltage monitor shows 42V)
   - Charge full battery: 0% → 100% (measure time, verify BMS cutoff at 42V)
   - Charging while system powered: Verify safe (BMS allows simultaneous charge/discharge)

9. **Voltage Monitoring & Cutoff:**
   - Battery voltage displayed on voltage monitor (visible from outside torso)
   - Low voltage warning: 33V (critical: 30V shutdown to prevent over-discharge)
   - Software monitoring (Story 5.6): Pi reads battery voltage via ADC or I2C voltage sensor

10. **Cable Management:**
    - All power wiring routed through `cable-channels-torso.stl`
    - High-current wires (36V, 5V mains) separated from signal wires (I2C, USB)
    - Strain relief: All connectors secured to prevent wire pull
    - Access panel allows maintenance without full disassembly

11. **Documentation:**
    - Power system diagram: `hardware/wiring/power-distribution-diagram.png`
    - Buck converter settings noted: Output voltages, current limits
    - Safety checklist: Fuse ratings, emergency cutoff procedure
    - Battery care: Charging procedure, storage voltage (50% = ~36V)
    - Wiring photos: `hardware/wiring/power-system-assembly/`

**Dependencies:** Story 5.1 (power enclosure printed)

**Estimated Effort:** 10-14 hours

**Critical Safety Note:** Lithium batteries require proper BMS, fuses, and ventilation. NEVER charge unattended initially. Test BMS cutoff functionality before full integration.

---

## Story 5.4: Raspberry Pi Integration & Torso Assembly

**As a** hardware builder,
**I want** Raspberry Pi mounted in torso with all connections,
**so that** the orchestrator is integrated with power and body module.

### Acceptance Criteria:

1. **Component Preparation:**
   - Raspberry Pi 4B (or Pi 5) with heatsinks/fan (optional but recommended)
   - MicroSD card (64GB, ROS2 + orchestrator software from Epic 1)
   - USB cables: Conference mic+speaker (Epic 3 note: audio is USB-based)
   - I2C wiring to Body Module ESP32 (from Story 5.2)
   - I2C bus from Epic 3: Head (0x08), Ears+Neck (0x09) already connected

2. **Raspberry Pi Mounting:**
   - Pi secured to `raspberry-pi-mount.stl` with M2.5 screws and standoffs
   - Mount positioned in torso chassis (central location, accessible)
   - GPIO header accessible for I2C connections
   - USB ports facing accessible direction (for mic, speaker, peripherals)
   - Heat dissipation: Clearance for airflow, heatsinks not blocked

3. **Power Connection:**
   - 5V from power distribution board → Pi GPIO pins (5V + GND) or USB-C (Pi 5)
   - Current tested: Pi under load (ROS2 nodes running) draws <3A
   - Stable power: Pi boots successfully, no undervoltage warnings (`vcgencmd get_throttled`)

4. **I2C Bus Integration:**
   - **Existing I2C devices (Epic 3):**
     - Head Module: 0x08
     - Ears+Neck Module: 0x09
   - **New I2C device:**
     - Body Module: 0x0A (ESP32-S3 from Story 5.2)
   - **I2C Wiring:**
     - Pi GPIO2 (SDA) → I2C bus (shared with all ESP32 modules)
     - Pi GPIO3 (SCL) → I2C bus (shared with all ESP32 modules)
     - Common ground: All ESP32s + Pi share GND
     - Pull-up resistors: 4.7kΩ on SDA/SCL (verify with multimeter, may be onboard Pi)
   - **I2C Scan:**
     - Command: `sudo i2cdetect -y 1`
     - Expected output: Devices at 0x08, 0x09, 0x0A
     - No conflicts: All addresses unique

5. **USB Peripherals:**
   - USB conference microphone connected (for Epic 13 voice, placeholder now)
   - USB conference speaker connected (for Epic 13 voice, placeholder now)
   - Peripherals detected: `lsusb` shows audio devices
   - Test audio: `arecord` captures mic, `aplay` plays speaker (verify hardware functional)

6. **Network Configuration:**
   - WiFi configured (for SSH access, cloud AI API in Epic 13)
   - Static IP optional (recommended for ROS2 networking)
   - SSH enabled: Remote access for development
   - ROS2 domain ID set: Environment variable `ROS_DOMAIN_ID=42` (from tech stack)

7. **Torso Structural Assembly:**
   - `torso-front-panel.stl` (with heart LCD) attached to `torso-main-chassis.stl`
   - `torso-back-panel.stl` secured (removable for maintenance)
   - `torso-side-left.stl`, `torso-side-right.stl` attached
   - Power enclosure integrated into torso base
   - Cable routing verified: All wiring fits inside chassis without pinching
   - Structural integrity: Torso can support head assembly weight (test fit if head available)

8. **Component Organization:**
   - `component-bay-dividers.stl` installed: Separates Pi, power components, ESP32s
   - Cable management: `cable-channels-torso.stl` organizes wiring
   - Access panels clearly labeled: "Power Access", "Pi Access", "Body Module Access"

9. **Functional Testing:**
   - Pi boots from SD card (ROS2 orchestrator software from Epic 1)
   - I2C communication: Test Epic 3 head expressions work with new power system
   - Body Module responds: `i2cget -y 1 0x0A 0x00` returns module ID (0x0A)
   - Power system stable: All components powered, no voltage sag
   - Thermal check: 15 minutes continuous operation, no overheating

10. **Documentation:**
    - Photos: Complete torso assembly (multiple angles)
    - Wiring diagram: `hardware/wiring/torso-complete-wiring.png`
    - I2C bus topology: Updated with Body Module 0x0A
    - Pi configuration notes: `ros2/README.md` updated with I2C setup

**Dependencies:** Story 5.2 (Body Module hardware), Story 5.3 (power system), Epic 3 (Head/Ears+Neck modules for I2C testing)

**Estimated Effort:** 6-8 hours

**Note:** This story completes torso hardware. Epic 3 head can be test-fitted but full integration happens in Epic 7.

---

## Story 5.5: Body Module ESP32 Firmware - Heart LCD Animation

**As a** firmware developer,
**I want** ESP32 firmware rendering animated heart on ST77916 LCD with I2C control,
**so that** the orchestrator can express emotions through body display.

### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO project: `firmware/body/firmware/`
   - Platform: ESP32-S3 (Arduino framework)
   - Libraries:
     - `Wire.h` (I2C slave)
     - `Adafruit_GFX.h` (graphics primitives)
     - `Adafruit_ST7789.h` (ST77916 driver, compatible with ST7789 library for round displays)
     - Custom QSPI initialization for ST77916 (may require library modification or custom driver)

2. **I2C Slave Implementation:**
   - ESP32-S3 configured as I2C slave at address **0x0A**
   - I2C receive/request handlers implemented
   - Interrupt-driven I2C communication

3. **I2C Register Map (Body Module 0x0A):**
   - `0x00`: Module ID (returns 0x0A)
   - `0x02`: Status byte (READY/BUSY/ERROR)
   - **Heart Animation Control:**
     - `0x10`: Animation mode (0=off, 1=idle-pulse, 2=excited-fast, 3=sad-slow, 4=custom)
     - `0x11`: Heart rate BPM (40-180, maps to animation speed)
     - `0x12`: Color hue (0-255, HSV color wheel for heart color)
     - `0x13`: Brightness (0-255, PWM backlight control)
   - **Display Control:**
     - `0x20`: Display on/off (0=off, 1=on)
     - `0x21`: Test pattern (write 1 to show test pattern for calibration)

4. **ST77916 QSPI Display Driver:**
   - QSPI interface initialized (GPIO10-18 from Story 5.2)
   - Display resolution: 360×360 pixels (circular mask applied for round display)
   - Frame buffer: 360×360 RGB565 (requires 259.2KB, may need PSRAM on ESP32-S3)
   - Target frame rate: 30 FPS minimum (60 FPS ideal)
   - Backlight PWM: GPIO18 controls brightness (0-100%)

5. **Heart Animation Engine:**
   - **Rendering Pipeline:**
     - Clear frame buffer (black background)
     - Draw heart shape (parametric or sprite-based)
     - Apply animation transform (scale, color pulse)
     - Transfer frame buffer to LCD via QSPI
   - **Animation Modes:**
     - **Idle Pulse (mode 1):** Gentle heartbeat (60 BPM default)
       - Scale: 90% → 100% → 90% (smooth easing)
       - Color: Solid red (#FF0000)
     - **Excited Fast (mode 2):** Rapid heartbeat (120 BPM)
       - Scale: 85% → 105% → 85% (more pronounced)
       - Color: Bright red with slight yellow tint (#FF3030)
     - **Sad Slow (mode 3):** Slow heartbeat (45 BPM)
       - Scale: 95% → 100% → 95% (subtle)
       - Color: Darker red/blue tint (#CC0044)
     - **Custom (mode 4):** BPM and color from registers 0x11, 0x12
   - **Heart Rate Calculation:**
     - BPM → Animation period (ms): `period = 60000 / BPM`
     - Example: 60 BPM = 1000ms per beat cycle

6. **Graphics Implementation:**
   - Heart shape rendered using:
     - Option A: Parametric equations (mathematical heart curve)
     - Option B: Sprite bitmap (pre-rendered heart PNG → C array)
   - Center position: (180, 180) - center of 360×360 display
   - Size: Heart fills ~60% of display diameter (216 pixels)
   - Anti-aliasing: Smooth edges (if performance allows)

7. **Performance Optimization:**
   - Use PSRAM for frame buffer (ESP32-S3 N16R8 has 8MB PSRAM)
   - DMA transfer for QSPI (minimize CPU blocking)
   - Animation loop: Non-blocking, runs independently of I2C interrupts
   - Target: <33ms per frame (30 FPS minimum)
   - Memory stable: No heap fragmentation over 30 minutes

8. **Testing:**
   - Firmware compiled and uploaded to ESP32-S3
   - Serial: "I2C Slave initialized at 0x0A - Body Module"
   - Display initializes: Backlight on, test pattern renders (concentric circles)
   - I2C scan: `sudo i2cdetect -y 1` shows device at 0x0A
   - **Animation Tests:**
     - Write mode 1 (idle pulse): Heart beats at 60 BPM, smooth animation
     - Write mode 2 (excited): Heart beats at 120 BPM, faster animation
     - Write mode 3 (sad): Heart beats at 45 BPM, slower animation
     - Custom test: Write BPM 90, hue 120 (green heart) → animates at 90 BPM in green
   - **Brightness Control:**
     - Write brightness 255: Full brightness
     - Write brightness 128: Half brightness (backlight dims)
     - Write brightness 0: Display dark (backlight off, but heart still animating)
   - Frame rate measured: Actual FPS (log to serial or count frames)

9. **Integration Testing:**
   - I2C latency: Command sent → animation mode changes <50ms
   - Combined test: Send multiple commands (mode + BPM + color) → all apply simultaneously
   - Reliability: 50 mode changes without failure or visual glitches
   - Thermal check: 30 minutes continuous animation, ESP32 <65°C, display <50°C

10. **Code Quality:**
    - Files:
      - `main.cpp`
      - `i2c_slave.cpp`
      - `st77916_driver.cpp` (QSPI display driver)
      - `heart_animation.cpp` (animation engine)
      - `heart_graphics.cpp` (heart shape rendering)
    - Comments explain QSPI initialization, animation timing, heart rate math
    - Code committed to `firmware/body/firmware/`

**Dependencies:** Story 5.2 (Body Module hardware assembled)

**Estimated Effort:** 10-14 hours

**Technical Note:** ST77916 QSPI requires custom driver work if Adafruit library doesn't support. May need to adapt existing ST7789/GC9A01 libraries. 360×360 resolution is demanding - PSRAM essential.

---

## Story 5.6: ROS2 Body Driver Node & Power Monitoring

**As a** software developer,
**I want** ROS2 driver node for Body Module (heart display) and power system monitoring,
**so that** the orchestrator can control body expressions and monitor battery health.

### Acceptance Criteria:

1. **Node Structure:**
   - Python ROS2 node: `ros2/src/orchestrator/ros2_nodes/hardware_drivers/body_driver.py`
   - Node name: `/olaf/body_driver`
   - Uses `rclpy` and `smbus2`

2. **I2C Communication:**
   - Opens I2C bus 1: `bus = SMBus(1)`
   - Helper functions for register read/write to address **0x0A**

3. **ROS2 Subscriptions:**
   - `/olaf/body/heart_animation` (custom msg: mode, bpm, color_hue, brightness)
     - Callback writes to registers 0x10-0x13 (animation control)
   - `/olaf/body/display_power` (std_msgs/Bool)
     - Callback writes to register 0x20 (display on/off)

4. **ROS2 Publications:**
   - `/olaf/body/status` (std_msgs/String) at 10Hz
     - Reads status register 0x02, publishes "READY", "BUSY", or "ERROR"
   - `/olaf/body/current_animation` (custom msg) at 1Hz
     - Reads registers 0x10-0x13, publishes current animation state
   - `/olaf/power/battery_voltage` (std_msgs/Float32) at 1Hz
     - Reads battery voltage via ADC or I2C voltage sensor module
     - Publishes voltage in volts (e.g., 36.5V)
   - `/olaf/power/battery_percentage` (std_msgs/UInt8) at 1Hz
     - Calculates percentage: 30V=0%, 42V=100% (linear approximation)
     - Publishes 0-100%

5. **Power Monitoring Implementation:**
   - **Option A: ADC Voltage Divider:**
     - 36V battery → Voltage divider (10kΩ + 1kΩ) → Pi GPIO ADC (3.3V max)
     - Ratio: 36V → 3.27V (11:1 divider)
     - Read ADC: Convert to voltage using calibration factor
   - **Option B: I2C Voltage Sensor:**
     - INA219/INA226 current/voltage sensor module on I2C bus
     - Read voltage register (more accurate than ADC)
     - Bonus: Also reads current draw (future feature)
   - Calibration: Measure actual battery voltage with multimeter, adjust software calibration constant

6. **Battery Health Warnings:**
   - **Low Battery Warning:**
     - Voltage <33V: Publish `/olaf/warnings/low_battery` (std_msgs/String: "Low battery - 33V")
     - Trigger visual/audio warning (future: make heart pulse red rapidly)
   - **Critical Battery Shutdown:**
     - Voltage <30V: Publish `/olaf/warnings/critical_battery` (std_msgs/String: "Critical - shutdown now!")
     - Initiate graceful shutdown sequence (future Epic 13 integration)
   - **Overvoltage Warning:**
     - Voltage >43V: Publish warning (indicates charger malfunction or BMS failure)

7. **Module Health Check:**
   - On startup: Read register 0x00, verify response is 0x0A
   - If not responding: Log error, publish ERROR status
   - Periodic health check: Every 30s verify module responsive

8. **Error Handling:**
   - I2C timeout handling (module busy during animation rendering)
   - Retry logic: 3 attempts with 100ms delay
   - Persistent failure: Publish ERROR, log details
   - ADC read errors: Log warning, use last known voltage (don't crash)

9. **Testing:**
   - Node launches: `ros2 run orchestrator body_driver`
   - Topics appear: `/olaf/body/heart_animation`, `/olaf/power/battery_voltage`
   - **Heart Animation Tests:**
     - Publish mode 1 (idle pulse): Heart beats at 60 BPM
     - Publish mode 2 (excited): Heart beats faster
     - Publish custom: BPM 90, hue 180 (cyan heart) → heart animates in cyan at 90 BPM
     - Publish brightness 128: Heart dims
   - **Power Monitoring Tests:**
     - Battery at full charge (~42V): `/olaf/power/battery_voltage` publishes 42.0
     - Battery at ~50% (~36V): `/olaf/power/battery_percentage` publishes 50
     - Discharge battery below 33V: Low battery warning published
   - **Display Control:**
     - Publish display_power False: Heart LCD turns off (backlight off)
     - Publish display_power True: Heart LCD turns on

10. **Integration Testing:**
    - Combined test: Publish heart animation while monitoring battery voltage → both work simultaneously
    - Latency: Animation command → visible change <100ms
    - Reliability: 50 animation mode changes without failure

11. **Code Quality:**
    - File: `body_driver.py`
    - Message definitions:
      - `orchestrator/msg/HeartAnimation.msg`
      - Reuse or create battery messages
    - Comments explain power monitoring approach (ADC or I2C sensor)
    - Voltage calculation calibration constants documented
    - Code committed to `ros2/src/orchestrator/ros2_nodes/hardware_drivers/`

**Dependencies:** Story 5.5 (Body Module firmware), Story 5.3 (power system for voltage monitoring)

**Estimated Effort:** 6-8 hours

**Note:** Power monitoring is critical for battery safety. Test with fully charged and nearly depleted battery to verify thresholds.

---

## Story 5.7: Coordinated Expression Enhancement - Body Integration

**As a** software developer,
**I want** Epic 3 coordinated expression system enhanced with body (heart display) integration,
**so that** Olaf displays coherent emotions across eyes, ears, neck, AND body.

### Acceptance Criteria:

1. **Existing System Base:**
   - Start from Epic 3 `coordinated_expression.py`
   - Existing: Eyes (0x08), Ears+Neck (0x09) coordination working
   - 7 emotions already defined (neutral, happy, curious, thinking, confused, sad, excited)

2. **Body Expression Mapping:**
   - Enhance emotion definitions to include heart animation parameters:
   - **Neutral:**
     - Heart: Mode 1 (idle pulse), 60 BPM, red (#FF0000), brightness 128
   - **Happy:**
     - Heart: Mode 2 (excited), 90 BPM, bright red (#FF3030), brightness 255
   - **Curious:**
     - Heart: Mode 1 (idle pulse), 75 BPM, orange (#FF8800), brightness 200
   - **Thinking:**
     - Heart: Mode 3 (slow), 50 BPM, blue-tinted (#8800FF), brightness 180
   - **Confused:**
     - Heart: Mode 1 (idle pulse), 65 BPM, purple (#CC00CC), brightness 150
   - **Sad:**
     - Heart: Mode 3 (slow), 45 BPM, dark blue (#0044AA), brightness 100
   - **Excited:**
     - Heart: Mode 2 (excited), 120 BPM, yellow-red (#FFAA00), brightness 255

3. **Multi-Modal Coordination Logic Enhancement:**
   - For each emotion, compute and publish:
     - Eye expression → `/olaf/head/expression` (Epic 1)
     - Ear positions → `/olaf/ears/position` (Epic 3)
     - Neck position → `/olaf/neck/position` (Epic 3)
     - **NEW: Heart animation** → `/olaf/body/heart_animation` (Epic 5)
   - All four modality commands published simultaneously (atomic emotion change)

4. **Color-Emotion Mapping Research:**
   - Document color choices in code comments (psychology-based):
     - Red: Excitement, love, passion (happy, excited)
     - Orange: Curiosity, playfulness (curious)
     - Blue: Calm, sadness, thinking (sad, thinking)
     - Purple: Confusion, mystery (confused)
   - Hue values (0-255 HSV):
     - Red: 0, Orange: 21, Yellow: 42, Green: 85, Cyan: 127, Blue: 170, Purple: 200

5. **Timing Coordination:**
   - All modality commands (eyes, ears, neck, heart) published within 10ms
   - Smooth transitions: Heart animation mode changes interpolated over 500ms (firmware handles)
   - Synchronized start: Heart rate change aligns with neck movement start

6. **Presence-Aware Behaviors Enhancement:**
   - Existing: Presence detected → "curious" expression (eyes+ears+neck)
   - **NEW:** Presence detected → Heart BPM increases to 90 (excited to see person)
   - Presence lost → Heart BPM returns to 60 (calm idle)
   - Duration-based: If presence >30 seconds, heart calms to 70 BPM (comfortable with person)

7. **Testing:**
   - Launch enhanced coordinated expression node
   - **Emotion Tests (all 7 emotions):**
     - Publish "happy": Eyes brighten, ears up, head tilts, **heart beats fast in bright red**
     - Publish "sad": Eyes droop, ears down, head lowers, **heart beats slow in dark blue**
     - Publish "curious": Eyes widen, ears forward, head rolls, **heart beats medium in orange**
     - Verify visual coherence: Heart color matches emotional tone
   - **Presence Test:**
     - Approach sensor: Curious expression + **heart BPM 90**
     - Stay 30 seconds: Heart BPM drops to 70
     - Leave: Heart returns to 60 BPM idle
   - **Multi-Modal Sync:**
     - Verify all four modalities change simultaneously (<50ms variance)
     - 50 emotion cycles without desynchronization

8. **End-to-End Validation:**
   - Full chain: Expression command → Coordinated expression node → 4 drivers (head, ears/neck, body) → I2C → 3 ESP32s → eyes, servos, heart LCD
   - Latency: Expression command to all visible changes <300ms
   - Reliability: 100 expression changes without failure
   - Visual inspection: Four modalities create coherent emotional display

9. **Code Quality:**
   - File: `coordinated_expression.py` updated
   - Emotion mapping config: YAML or Python dict with heart parameters
   - Comments explain color psychology, BPM-emotion mapping
   - Code committed to `ros2/src/orchestrator/ros2_nodes/`

**Dependencies:** Story 5.6 (Body driver), Epic 3 Story 3.9 (coordinated expression base)

**Estimated Effort:** 4-6 hours

**Note:** This completes 4-modality expression system. Epic 9 (Super Expressive System) will expand to 7 modalities (add sounds, LEDs, body lean).

---

## Story 5.8: Power System Integration Testing & Validation

**As a** quality assurance tester,
**I want** complete power system validated with load testing and fail-safe verification,
**so that** Epic 5 power infrastructure is safe and reliable for full robot operation.

### Acceptance Criteria:

1. **Test Setup:**
   - All hardware from Epic 5 assembled:
     - 36V battery with BMS
     - Buck converters (12V, 5V)
     - Power distribution board with fuses and switches
     - Raspberry Pi running ROS2 orchestrator
     - Body Module (ESP32-S3 + heart LCD)
   - All hardware from Epic 3 connected:
     - Head Module (ESP32 + eyes + presence sensor)
     - Ears+Neck Module (ESP32-S3 + 7 servos)
   - Test equipment:
     - Multimeter (voltage/current measurement)
     - IR thermometer or thermocouples (temperature monitoring)
     - Power analyzer or clamp meter (current draw measurement)
     - Stopwatch/timer

2. **Voltage Regulation Testing:**
   - **5V Rail Under Load:**
     - No load: 5.00V ±0.05V
     - Light load (Pi idle): 5.00V ±0.05V
     - Medium load (Pi + 3 ESP32s idle): 4.95-5.05V
     - Heavy load (Pi + 3 ESP32s + 7 servos moving): 4.90-5.10V
     - Peak load (all servos + heart animation + ROS2): >4.85V minimum
   - **12V Rail:**
     - No load: 12.00V ±0.2V
     - Light load (test resistor 1A): 11.9-12.1V
     - Ready for Epic 6 motor integration
   - **36V Battery:**
     - Full charge: 41.5-42.0V
     - 50% charge: 35-37V
     - Critical cutoff: 30.0V (triggers warning)

3. **Current Draw Measurement:**
   - **Idle State (all powered, no activity):**
     - Raspberry Pi: ~0.8-1.2A @ 5V
     - ESP32 Head: ~0.15A @ 5V
     - ESP32 Ears+Neck: ~0.2A @ 5V
     - ESP32 Body: ~0.2A @ 5V
     - Total idle 5V: ~1.5-2.0A
   - **Active State (Epic 3 expressions + heart animation):**
     - Total 5V: 3-5A (includes servo movement)
     - Peak 5V: <8A (all 7 servos + displays active)
   - **Battery Life Calculation:**
     - Battery capacity: 4.4Ah @ 36V = 158.4Wh
     - Average power draw (5V rail @ 3A): 15W
     - Estimated runtime: 158.4Wh / 15W ≈ 10 hours (conservative, includes losses)
     - Under heavy use (5A continuous): ~6 hours

4. **Thermal Testing (60-Minute Continuous Operation):**
   - **Test Procedure:**
     - Launch all ROS2 nodes (Epic 3 + Epic 5)
     - Run coordinated expressions: Cycle all 7 emotions every 45 seconds (80 cycles total)
     - Heart animation: Continuous beating throughout test
     - Servo activity: Include gestures every 3 minutes (20 gestures total)
     - Monitor temperatures every 10 minutes
   - **Temperature Targets:**
     - Buck converters (36V→5V, 36V→12V): <65°C
     - Battery pack: <45°C (lithium safe limit)
     - Raspberry Pi SoC: <70°C (check `vcgencmd measure_temp`)
     - ESP32 modules: <60°C
     - Power wiring: No warm spots (indicates high-resistance joints)
   - **Thermal Shutdown Test:**
     - If converter exceeds 70°C: Verify thermal protection engages (output cuts off)
     - Allow cooling, verify automatic recovery

5. **Fuse Protection Testing:**
   - **5V Rail Fuse Test:**
     - Create controlled short circuit on 5V rail
     - Verify 10A fuse blows before damage to converter
     - Replace fuse, verify system recovers
   - **12V Rail Fuse Test:**
     - Same procedure for 12V rail
   - **Fuse Response Time:**
     - Fast-blow fuses: <100ms trip time (measure with oscilloscope if available)

6. **Emergency Cutoff Testing:**
   - **Emergency Button Test:**
     - System running normally (all modules active)
     - Press emergency cutoff button: All power rails immediately disconnect
     - Verify: Raspberry Pi shuts down (may not be graceful - power loss)
     - Release button: Power restored, system reboots
   - **Graceful Shutdown Procedure (Future Enhancement):**
     - Note: Emergency cutoff is hardware-based (immediate, not graceful)
     - Software graceful shutdown implemented in Epic 13 (AI agent monitors battery)

7. **Battery Discharge Testing:**
   - **Full Discharge Cycle:**
     - Start with full battery (42V)
     - Run system continuously with moderate load (3A @ 5V)
     - Monitor voltage every 30 minutes
     - ROS2 battery monitoring publishes voltage and percentage
     - **Discharge Curve:**
       - 42V (100%) → 39V (75%) → 36V (50%) → 33V (25%) → 30V (0%)
       - Verify low battery warning triggers at 33V
       - **CRITICAL:** Manually shut down at 30V (do not over-discharge)
   - **BMS Protection Verification:**
     - BMS should cut output at ~28V (protection threshold)
     - Verify BMS prevents over-discharge damage

8. **Charging Testing:**
   - **Charge Cycle:**
     - Connect 36V charger to barrel jack
     - Verify BMS accepts charge (voltage monitor shows rising voltage)
     - Charge from 50% (36V) to 100% (42V)
     - Measure time: Expected ~2 hours @ 2A charger
     - Verify BMS cuts off charging at 42V (overcharge protection)
   - **Simultaneous Charge/Operation:**
     - System running (low load) while charging
     - Verify safe: BMS allows charge + discharge simultaneously
     - Battery voltage increases while system operates

9. **ROS2 Power Monitoring Validation:**
   - `/olaf/power/battery_voltage` topic:
     - Publishes actual battery voltage (verify with multimeter, <0.5V error)
     - Update rate: 1Hz
   - `/olaf/power/battery_percentage` topic:
     - Publishes 0-100% based on voltage
     - 42V → 100%, 36V → 50%, 30V → 0%
   - Warning topics:
     - Discharge to 33V: `/olaf/warnings/low_battery` published
     - Discharge to 30V: `/olaf/warnings/critical_battery` published

10. **Pass/Fail Criteria:**
    - **PASS Requirements:**
      - All voltage rails stable under load (within specified tolerances)
      - No component overheating (all temps <75°C)
      - Fuses protect against shorts (verified with test)
      - Emergency cutoff immediately disconnects power
      - Battery lasts >5 hours under realistic use
      - ROS2 monitoring accurate (voltage within 0.5V of actual)
      - No electrical failures during 60-minute test
    - **FAIL Triggers:**
      - Voltage sag >10% under load
      - Any component >80°C
      - Fuse fails to protect against short
      - Emergency cutoff malfunction
      - Battery monitoring inaccurate (>1V error)

11. **Documentation:**
    - Test results: `tests/integration/epic5_power_validation_results.md`
    - Voltage/current measurements table
    - Thermal profile graph (temperature vs. time)
    - Battery discharge curve (voltage vs. time)
    - Photos: Thermal imaging (if available), multimeter readings
    - Video: Emergency cutoff demonstration

**Dependencies:** All Epic 5 stories (5.1-5.7), Epic 3 complete (for load testing)

**Estimated Effort:** 3-4 hours test execution + 2-3 hours documentation + 4-8 hours debugging if issues found

**Critical Safety Note:** This is the Epic 5 safety gate. Must pass before Epic 6 (adds high-current motors). Lithium battery failures can be dangerous - verify all BMS protections work.

---

## Story 5.9: Epic 5 Documentation & Build-in-Public Content

**As a** future contributor or follower,
**I want** comprehensive Epic 5 documentation and build-in-public content,
**so that** I can replicate the torso/power system and follow the journey.

### Acceptance Criteria:

1. **Technical Documentation:**
   - `firmware/body/README.md` created:
     - ST77916 QSPI display integration
     - Heart animation engine architecture
     - I2C register map (0x10-0x21)
     - Firmware build/upload process
     - BPM-to-animation-period formula
   - `hardware/power-system/README.md` created:
     - 36V battery specifications and safety
     - Buck converter configuration (output voltage settings)
     - Power distribution board schematic
     - Fuse ratings and replacement procedure
     - Emergency cutoff wiring diagram
     - Charging procedure and BMS care
   - `ros2/src/orchestrator/ros2_nodes/README.md` updated:
     - Body driver usage
     - Power monitoring topics
     - Battery warning thresholds
     - 4-modality coordinated expression API

2. **Hardware Documentation:**
   - BOM: `hardware/bom/epic5_bom.csv`
     - ESP32-S3 (Body module)
     - 1.53" Round TFT 360×360 ST77916 QSPI display
     - 36V hoverboard battery (10S2P 18650)
     - Buck converters (36V→12V, 36V→5V)
     - BMS (integrated with battery)
     - Fuses, switches, connectors (XT60, Anderson Powerpole)
     - 36V charger (42V max, 2A)
     - Voltage monitor module
     - 14AWG/16AWG wire, perfboard/PCB
     - 3D printed parts list with print times
     - Supplier links, costs
   - Wiring diagrams:
     - `hardware/wiring/body-module-wiring.png` (ESP32 + heart LCD QSPI)
     - `hardware/wiring/power-distribution-schematic.png` (complete power system)
     - `hardware/wiring/power-safety-diagram.png` (fuses, emergency cutoff, BMS)
     - `hardware/wiring/torso-complete-assembly.png` (Pi + Body Module + power)

3. **Assembly Guides:**
   - `hardware/assembly/torso-power-assembly-guide.md`:
     - Step-by-step instructions with photos
     - 3D printing tips for torso/power enclosure
     - Buck converter output voltage adjustment procedure
     - Battery safety guidelines (handling, storage, charging)
     - Cable management best practices
     - Power system testing sequence (voltage checks before connecting loads)
     - Troubleshooting common issues

4. **Photos/Videos:**
   - **Photos:**
     - 3D printed torso parts laid out
     - Power distribution board assembly (close-up)
     - Battery installed in enclosure
     - Heart LCD mounted in torso front panel
     - Complete torso assembly (multiple angles)
     - Heart LCD showing different emotions (happy red, sad blue, thinking purple)
     - Voltage monitor displaying battery status
   - **Videos:**
     - 30-second time-lapse of torso assembly
     - 1-minute demo: Heart LCD animation cycling through 7 emotion modes
     - 15-second clip: Coordinated 4-modality expression (eyes+ears+neck+heart)
     - 20-second clip: Power monitoring demo (battery voltage, percentage display)

5. **GitHub Milestone:**
   - Milestone "Epic 5: Core Torso & Power System" created
   - All stories (5.1-5.9) closed and linked
   - Milestone marked complete

6. **Build-in-Public Content:**
   - **LinkedIn Post:** "Olaf's Got Heart! ❤️🔋"
     - Photo carousel: Heart LCD in different colors, power system assembly, complete torso
     - Explanation: "Epic 5 complete! Olaf now has a beating heart display (360×360 QSPI LCD) and a robust 36V power system. 10+ hours of battery life, safe BMS protection, and 4-modality emotional expression (eyes+ears+neck+heart). Power infrastructure ready for self-balancing in Epic 6!"
     - Technical depth: QSPI display challenge, buck converter thermal testing, BMS safety features
     - GitHub link to Epic 5 documentation
   - **Optional Twitter/X Thread:**
     - Tweet 1: "Built a beating heart for my robot! 1.53\" round LCD (360×360) with QSPI interface on ESP32-S3. 60 FPS heart animation synchronized with emotions. ❤️🤖 [GIF: heart changing colors/speeds]"
     - Tweet 2: "Power system: 36V hoverboard battery, dual buck converters, BMS protection, ~10 hour runtime. Emergency cutoff tested! 🔋⚡ [Photo: power distribution board]"
     - Tweet 3: "4-modality emotional expression now complete: Eyes (manga-style), Ears (Doberman positioning), Neck (WALL-E head tilts), Heart (color+BPM). Check out docs: [GitHub link]"

7. **Troubleshooting Guide:**
   - `docs/troubleshooting-epic5.md`:
     - "Heart LCD not displaying" → Check QSPI wiring (GPIO10-18), verify PSRAM enabled
     - "Buck converter overheating" → Reduce load, verify heatsink installed, check ventilation
     - "Battery not charging" → Verify charger voltage (42V), check BMS balance leads
     - "5V rail voltage sag under load" → Check converter current rating, verify wiring gauge (16AWG minimum)
     - "Emergency cutoff doesn't work" → Test normally-closed button circuit with multimeter
     - "ROS2 battery voltage inaccurate" → Calibrate ADC or I2C sensor, verify voltage divider resistor values
     - "Body Module I2C 0x0A not detected" → Check I2C wiring, verify address in firmware, scan bus

**Dependencies:** All Epic 5 stories complete (5.1-5.8)

**Estimated Effort:** 8-10 hours (documentation + photos/videos + content)

---

## Epic 5 Summary

**Total Stories:** 9

**Estimated Total Effort:** 73-102 hours (2-3 weeks for solo builder)

**Key Deliverables:**
- ✅ Complete 3D-printed core torso (panels, power enclosure, component bays)
- ✅ **Body Module:** ESP32-S3 + 1.53" round TFT heart display (360×360 ST77916 QSPI) at I2C 0x0A
- ✅ Heart animation engine: 4 modes, variable BPM (40-180), HSV color control, 30-60 FPS
- ✅ **Power System:** 36V hoverboard battery + BMS, buck converters (12V, 5V), fuses, emergency cutoff
- ✅ Power distribution: Safe wiring harness, voltage monitoring, charging system
- ✅ Raspberry Pi integration: Mounted in torso, connected to all modules (0x08, 0x09, 0x0A)
- ✅ ROS2 body driver: Heart animation control, power monitoring (voltage, percentage, warnings)
- ✅ Coordinated expression enhanced: 4-modality synchronization (eyes+ears+neck+heart)
- ✅ Power system validation: 60-minute thermal testing, load testing, battery discharge testing
- ✅ Complete documentation + build-in-public content

**Architecture Validated:**
- ✅ Body Module I2C integration (3 modules: Head 0x08, Ears+Neck 0x09, Body 0x0A)
- ✅ QSPI high-resolution display (360×360 @ 30-60 FPS)
- ✅ 36V battery architecture with dual voltage rails (12V, 5V)
- ✅ BMS and safety systems (fuses, emergency cutoff, voltage monitoring)
- ✅ 4-modality emotional expression (eyes, ears, neck, heart)
- ✅ ROS2 power monitoring (battery health, warnings)
- ✅ Thermal management under continuous load
- ✅ 10+ hour battery runtime under realistic use

**Success Criteria Met:**
- Power infrastructure robust and safe (thermal testing, fuse protection, BMS validated)
- Heart display creates engaging emotional body language (color+BPM synchronized with emotions)
- Raspberry Pi integrated with all modules (I2C bus functional: 0x08, 0x09, 0x0A)
- Torso housing professional and maintainable (accessible panels, cable management)
- Battery life sufficient for extended operation (10+ hours idle, 6+ hours active)
- Foundation ready for Epic 6 (12V rail prepared for ODrive motor controller)

**Next Epic:** Epic 6: Self-Balancing Base - Complete Build

**Note:** Epic 5 establishes core infrastructure. Epic 6 adds mobility (motors, IMU, self-balancing). Epic 7 integrates all modules into unified structure.

---

**Related Documents:**
- [PRD Epic List](epic-list.md) - Overall project roadmap
- [Epic 1: Foundation](epic-01-foundation.md) - I2C architecture foundation
- [Epic 3: Complete Head Assembly](epic-03-complete-head-assembly.md) - Head/Ears/Neck modules
- [Architecture - Components](../architecture/components.md) - Body module specifications
- [Tech Stack](../architecture/tech-stack.md) - ESP32-S3, ST77916 display, power components
