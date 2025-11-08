# Epic 3: Complete Head Assembly (Head + Ears + Neck)

**Epic Goal:** Complete the full head assembly by 3D printing head housing, ear mounts, and neck gimbal components (with Epic 2 feedback incorporated). Assemble and integrate the complete head system: dual GC9A01 round TFT eyes (240×240), mmWave presence sensor, 2-DOF articulated ears (4× Feetech SCS0009 servos), and 3-DOF neck gimbal (pan ±90°, tilt ±45°, roll ±30° with 3× Feetech STS3215 servos). Implement coordinated kinematics engine for synchronized ear+neck gestures and enhanced ESP32 firmware for expressive multi-modal emotional displays.

**Duration:** 3-4 weeks (Weeks 7-10)

**Prerequisites:**
- Epic 1 (Foundation & I2C Communication) - Head Module eyes validated
- Epic 2 (Complete OnShape Design & Community Feedback) - All STLs exported and feedback incorporated

**Value Delivered:** Complete expressive head assembly with full upper-body mobility, coordinated emotional expression across eyes+ears+neck, presence detection capability, professional mechanical integration. Validates the multi-modal expressiveness concept before body and base integration. Creates "Olaf comes alive with personality!" moment with synchronized gestures.

**Architecture Focus:** This epic proves the coordinated multi-modal expression system with synchronized control across three physical systems (eyes, ears, neck). Demonstrates the Ears+Neck Module (I2C 0x09) controlling 7 servos via dual bus servo controllers. Validates kinematics engine for natural head movement and emotional gesture coordination.

**Hardware Note:** Head Module (eyes + presence) uses ESP32 from Epic 1. New Ears+Neck Module requires dedicated ESP32-S3 with dual UART connections for servo controllers.

---

## Story 3.1: 3D Print Head Housing, Ear Mounts & Neck Gimbal Components

**As a** hardware builder,
**I want** all head assembly components 3D printed with Epic 2 feedback incorporated,
**so that** I have all physical parts ready for assembly.

### Acceptance Criteria:

1. **STL Files Verified:**
   - Head housing STLs from Epic 2 design downloaded from OnShape
   - Files reviewed for completeness:
     - `head-housing-main.stl` (main head shell)
     - `head-housing-back.stl` (back panel with cable access)
     - `head-front-panel.stl` (front panel with eye cutouts)
     - `eye-mount-left.stl`, `eye-mount-right.stl` (GC9A01 display mounts)
     - `presence-sensor-mount.stl` (mmWave sensor bracket)
     - `ear-base-left.stl`, `ear-base-right.stl` (ear servo mounts)
     - `ear-fin-left.stl`, `ear-fin-right.stl` (visible ear components)
     - `neck-gimbal-base.stl` (bottom plate, mounts to torso)
     - `neck-gimbal-pan.stl` (pan axis rotating component)
     - `neck-gimbal-tilt.stl` (tilt axis component)
     - `neck-gimbal-roll.stl` (roll axis, connects to head)
     - `servo-bracket-neck-pan.stl`, `servo-bracket-neck-tilt.stl`, `servo-bracket-neck-roll.stl`
     - `cable-guide-neck.stl` (cable management)

2. **3D Printing Settings:**
   - Material: PLA or PETG (PETG recommended for strength)
   - Layer height: 0.2mm (0.15mm for eye mounts for precision)
   - Infill: 20% for housing, 30% for structural parts (gimbal, servo brackets)
   - Supports: Auto-generated where needed (eye mounts, overhangs)
   - Print orientation optimized for strength and minimal supports

3. **Printing Execution:**
   - All components printed successfully
   - No warping, layer separation, or print failures
   - Support material removed cleanly
   - Screw holes drilled/cleaned to proper size (M3 for servo mounting, M4 for gimbal joints)

4. **Quality Verification:**
   - Eye display cutouts fit GC9A01 displays (1.28" diameter)
   - Presence sensor mount fits DFRobot SEN0395 dimensions
   - Servo brackets fit Feetech SCS0009 (ears) and STS3215 (neck) mounting holes
   - Gimbal joints move smoothly without binding
   - Cable routing channels clear and accessible

5. **Post-Processing:**
   - Support scars sanded smooth
   - Functional surfaces (servo mounts, gimbal pivots) checked for fit
   - Test-fit major components before final assembly

6. **Documentation:**
   - Print time logged: `hardware/3d-models/print-log.md`
   - Photos of printed parts: `hardware/3d-models/Head/assembly-parts-photos/`
   - Any print issues noted for future reprints

**Dependencies:** Epic 2 complete (STLs exported, community feedback incorporated)

**Estimated Effort:** 8-12 hours print time (spread over 2-3 days) + 2-3 hours post-processing

**Note:** Some components (neck gimbal) may require multiple print batches. Plan printing schedule to avoid blocking assembly work.

---

## Story 3.2: Head Module Assembly - Eyes & Presence Sensor Integration

**As a** hardware builder,
**I want** the Head Module from Epic 1 integrated into the 3D-printed head housing with presence sensor added,
**so that** I have a complete head unit with eyes and presence detection.

### Acceptance Criteria:

1. **Component Preparation:**
   - ESP32 Head Module from Epic 1 (with dual GC9A01 eyes working)
   - DFRobot SEN0395 mmWave presence sensor acquired
   - M3 screws/standoffs for mounting (8× M3×8mm screws, 4× M3×10mm standoffs)
   - Wire management: 20cm jumper wires for sensor connections

2. **Eye Display Mounting:**
   - Left GC9A01 display mounted in `eye-mount-left` bracket
   - Right GC9A01 display mounted in `eye-mount-right` bracket
   - Eye mounts secured to `head-front-panel.stl` with M3 screws
   - Displays aligned: Both eyes level, centered in cutouts, parallel to each other
   - SPI wiring from Epic 1 maintained (minimal wire length to ESP32)

3. **Presence Sensor Integration:**
   - DFRobot SEN0395 mounted in `presence-sensor-mount.stl` bracket
   - Sensor positioned: Forward-facing, centered above/between eyes
   - Wiring to ESP32 GPIO:
     - VCC → 3.3V (ESP32)
     - GND → GND
     - OUT → GPIO16 (presence detection digital output)
     - Optional: RX/TX → GPIO17/GPIO18 for advanced configuration (UART)

4. **ESP32 Mounting:**
   - ESP32-S3 secured inside head housing (rear compartment)
   - Access to USB port for firmware updates maintained
   - SPI wiring to both eyes organized and secured (no loose wires)
   - I2C wiring (to Raspberry Pi) routed through neck cable channel

5. **Head Housing Assembly:**
   - `head-front-panel.stl` attached to `head-housing-main.stl`
   - `head-housing-back.stl` secured (removable for maintenance)
   - Cable access points clear for neck routing
   - Structural integrity: No flex, all panels secure

6. **Functional Testing:**
   - Power on: ESP32 boots, both eyes display neutral expression (Epic 1 firmware)
   - Presence sensor: Digital output toggles when presence detected (verified with multimeter or serial monitor)
   - I2C communication: `sudo i2cdetect -y 1` shows device at 0x08
   - Epic 1 expression test: `olaf-test head expression 1 3` → eyes show happy expression

7. **Documentation:**
   - Wiring photos: `hardware/wiring/head-module-v2-assembly.jpg`
   - Presence sensor pinout documented in `firmware/head/README.md`
   - Updated component locations in head housing

**Dependencies:** Story 3.1 (head parts printed), Epic 1 (Head Module working)

**Estimated Effort:** 4-6 hours

**Note:** This story integrates existing Epic 1 Head Module into physical housing. Presence sensor firmware integration happens in Story 3.6.

---

## Story 3.3: Ear Servo Assembly & Mechanical Integration

**As a** hardware builder,
**I want** 2-DOF articulated ears assembled with servo mechanisms,
**so that** Olaf can express emotions through ear positioning (Doberman-inspired expressiveness).

### Acceptance Criteria:

1. **Components Acquired:**
   - 4× Feetech SCS0009 servos (serial bus servos for ear articulation)
   - 1× Bus Servo Controller (STSC series, controls 4 servos via UART)
   - M2/M3 screws for servo mounting
   - Servo horns/brackets (included with servos)

2. **Left Ear Assembly (2-DOF):**
   - **Horizontal Servo (Base):**
     - Servo 1 mounted in `ear-base-left.stl` bracket
     - Servo controls horizontal ear rotation (forward ↔ back, ±60°)
     - Bracket secured to left side of head housing (M3 screws)
   - **Vertical Servo (Fin):**
     - Servo 2 mounted to Servo 1 horn
     - Servo controls vertical ear orientation (up ↔ down, ±45°)
     - `ear-fin-left.stl` attached to Servo 2 horn (visible ear component)
   - **Range of Motion:**
     - Combined movement allows ear to point forward-up (alert), back-down (relaxed), or any position in between
     - No mechanical binding through full range
     - Smooth movement, no gear stripping

3. **Right Ear Assembly (2-DOF):**
   - Mirror configuration of left ear
   - Servo 3 (horizontal base) + Servo 4 (vertical fin)
   - `ear-base-right.stl` + `ear-fin-right.stl`
   - Range of motion matches left ear (symmetrical)

4. **Servo Controller Wiring:**
   - Bus Servo Controller connected to all 4 servos via daisy-chain serial bus:
     - Servo 1 (left-horizontal) → Servo 2 (left-vertical) → Servo 3 (right-horizontal) → Servo 4 (right-vertical)
   - Power: 5V supply to servo controller (from buck converter or bench supply for testing)
   - Controller UART interface wired (TX/RX) to Ears+Neck ESP32 (Story 3.4 provides ESP32)

5. **Servo Addressing:**
   - Servo 1: ID = 1 (left ear horizontal)
   - Servo 2: ID = 2 (left ear vertical)
   - Servo 3: ID = 3 (right ear horizontal)
   - Servo 4: ID = 4 (right ear vertical)
   - IDs configured via manufacturer software or test script

6. **Mechanical Testing:**
   - Manual test: Send UART commands to controller, verify all 4 servos respond
   - Test ear positions:
     - Alert: Both ears forward, up 30°
     - Relaxed: Both ears back, horizontal
     - Curious (left): Left ear forward-up, right ear horizontal
     - Asymmetric test: Each ear moves independently
   - No mechanical interference with head housing or neck assembly
   - Stable mounting: No wobble or servo slippage

7. **Documentation:**
   - Ear assembly photos: `hardware/wiring/ear-assembly-photos/`
   - Servo ID mapping documented: `firmware/ears_neck/README.md`
   - Mechanical range of motion specifications logged

**Dependencies:** Story 3.1 (ear parts printed)

**Estimated Effort:** 5-7 hours

**Note:** Servo controller will be connected to Ears+Neck ESP32 in Story 3.4. This story focuses on mechanical assembly and servo ID configuration.

---

## Story 3.4: Neck Gimbal Assembly & Mechanical Integration

**As a** hardware builder,
**I want** a 3-DOF neck gimbal assembled with servo actuators,
**so that** Olaf's head can pan, tilt, and roll for expressive head gestures.

### Acceptance Criteria:

1. **Components Acquired:**
   - 3× Feetech STS3215 servos (30 kg·cm torque for neck articulation)
   - 1× Bus Servo Controller (STSC series, controls 3 neck servos via UART)
   - 1× ESP32-S3-WROOM-2 (N16R8) - Ears+Neck Module controller
   - M4 bolts for gimbal joints, M3 screws for servo mounting
   - Bearings (optional, for smoother gimbal rotation)

2. **3-DOF Gimbal Kinematics:**
   - **Pan Axis (Yaw):** Rotates head left ↔ right (±90°)
   - **Tilt Axis (Pitch):** Tilts head up ↔ down (±45°)
   - **Roll Axis:** Rolls head clockwise ↔ counterclockwise (±30°)
   - Gimbal order: Pan (base) → Tilt (middle) → Roll (top, connects to head)

3. **Pan Servo Assembly:**
   - Servo 5 mounted in `servo-bracket-neck-pan.stl`
   - Servo output drives `neck-gimbal-pan.stl` component
   - `neck-gimbal-base.stl` secured to torso mounting point (will connect to torso in Epic 7)
   - Pan range: -90° (left) to +90° (right), center = 0° (forward)

4. **Tilt Servo Assembly:**
   - Servo 6 mounted in `servo-bracket-neck-tilt.stl` (attached to pan component)
   - Servo output drives `neck-gimbal-tilt.stl` component
   - Tilt range: -45° (down) to +45° (up), center = 0° (level)

5. **Roll Servo Assembly:**
   - Servo 7 mounted in `servo-bracket-neck-roll.stl` (attached to tilt component)
   - Servo output drives `neck-gimbal-roll.stl` component
   - `neck-gimbal-roll.stl` connects to head housing bottom
   - Roll range: -30° (CCW) to +30° (CW), center = 0° (upright)

6. **Head Attachment:**
   - Head housing from Story 3.2 mounted to neck-gimbal-roll component
   - M3 screws secure head to roll gimbal
   - Weight distribution balanced (head doesn't sag under own weight when servos unpowered)

7. **Cable Routing:**
   - I2C cable from head ESP32 routed through neck gimbal cable channels
   - Cables organized to avoid pinching during full range of motion (use `cable-guide-neck.stl`)
   - Presence sensor and eye wiring secured inside head housing

8. **Servo Controller & ESP32 Integration:**
   - Bus Servo Controller (neck servos) mounted near gimbal base
   - Neck servos daisy-chained: Servo 5 → Servo 6 → Servo 7
   - Servo addressing:
     - Servo 5: ID = 5 (pan)
     - Servo 6: ID = 6 (tilt)
     - Servo 7: ID = 7 (roll)
   - ESP32-S3 (Ears+Neck Module) positioned near gimbal base
   - **Dual UART Wiring:**
     - UART1 (GPIO4/GPIO5): Connected to ear servo controller (Servos 1-4 from Story 3.3)
     - UART2 (GPIO16/GPIO17): Connected to neck servo controller (Servos 5-7)
   - **I2C Wiring to Raspberry Pi:**
     - ESP32 GPIO21 (SDA) → Pi GPIO2 (SDA)
     - ESP32 GPIO22 (SCL) → Pi GPIO3 (SCL)
     - I2C address configured: **0x09** (Ears+Neck Module)
     - Common ground: ESP32 GND ↔ Pi GND

9. **Mechanical Testing:**
   - Manual test: UART commands to neck controller verify all 3 servos respond
   - Test movements:
     - Pan sweep: -90° to +90° (smooth, no binding)
     - Tilt sweep: -45° to +45° (head doesn't sag)
     - Roll sweep: -30° to +30° (balanced)
     - Combined: Pan + tilt + roll simultaneously (no mechanical interference)
   - Full range tested with head assembly weight
   - Cables remain clear of moving parts throughout full range

10. **Power Testing:**
    - Servo controller powered from 5V buck converter (or bench supply)
    - ESP32-S3 powered from 5V (shared or separate supply)
    - No voltage sag when all 7 servos (ears + neck) move simultaneously
    - Current draw measured: <3A peak with all servos active

11. **Documentation:**
    - Neck gimbal assembly photos: `hardware/wiring/neck-gimbal-assembly-photos/`
    - Servo ID mapping: `firmware/ears_neck/README.md` updated with neck servos
    - Gimbal kinematics diagram: Forward/inverse kinematics reference
    - Cable routing documented

**Dependencies:** Story 3.2 (head assembled), Story 3.3 (ears assembled)

**Estimated Effort:** 8-10 hours

**Note:** This story completes the physical assembly of the full head system. Firmware for coordinated control happens in Story 3.5.

---

## Story 3.5: Ears+Neck ESP32 Firmware - Servo Control & Kinematics

**As a** firmware developer,
**I want** ESP32 firmware controlling 7 servos (4 ears + 3 neck) with coordinated kinematics,
**so that** the orchestrator can command natural head gestures and ear expressions.

### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO project: `firmware/ears_neck/firmware/`
   - Platform: ESP32-S3 (Arduino framework)
   - Libraries: `Wire.h` (I2C), `HardwareSerial.h` (UART), Feetech servo library (SCS/STS compatible)

2. **I2C Slave Implementation:**
   - ESP32 configured as I2C slave at address **0x09**
   - I2C receive/request handlers implemented
   - Interrupt-driven I2C communication

3. **I2C Register Map (Ears+Neck Module 0x09):**
   - `0x00`: Module ID (returns 0x09)
   - `0x02`: Status byte (READY/BUSY/ERROR)
   - **Ear Control Registers:**
     - `0x10`: Left ear horizontal position (0-180°, maps to -60° to +60°)
     - `0x11`: Left ear vertical position (0-180°, maps to -45° to +45°)
     - `0x12`: Right ear horizontal position (0-180°, maps to -60° to +60°)
     - `0x13`: Right ear vertical position (0-180°, maps to -45° to +45°)
   - **Neck Control Registers:**
     - `0x20`: Neck pan position (0-180°, maps to -90° to +90°)
     - `0x21`: Neck tilt position (0-180°, maps to -45° to +45°)
     - `0x22`: Neck roll position (0-180°, maps to -30° to +30°)
   - **Coordinated Gesture Register:**
     - `0x30`: Gesture ID (0=neutral, 1=look-left, 2=look-right, 3=nod-yes, 4=shake-no, 5=tilt-curious, 6=alert-ears-up)
     - `0x31`: Gesture speed (1=slow, 5=fast)
     - `0x32`: Gesture trigger (write any value to start gesture)

4. **Dual UART Servo Communication:**
   - **UART1 (GPIO4 TX, GPIO5 RX):** Ear servo controller (Servos 1-4)
   - **UART2 (GPIO16 TX, GPIO17 RX):** Neck servo controller (Servos 5-7)
   - Servo library initialized for both buses
   - Position commands sent to servos in degrees (0-180°)

5. **Kinematics Engine:**
   - **Ear Positioning:**
     - Independent control: Each ear's 2-DOF position calculated from horizontal/vertical inputs
     - Coordinated modes: Both ears mirror (symmetrical) or asymmetric (one forward, one back)
   - **Neck Kinematics:**
     - Forward kinematics: Pan, tilt, roll → final head orientation
     - Inverse kinematics: Not required (direct servo control sufficient)
     - Combined movements: Smooth interpolation when multiple axes change simultaneously

6. **Coordinated Gestures:**
   - Predefined gesture sequences combining ears + neck:
     - **look-left:** Pan -60°, left ear forward, right ear back
     - **look-right:** Pan +60°, right ear forward, left ear back
     - **nod-yes:** Tilt down 20° → up 10° → center (repeat 2×)
     - **shake-no:** Pan left 30° → right 30° → center (repeat 2×)
     - **tilt-curious:** Roll 15°, tilt up 20°, ears asymmetric
     - **alert-ears-up:** Both ears vertical +45°, neck level
   - Gestures execute asynchronously (non-blocking I2C communication)
   - Status register shows BUSY during gesture, READY when complete

7. **Motion Profiles:**
   - Smooth acceleration/deceleration (easing functions)
   - Speed control: Gesture speed register (1-5) adjusts movement duration (500ms-2s)
   - Simultaneous servo updates: All servos start movement together for coordinated gestures

8. **Testing:**
   - Firmware compiled and uploaded to ESP32-S3
   - Serial: "I2C Slave initialized at 0x09 - Ears+Neck Module"
   - I2C scan: `sudo i2cdetect -y 1` shows device at 0x09
   - Manual ear test: Write to 0x10-0x13 → ears move to positions
   - Manual neck test: Write to 0x20-0x22 → neck moves to positions
   - Gesture test: Write gesture ID 1, trigger 0x32 → executes look-left gesture
   - All 7 servos responsive, smooth motion, no jitter

9. **Performance:**
   - I2C latency: <10ms from command received to servo movement started
   - Gesture execution time: 500ms-2s (speed-dependent)
   - Memory stable over 30 minutes continuous operation
   - No servo overheating (thermal check after 10 minutes of movement)

10. **Code Quality:**
    - Files:
      - `main.cpp`
      - `i2c_slave.cpp`
      - `ear_servo_controller.cpp` (UART1, servos 1-4)
      - `neck_servo_controller.cpp` (UART2, servos 5-7)
      - `kinematics.cpp` (coordinate transformations)
      - `coordinated_gestures.cpp` (predefined gesture sequences)
      - `motion_profiles.cpp` (easing functions)
    - Comments explain dual-UART architecture and gesture coordination
    - Code committed to `firmware/ears_neck/firmware/`

**Dependencies:** Story 3.3 (ears assembled), Story 3.4 (neck assembled)

**Estimated Effort:** 8-12 hours

**Note:** This firmware controls ALL upper-body articulation (ears + neck). Coordinated with Head Module (eyes) via ROS2 orchestration layer.

---

## Story 3.6: Head Module ESP32 Firmware Enhancement - Presence Sensor Integration

**As a** firmware developer,
**I want** Epic 1 Head Module firmware enhanced with mmWave presence sensor integration,
**so that** Olaf can detect human presence and react accordingly.

### Acceptance Criteria:

1. **Existing Firmware Base:**
   - Start from Epic 1 `firmware/head/firmware/main.cpp`
   - Existing dual-eye expression engine remains functional
   - I2C slave at address 0x08 remains operational

2. **Presence Sensor Driver:**
   - DFRobot SEN0395 library integrated or custom driver implemented
   - GPIO16 configured for digital presence detection (HIGH = presence, LOW = no presence)
   - Optional: UART configuration for advanced settings (detection range, sensitivity)

3. **I2C Register Map Enhancement (Head Module 0x08):**
   - Existing registers from Epic 1 maintained:
     - `0x00`: Module ID (0x08)
     - `0x02`: Status
     - `0x10`: Expression type
     - `0x11`: Expression intensity
     - `0x12`: Blink trigger
   - **New presence detection registers:**
     - `0x20`: Presence status (0=none, 1=detected)
     - `0x21`: Presence duration (seconds, 0-255)
     - `0x22`: Detection event count (increments on new presence)

4. **Presence Detection Logic:**
   - Poll sensor at 20Hz (50ms interval)
   - Debouncing: Presence must be stable for 200ms to register as detected
   - Duration tracking: Count seconds while presence continuously detected
   - Event counting: Increment counter on rising edge (absence → presence transition)

5. **Autonomous Reactions (Optional Enhancement):**
   - When presence detected after absence: Trigger automatic blink (eyes respond to person entering)
   - Configurable via register 0x23 (autonomous mode: 0=off, 1=on)
   - This creates "awareness" behavior without orchestrator command

6. **Testing:**
   - Firmware compiled and uploaded
   - Serial: "Presence sensor initialized on GPIO16"
   - No presence: Register 0x20 reads 0
   - Person approaches: Register 0x20 reads 1, duration increments
   - Person leaves: Presence status returns to 0, event count incremented
   - Existing eye expressions still functional (Epic 1 tests pass)
   - I2C communication stable with new registers

7. **Integration Testing:**
   - Combined test: Send expression command while presence detected → both work simultaneously
   - Autonomous blink test: Enable 0x23, approach sensor → eyes blink automatically
   - Performance: 60 FPS eye animation maintained while presence polling runs

8. **Code Quality:**
   - Files:
     - `main.cpp` (updated with presence sensor)
     - `presence_sensor.cpp` (new driver)
     - Existing: `eye_expression.cpp`, `gc9a01_driver_spi.cpp`, `i2c_slave.cpp`
   - Comments explain presence detection logic and debouncing
   - Code committed to `firmware/head/firmware/`

**Dependencies:** Story 3.2 (presence sensor physically mounted), Epic 1 (existing head firmware)

**Estimated Effort:** 4-6 hours

**Note:** This enhances Epic 1 Head Module firmware. Eyes + presence sensor both operate on single ESP32 at I2C 0x08.

---

## Story 3.7: ROS2 Ears+Neck Driver Node

**As a** software developer,
**I want** a ROS2 driver node translating ROS2 topics into I2C commands for the Ears+Neck Module,
**so that** the orchestrator can control upper-body articulation without I2C details.

### Acceptance Criteria:

1. **Node Structure:**
   - Python ROS2 node: `ros2/src/orchestrator/ros2_nodes/hardware_drivers/ears_neck_driver.py`
   - Node name: `/olaf/ears_neck_driver`
   - Uses `rclpy` and `smbus2`

2. **I2C Communication:**
   - Opens I2C bus 1: `bus = SMBus(1)`
   - Helper functions for register read/write to address **0x09**

3. **ROS2 Subscriptions:**
   - `/olaf/ears/position` (custom msg: left_h, left_v, right_h, right_v)
     - Callback writes to registers 0x10-0x13 (ear positions)
   - `/olaf/neck/position` (custom msg: pan, tilt, roll)
     - Callback writes to registers 0x20-0x22 (neck positions)
   - `/olaf/gesture` (custom msg: gesture_id, speed)
     - Callback writes to registers 0x30-0x32 (coordinated gesture)

4. **ROS2 Publications:**
   - `/olaf/ears_neck/status` (std_msgs/String) at 10Hz
     - Reads status register 0x02, publishes "READY", "BUSY", or "ERROR"
   - `/olaf/ears_neck/current_position` (custom msg) at 5Hz
     - Reads registers 0x10-0x13, 0x20-0x22, publishes current positions

5. **Module Health Check:**
   - On startup: Read register 0x00, verify response is 0x09
   - If not responding: Log error, publish ERROR status
   - Periodic health check: Every 30s verify module responsive

6. **Error Handling:**
   - I2C timeout handling (module busy during gesture execution)
   - Retry logic: 3 attempts with 100ms delay
   - Persistent failure: Publish ERROR, log details

7. **Testing:**
   - Node launches: `ros2 run orchestrator ears_neck_driver`
   - Topics appear: `/olaf/ears/position`, `/olaf/neck/position`, `/olaf/gesture`
   - Manual test: Publish ear position → ears move
   - Manual test: Publish neck position → head pans/tilts/rolls
   - Manual test: Publish gesture 1 → look-left gesture executes

**Dependencies:** Story 3.5 (Ears+Neck firmware)

**Estimated Effort:** 6-8 hours

**Note:** This driver abstracts I2C complexity. Orchestrator sends high-level commands (positions, gestures) via ROS2 topics.

---

## Story 3.8: ROS2 Head Driver Enhancement - Presence Detection

**As a** software developer,
**I want** Epic 1 ROS2 head driver enhanced with presence detection topic publication,
**so that** the orchestrator can react to human presence.

### Acceptance Criteria:

1. **Existing Driver Base:**
   - Start from Epic 1 `ros2_nodes/hardware_drivers/head_driver.py`
   - Existing expression and blink topics remain functional
   - I2C communication to address 0x08 working

2. **New ROS2 Publications:**
   - `/olaf/head/presence` (custom msg: detected, duration, event_count) at 10Hz
     - Reads registers 0x20-0x22 (presence status, duration, events)
     - Publishes current presence data
   - Message definition: `PresenceData.msg`:
     ```
     bool detected           # True if presence currently detected
     uint8 duration          # Seconds of continuous presence
     uint8 event_count       # Total presence events
     ```

3. **Presence Event Callback (Optional):**
   - Detect rising edge: `detected` changes False → True
   - Publish event to `/olaf/events/presence_detected` (std_msgs/Empty)
   - Enables orchestrator to react immediately to new presence without polling

4. **Testing:**
   - Node launches: Existing head driver functionality works
   - New topic appears: `/olaf/head/presence`
   - No presence: Topic publishes `detected=False, duration=0`
   - Approach sensor: Topic publishes `detected=True, duration` increments
   - Leave sensor: `detected=False`, event_count incremented
   - Event test: Approach sensor → `/olaf/events/presence_detected` published

5. **Integration Testing:**
   - Combined test: Send expression command while presence detected → both work
   - Latency: Presence detection → ROS2 topic publish <100ms

6. **Code Quality:**
   - File: `head_driver.py` updated
   - Message definition: `orchestrator/msg/PresenceData.msg`
   - Comments explain presence polling and event detection
   - Code committed to `ros2/src/orchestrator/`

**Dependencies:** Story 3.6 (presence sensor firmware), Epic 1 (head driver)

**Estimated Effort:** 3-4 hours

**Note:** Simple enhancement to existing driver. Enables presence-aware personality behaviors.

---

## Story 3.9: Coordinated Expression System - Eyes + Ears + Neck

**As a** software developer,
**I want** a ROS2 orchestrator node coordinating eyes, ears, and neck for synchronized emotional expressions,
**so that** Olaf displays coherent multi-modal emotions.

### Acceptance Criteria:

1. **Orchestrator Node:**
   - Python ROS2 node: `ros2/src/orchestrator/ros2_nodes/coordinated_expression.py`
   - Node name: `/olaf/coordinated_expression`

2. **High-Level Expression API:**
   - Subscribes to: `/olaf/expression/set` (custom msg: emotion_type, intensity)
   - Emotion types supported:
     - `neutral`: Eyes neutral, ears centered, neck level
     - `happy`: Eyes wide/bright, ears up, slight head tilt
     - `curious`: Eyes large pupils, ears forward, head tilt + roll
     - `thinking`: Eyes looking up-right, ears back, head tilt up
     - `confused`: Eyes asymmetric, ears asymmetric, slight head shake
     - `sad`: Eyes small pupils, ears down/back, head tilt down
     - `excited`: Eyes very wide, ears alert up, head rapid micro-movements

3. **Multi-Modal Coordination Logic:**
   - For each emotion:
     - Compute eye expression parameters → publish to `/olaf/head/expression`
     - Compute ear positions (4 servos) → publish to `/olaf/ears/position`
     - Compute neck position (pan/tilt/roll) → publish to `/olaf/neck/position`
     - Optional: Use gesture commands for complex movements (e.g., "look-left" for curious)

4. **Example Emotion Mappings:**
   - **Happy:**
     - Eyes: Expression type 1 (happy), intensity 4
     - Ears: Both vertical +30°, horizontal centered
     - Neck: Tilt up 10°, roll 0°, pan 0°
   - **Curious:**
     - Eyes: Expression type 2 (curious), intensity 3
     - Ears: Left forward +40°, right centered
     - Neck: Roll 15°, tilt up 15°, pan -20° (look left slightly)
   - **Sad:**
     - Eyes: Expression type 5 (sad), intensity 2
     - Ears: Both vertical -20° (down), horizontal back -30°
     - Neck: Tilt down -15°, roll 0°, pan 0°

5. **Timing Coordination:**
   - All modality commands (eyes, ears, neck) published simultaneously
   - Smooth transitions: Interpolate positions over 300-500ms
   - No "stuttering": Ears and neck move at coordinated speeds

6. **Presence-Aware Behaviors:**
   - Subscribes to `/olaf/head/presence`
   - When presence detected after absence: Automatically trigger "curious" expression (looks at person)
   - When presence lost: Return to "neutral" after 5 seconds
   - Configurable: Enable/disable autonomous reactions via parameter

7. **Testing:**
   - Launch: `ros2 run orchestrator coordinated_expression`
   - Manual test: Publish `happy` emotion → eyes brighten, ears up, head tilts
   - Manual test: Publish `sad` emotion → eyes droop, ears down, head lowers
   - All 7 emotions tested, verified visually
   - Presence test: Approach sensor → Olaf looks at you with curious expression

8. **End-to-End Validation:**
   - Full chain: Expression command → Coordinated expression node → Head driver + Ears/Neck driver → I2C → ESP32s → servos + eyes
   - Latency: Expression command to visible movement <200ms
   - Reliability: 50 expression changes without failure
   - Smooth synchronized movement: Eyes, ears, neck all move together

9. **Code Quality:**
   - File: `coordinated_expression.py`
   - Emotion mapping config: YAML or Python dict
   - Comments explain coordination logic
   - Code committed to `ros2/src/orchestrator/ros2_nodes/`

**Dependencies:** Story 3.7 (ears/neck driver), Story 3.8 (head driver with presence)

**Estimated Effort:** 6-8 hours

**Note:** This is the "personality core" for Epic 3. All future emotional expressions (Epic 9) will build on this coordination framework.

---

## Story 3.10: Complete Head Assembly Integration & Testing

**As a** quality assurance tester,
**I want** the complete head assembly tested as an integrated system with 30-minute stability validation,
**so that** Epic 3 is production-ready before Epic 4 (body integration).

### Acceptance Criteria:

1. **System Integration Checklist:**
   - [ ] Head Module (ESP32 0x08): Eyes + presence sensor functional
   - [ ] Ears+Neck Module (ESP32 0x09): 7 servos responsive
   - [ ] ROS2 drivers: head_driver, ears_neck_driver both running
   - [ ] Coordinated expression node running
   - [ ] All I2C devices detected: `sudo i2cdetect -y 1` shows 0x08, 0x09
   - [ ] Power system stable: 5V supply for ESP32s + servo controllers

2. **Functional Testing:**
   - **Eyes:** All 7 expressions render correctly at 60 FPS
   - **Ears:** All 4 servos move through full range, smooth motion
   - **Neck:** All 3 axes (pan/tilt/roll) move through full range, head stable
   - **Presence Sensor:** Detects presence within 2 meters, publishes to ROS2
   - **Coordinated Expressions:** All 7 emotions display correctly across eyes+ears+neck

3. **Gesture Testing:**
   - Execute all predefined gestures:
     - look-left, look-right, nod-yes, shake-no, tilt-curious, alert-ears-up
   - Verify smooth coordinated movement
   - No mechanical binding or servo stalling

4. **Presence-Aware Behavior Testing:**
   - Autonomous mode enabled
   - Approach Olaf: Automatically transitions to "curious" expression
   - Leave: Returns to "neutral" after 5 seconds
   - Verify presence event counting accurate

5. **30-Minute Continuous Operation Test:**
   - **Test Setup:**
     - Launch all nodes: `ros2 launch orchestrator head_system.launch.py`
     - Power system stable (bench supply or battery)
     - Timer set for 30 minutes
   - **Test Execution:**
     - Cycle through all 7 emotions every 30 seconds (60 cycles total)
     - Include random gestures every 2 minutes (15 gestures total)
     - Simulate presence events: Approach/leave 10 times during test
   - **Success Criteria:**
     - No crashes: All nodes run 30 minutes
     - No missed updates: All 60 emotion cycles complete
     - Stable latency: Expression command → visible change <200ms
     - Memory stable: ESP32 free heap doesn't decrease
     - No servo overheating: Thermal check post-test
     - No mechanical wear: Gears, brackets remain secure

6. **Metrics Collected:**
   - Total runtime: 30:00 minutes
   - Total emotion changes: 60
   - Total gestures executed: 15
   - Presence events: 10
   - I2C errors: 0
   - Average expression latency: <200ms (sampled 10 times)
   - Peak current draw: <3A

7. **Pass/Fail Criteria:**
   - PASS: All criteria met, no failures, smooth operation
   - FAIL: Any crash, >5% missed updates, latency >300ms, servo failure

8. **Documentation:**
   - Results: `tests/integration/epic3_30min_test_results.md`
   - Video: 30-second clip showing coordinated expressions
   - Photos: Full head assembly from multiple angles

**Dependencies:** All Epic 3 stories (3.1-3.9)

**Estimated Effort:** 2 hours test execution + 4-6 hours debugging if issues found

**Note:** This is the Epic 3 quality gate. Must pass before moving to Epic 4 (body assembly).

---

## Story 3.11: Epic 3 Documentation & Build-in-Public Content

**As a** future contributor or follower,
**I want** comprehensive Epic 3 documentation and build-in-public content,
**so that** I can replicate the head assembly and follow the journey.

### Acceptance Criteria:

1. **Technical Documentation:**
   - `firmware/head/README.md` updated:
     - Presence sensor integration
     - Updated I2C register map (0x20-0x23)
     - Firmware build/upload process
   - `firmware/ears_neck/README.md` created:
     - Dual UART servo controller architecture
     - Servo ID mappings (1-7)
     - I2C register map (0x10-0x32)
     - Kinematics reference
     - Gesture definitions
   - `ros2/src/orchestrator/ros2_nodes/README.md` updated:
     - Ears+Neck driver usage
     - Coordinated expression API
     - Presence detection topics

2. **Hardware Documentation:**
   - BOM: `hardware/bom/epic3_bom.csv`
     - ESP32-S3 (Ears+Neck module)
     - 4× Feetech SCS0009 servos
     - 3× Feetech STS3215 servos
     - 2× Bus servo controllers
     - DFRobot SEN0395 presence sensor
     - M3/M4 screws, standoffs
     - 3D printed parts list with print times
     - Supplier links, costs
   - Wiring diagrams:
     - `hardware/wiring/head-module-v2-wiring.png` (eyes + presence sensor)
     - `hardware/wiring/ears-servo-wiring.png` (4 servos + controller)
     - `hardware/wiring/neck-gimbal-wiring.png` (3 servos + controller + ESP32)
     - `hardware/wiring/full-head-i2c-diagram.png` (both ESP32s to Pi)

3. **Assembly Guides:**
   - `hardware/assembly/head-assembly-guide.md`:
     - Step-by-step instructions with photos
     - 3D printing tips
     - Servo calibration procedure
     - Cable management best practices
     - Troubleshooting common issues

4. **Photos/Videos:**
   - Photos:
     - 3D printed parts laid out
     - Ear servo assembly (close-up)
     - Neck gimbal assembly (all 3 axes)
     - Complete head assembly (multiple angles)
     - Eyes + ears + neck coordinated expression (happy, sad, curious)
   - Videos:
     - 30-second time-lapse of assembly
     - 1-minute demo: All 7 emotions cycling
     - 15-second clip: Presence detection reaction (person approaches → curious expression)

5. **GitHub Milestone:**
   - Milestone "Epic 3: Complete Head Assembly" created
   - All stories (3.1-3.11) closed and linked
   - Milestone marked complete

6. **Build-in-Public Content:**
   - **LinkedIn Post:** "Olaf's Personality Comes Alive! 🤖✨"
     - Photo carousel: Eyes, ears, neck in different expressions
     - Explanation: "After 3 weeks of building, Olaf can now express emotions through coordinated eyes, ears, and neck movements. 7 servos, 2 ESP32s, and a lot of kinematics later... he's got personality!"
     - Technical depth: Multi-modal expression coordination, 3-DOF neck gimbal, presence detection
     - GitHub link to Epic 3 documentation
   - **Optional Reddit Post (r/robotics):**
     - Title: "Built a 3-DOF Neck Gimbal + Articulated Ears for Expressive Robot Head"
     - Content: Assembly process, challenges (cable routing, servo calibration), results (coordinated expressions)
     - GIF: Head looking left/right with ears following

7. **Troubleshooting Guide:**
   - `docs/troubleshooting-epic3.md`:
     - "Ears not moving" → Check UART1 wiring, servo IDs
     - "Neck servo stalling under head weight" → Verify STS3215 torque settings, check for binding
     - "ESP32 0x09 not detected on I2C" → Verify address configuration, check wiring
     - "Jerky servo movement" → Adjust motion profile easing, check power supply voltage sag
     - "Presence sensor not detecting" → Check GPIO16 wiring, test with serial monitor

**Dependencies:** All Epic 3 stories complete (3.1-3.10)

**Estimated Effort:** 8-10 hours (documentation + photos/videos + content)

---

## Epic 3 Summary

**Total Stories:** 11

**Estimated Total Effort:** 66-91 hours (3-4 weeks for solo builder)

**Key Deliverables:**
- ✅ Complete 3D-printed head housing (head shell, ear mounts, neck gimbal components)
- ✅ **Head Module Enhanced:** Dual GC9A01 eyes (Epic 1) + mmWave presence sensor at I2C 0x08
- ✅ **Ears+Neck Module:** 7 servos (4 ears + 3 neck) controlled via dual UART at I2C 0x09
- ✅ 2-DOF articulated ears (±60° horizontal, ±45° vertical per ear)
- ✅ 3-DOF neck gimbal (pan ±90°, tilt ±45°, roll ±30°)
- ✅ Coordinated kinematics engine with predefined gestures
- ✅ ESP32 firmware for ears+neck servo control and head presence detection
- ✅ ROS2 drivers: head_driver (enhanced), ears_neck_driver (new)
- ✅ Coordinated expression system: 7 emotions across eyes+ears+neck
- ✅ Presence-aware autonomous behaviors
- ✅ 30-minute continuous operation validated
- ✅ Complete documentation + build-in-public content

**Architecture Validated:**
- ✅ Multi-modal emotional expression (eyes + ears + neck synchronized)
- ✅ Dual I2C modules: Head (0x08) + Ears+Neck (0x09)
- ✅ Dual UART servo control architecture (independent ear and neck buses)
- ✅ 3-DOF gimbal kinematics for natural head movement
- ✅ Presence detection for autonomous reactivity
- ✅ Coordinated ROS2 orchestration layer
- ✅ Physical mechanical integration (3D printed parts + servos + electronics)

**Success Criteria Met:**
- Multi-modal expressiveness proven (eyes + ears + neck work together)
- "Olaf comes alive with personality!" moment achieved
- Upper-body mobility validated (full range of motion tested)
- Presence awareness enables reactive behaviors
- Mechanical assembly complete and robust (30-min stability test passed)
- Foundation ready for Epic 4 (Core Torso & Power System integration)

**Next Epic:** Epic 5: Core Torso & Power System - Complete Build

**Note:** Audio remains USB-based (conference mic+speaker connected to Pi). OAK-D Pro camera integration deferred to Epic 10 (SLAM & Spatial Awareness).

---

**Related Documents:**
- [PRD Epic List](epic-list.md) - Overall project roadmap
- [Epic 1: Foundation](epic-01-foundation.md) - Head Module eyes foundation
- [Architecture - Components](../architecture/components.md) - Head, Ears+Neck module specifications
- [Tech Stack](../architecture/tech-stack.md) - ESP32-S3, Feetech servos, GC9A01 displays
