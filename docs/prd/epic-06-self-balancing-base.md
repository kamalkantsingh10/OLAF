# Epic 6: Self-Balancing Base - Complete Build

**Epic Goal:** Complete the self-balancing base platform by 3D printing base components (with Epic 2 feedback incorporated), assembling two-wheel balancing system with hoverboard motors and wheels, integrating ODrive motor controller and MPU6050 IMU, implementing 200Hz PID control loop for active balancing, deploying kickstand servo for fall protection, and integrating odometry for SLAM readiness.

**Duration:** 3-4 weeks (Weeks 14-17)

**Prerequisites:**
- Epic 1 (Foundation & I2C Communication) - I2C architecture validated
- Epic 2 (Complete OnShape Design & Community Feedback) - All STLs exported and feedback incorporated
- Epic 5 (Core Torso & Power System) - 12V and 5V power rails ready, power system validated

**Value Delivered:** Active mobility foundation enabling autonomous movement, self-balancing capability for stable operation without kickstand, navigation-ready odometry for SLAM integration in Epic 10, fall protection system for safety, foundation for autonomous navigation in Epic 11.

**Architecture Focus:** This epic establishes the Base Module (I2C 0x0B) controlling self-balancing system, implements real-time 200Hz PID control loop on ESP32, integrates ODrive motor controller for hoverboard motors, validates MPU6050 IMU for gyro/accelerometer data, and creates ROS2 odometry publisher for navigation stack.

**Hardware Note:** Base Module requires ESP32-S3 for real-time PID loop (200Hz), MPU6050 IMU for balancing sensor fusion, ODrive v3.6 for dual hoverboard motor control, and Feetech STS3215 servo for kickstand deployment.

---

## Story 6.1: 3D Print Base Platform Components

**As a** hardware builder,
**I want** all base platform components 3D printed with Epic 2 feedback incorporated,
**so that** I have all physical parts ready for assembly.

### Acceptance Criteria:

1. **STL Files Verified:**
   - Base platform STLs from Epic 2 design downloaded from OnShape
   - Files reviewed for completeness:
     - `base-platform-main.stl` (main structural platform)
     - `base-platform-bottom.stl` (bottom plate)
     - `motor-mount-left.stl` (hoverboard motor mount, left)
     - `motor-mount-right.stl` (hoverboard motor mount, right)
     - `wheel-adapter-left.stl` (wheel-to-motor adapter, left)
     - `wheel-adapter-right.stl` (wheel-to-motor adapter, right)
     - `odrive-mount.stl` (ODrive controller mounting bracket)
     - `imu-mount.stl` (MPU6050 IMU mounting bracket)
     - `kickstand-base.stl` (kickstand pivot base)
     - `kickstand-foot.stl` (kickstand contact foot)
     - `kickstand-servo-bracket.stl` (STS3215 servo mounting)
     - `cable-routing-base.stl` (cable management for base)

2. **3D Printing Settings:**
   - Material: PETG (strength for load-bearing platform, impact resistance)
   - Layer height: 0.2mm (0.15mm for motor mounts precision)
   - Infill: 40% for structural parts (platform, motor mounts), 30% for brackets
   - Supports: Auto-generated where needed (motor mounts, kickstand pivot)
   - Print orientation optimized for strength along load vectors

3. **Printing Execution:**
   - All components printed successfully
   - No warping, layer separation, or print failures
   - Support material removed cleanly
   - Screw holes drilled/cleaned to proper size (M4 for motor mounts, M3 for brackets)
   - Load-bearing surfaces verified for strength

4. **Quality Verification:**
   - Motor mounts fit hoverboard motor dimensions (check bolt pattern)
   - Wheel adapters fit hoverboard wheels (typically 6.5" diameter)
   - ODrive mount fits ODrive v3.6 PCB dimensions
   - IMU mount fits MPU6050 breakout board
   - Kickstand mechanism moves smoothly (pivot test)
   - Platform can support full robot weight (~5-7kg estimated)

5. **Post-Processing:**
   - Support scars sanded smooth (especially load-bearing surfaces)
   - Functional surfaces (motor bolt holes, wheel adapters) checked for fit
   - Test-fit major components: Motors (if available), ODrive PCB, IMU
   - Structural load test: Place weight equivalent to full robot (~7kg)

6. **Documentation:**
   - Print time logged: `hardware/3d-models/print-log.md`
   - Photos of printed parts: `hardware/3d-models/Base/assembly-parts-photos/`
   - Any print issues noted for future reprints

**Dependencies:** Epic 2 complete (STLs exported, community feedback incorporated)

**Estimated Effort:** 10-14 hours print time (spread over 3-4 days) + 3-4 hours post-processing

**Note:** Base platform is load-bearing - PETG required for strength. Motor mounts critical for alignment.

---

## Story 6.2: Motor and Wheel Assembly

**As a** hardware builder,
**I want** hoverboard motors and wheels assembled to base platform,
**so that** I have the mechanical drive system ready for motor control.

### Acceptance Criteria:

1. **Component Preparation:**
   - 2× Hoverboard motors (brushless DC, ~350W each, with hall effect sensors)
   - 2× Hoverboard wheels (6.5" diameter, solid rubber tires)
   - M4 bolts and nuts for motor mounting (typically 4× M4×20mm per motor)
   - Wheel adapter hardware (set screws or bolts to secure wheel to motor shaft)

2. **Motor Mounting:**
   - Left motor mounted in `motor-mount-left.stl` with M4 bolts
   - Right motor mounted in `motor-mount-right.stl` with M4 bolts
   - Motor mounts secured to `base-platform-main.stl`
   - Motors aligned: Shafts parallel, wheel plane perpendicular to platform
   - Mounting bolts torqued properly (secure but not over-tightened)

3. **Wheel Installation:**
   - Left wheel attached to left motor shaft via `wheel-adapter-left.stl`
   - Right wheel attached to right motor shaft via `wheel-adapter-right.stl`
   - Wheel adapters secured with set screws or bolts
   - Wheels aligned: Both wheels parallel, same height from ground

4. **Motor Wiring Preparation:**
   - Motor phase wires (3 wires per motor: A, B, C) accessible for ODrive connection
   - Hall sensor wires (5 wires per motor: VCC, GND, HA, HB, HC) accessible
   - Wires organized and labeled:
     - Left motor: "Motor L - Phase A/B/C", "Motor L - Hall VCC/GND/HA/HB/HC"
     - Right motor: "Motor R - Phase A/B/C", "Motor R - Hall VCC/GND/HA/HB/HC"

5. **Mechanical Testing:**
   - Manual wheel rotation: Both wheels spin freely, no binding
   - Motor shaft alignment: No wobble when wheels rotated
   - Wheel adapters secure: No slippage under manual torque test
   - Platform stability: Place on flat surface, verify level with bubble level

6. **Load Testing:**
   - Place full robot weight (~7kg) on platform
   - Verify no flex or sagging in motor mounts
   - Check all mounting bolts remain tight under load
   - Wheels contact ground evenly (no rocking)

7. **Documentation:**
   - Photos: Motor mounting, wheel installation, complete base assembly
   - Save to `hardware/wiring/base-motor-assembly/`
   - Motor specifications documented: `firmware/base/README.md`
   - Wiring color codes noted

**Dependencies:** Story 6.1 (base platform printed)

**Estimated Effort:** 6-8 hours

**Note:** Hoverboard motors typically 350W, ~36V nominal, hall sensor equipped for commutation sensing.

---

## Story 6.3: ODrive Motor Controller Setup and Configuration

**As a** hardware builder,
**I want** ODrive motor controller integrated and configured for hoverboard motors,
**so that** I can control motor speed and direction via UART.

### Acceptance Criteria:

1. **Component Preparation:**
   - ODrive v3.6 motor controller (dual-channel, supports 2 motors)
   - XT60 connector for 12V power input (from Epic 5 power system)
   - USB cable for ODrive configuration (USB-C to PC)
   - UART wiring for ESP32 communication (Story 6.5)

2. **ODrive Physical Mounting:**
   - ODrive PCB secured in `odrive-mount.stl` bracket
   - Bracket positioned on base platform (accessible for wiring, heat dissipation)
   - ODrive heatsink verified attached (critical for thermal management)

3. **Power Wiring:**
   - 12V power from Epic 5 buck converter → ODrive XT60 power input
   - GND connection verified (common ground with base platform)
   - Power tested: ODrive LED turns on when powered (initial boot)

4. **Motor Phase Wiring:**
   - **Left Motor (Axis 0):**
     - Phase A → ODrive M0 A
     - Phase B → ODrive M0 B
     - Phase C → ODrive M0 C
   - **Right Motor (Axis 1):**
     - Phase A → ODrive M1 A
     - Phase B → ODrive M1 B
     - Phase C → ODrive M1 C
   - Phase wire connections secure (screw terminals or solder)

5. **Hall Sensor Wiring:**
   - **Left Motor Hall Sensors (Axis 0):**
     - Hall VCC → ODrive M0 5V
     - Hall GND → ODrive M0 GND
     - Hall A → ODrive M0 HA
     - Hall B → ODrive M0 HB
     - Hall C → ODrive M0 HC
   - **Right Motor Hall Sensors (Axis 1):**
     - Hall VCC → ODrive M1 5V
     - Hall GND → ODrive M1 GND
     - Hall A → ODrive M1 HA
     - Hall B → ODrive M1 HB
     - Hall C → ODrive M1 HC

6. **ODrive Configuration (via odrivetool):**
   - Connect ODrive to PC via USB
   - Install odrivetool: `pip install odrive`
   - Launch odrivetool: `odrivetool`
   - **Configure Axis 0 (Left Motor):**
     - `odrv0.axis0.motor.config.pole_pairs = 15` (hoverboard motors typically 15 pole pairs)
     - `odrv0.axis0.motor.config.current_lim = 20` (20A max current per motor)
     - `odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL`
     - `odrv0.axis0.encoder.config.cpr = 90` (15 pole pairs × 6 = 90 counts per revolution)
   - **Configure Axis 1 (Right Motor):** Same as Axis 0
   - Save configuration: `odrv0.save_configuration()`
   - Reboot ODrive: `odrv0.reboot()`

7. **Motor Calibration:**
   - **Axis 0 calibration:**
     - `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`
     - Motor spins, hall sensors calibrated
     - Verify: `odrv0.axis0.encoder.is_ready` returns `True`
   - **Axis 1 calibration:** Same procedure
   - Test velocity control:
     - `odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL`
     - `odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`
     - `odrv0.axis0.controller.input_vel = 2` (2 rev/s, wheel spins slowly)
     - Verify smooth rotation, no cogging

8. **UART Communication Setup:**
   - Configure ODrive for UART control (alternative to USB for ESP32):
     - `odrv0.config.enable_uart = True`
     - `odrv0.config.uart_baudrate = 115200`
     - Save and reboot
   - UART wiring to ESP32 (Story 6.5): ODrive GPIO1 (TX) → ESP32 RX, ODrive GPIO2 (RX) → ESP32 TX

9. **Safety Configuration:**
   - Configure velocity limits (prevent runaway):
     - `odrv0.axis0.controller.config.vel_limit = 10` (10 rev/s max, ~3 m/s linear)
     - `odrv0.axis1.controller.config.vel_limit = 10`
   - Configure current limits (prevent motor burnout):
     - `odrv0.axis0.motor.config.current_lim = 20` (20A per motor)
     - `odrv0.axis1.motor.config.current_lim = 20`

10. **Testing:**
    - Velocity control test: Both motors spin at commanded velocity
    - Direction test: Positive velocity → forward, negative → reverse
    - Hall sensor test: Encoder counts increment smoothly
    - Current limiting: Apply load (brake wheel), verify current doesn't exceed 20A

11. **Documentation:**
    - Photos: ODrive wiring, motor connections
    - Save to `hardware/wiring/odrive-motor-wiring/`
    - ODrive configuration parameters documented in `firmware/base/README.md`

**Dependencies:** Story 6.2 (motors mounted)

**Estimated Effort:** 8-10 hours (includes calibration and troubleshooting)

**Note:** ODrive calibration critical - motors must calibrate successfully before balancing control.

---

## Story 6.4: IMU (MPU6050) Integration

**As a** hardware builder,
**I want** MPU6050 IMU physically mounted and wired to ESP32-S3,
**so that** I have gyro/accelerometer data for balancing control.

### Acceptance Criteria:

1. **Component Preparation:**
   - MPU6050 breakout board (6-axis IMU: 3-axis gyro + 3-axis accelerometer)
   - M2.5 screws for IMU mounting
   - 15cm jumper wires for I2C connection to ESP32

2. **IMU Mounting:**
   - MPU6050 mounted in `imu-mount.stl` bracket with M2.5 screws
   - Bracket positioned on base platform:
     - Central location (minimize moment arm from wheels)
     - X-axis aligned with forward direction
     - Z-axis aligned with vertical (perpendicular to ground)
   - IMU orientation marked clearly (arrow indicating forward direction)

3. **I2C Wiring (MPU6050 → ESP32-S3):**
   - **Power:**
     - VCC → ESP32-S3 3.3V
     - GND → ESP32-S3 GND
   - **I2C Interface:**
     - SDA → ESP32-S3 GPIO26 (or available I2C SDA pin, NOT shared with Pi I2C bus)
     - SCL → ESP32-S3 GPIO27 (or available I2C SCL pin)
   - **Note:** Use separate I2C bus from Raspberry Pi I2C (Base Module doesn't share I2C with Pi for sensors)

4. **I2C Address Configuration:**
   - MPU6050 default address: 0x68 (or 0x69 if AD0 pin high)
   - Verify address with ESP32 I2C scan (firmware in Story 6.5)

5. **IMU Calibration Preparation:**
   - Place base platform on level surface
   - IMU should read:
     - Gyro: 0°/s on all axes (when stationary)
     - Accel: 0g on X and Y, +1g on Z (gravity)
   - Mark level orientation for future calibration reference

6. **Mechanical Stability:**
   - IMU mount rigid (no flex or vibration)
   - Test: Tap base platform, IMU shouldn't wobble or shift
   - Cable routing secured (IMU wires won't pull or stress mount)

7. **Documentation:**
   - Photos: IMU mounting, wiring
   - Save to `hardware/wiring/base-imu-assembly/`
   - IMU orientation diagram (X/Y/Z axes labeled)
   - I2C pinout documented in `firmware/base/README.md`

**Dependencies:** Story 6.1 (base platform printed)

**Estimated Effort:** 3-4 hours

**Note:** IMU orientation critical - incorrect orientation will cause balancing failure. X-axis = forward, Y-axis = left, Z-axis = up.

---

## Story 6.5: Base Module ESP32 Firmware - Balancing Controller

**As a** firmware developer,
**I want** ESP32 firmware implementing 200Hz PID balancing loop with MPU6050 and ODrive control,
**so that** the base can self-balance autonomously.

### Acceptance Criteria:

1. **Development Environment:**
   - PlatformIO project: `firmware/base/firmware/`
   - Platform: ESP32-S3 (Arduino framework)
   - Libraries:
     - `Wire.h` (I2C slave for Pi communication)
     - `MPU6050.h` (IMU driver)
     - `HardwareSerial.h` (UART for ODrive communication)

2. **I2C Slave Implementation (Pi Communication):**
   - ESP32-S3 configured as I2C slave at address **0x0B**
   - I2C receive/request handlers implemented
   - Interrupt-driven I2C communication

3. **I2C Register Map (Base Module 0x0B):**
   - `0x00`: Module ID (returns 0x0B)
   - `0x02`: Status byte (READY/BALANCING/ERROR/FALLEN)
   - **Balancing Control:**
     - `0x10`: Enable balancing (0=off, 1=on)
     - `0x11`: Target velocity (signed int16, m/s × 100, e.g., 50 = 0.5 m/s forward)
     - `0x12`: Target angular velocity (signed int16, rad/s × 100, for turning)
   - **Kickstand Control:**
     - `0x20`: Deploy kickstand (0=retract, 1=deploy)
   - **Sensor Data (read-only):**
     - `0x30-0x31`: Pitch angle (int16, degrees × 100)
     - `0x32-0x33`: Pitch rate (int16, deg/s × 100)
     - `0x34-0x35`: Left wheel velocity (int16, m/s × 100)
     - `0x36-0x37`: Right wheel velocity (int16, m/s × 100)

4. **MPU6050 IMU Interface:**
   - MPU6050 initialized on I2C (GPIO26/27)
   - Gyro configured: ±250°/s range (balancing typically <50°/s)
   - Accel configured: ±2g range (balancing within ±45°)
   - Sample rate: 200Hz (matches PID loop frequency)
   - Digital Low Pass Filter (DLPF): 98Hz (reduce noise)

5. **Sensor Fusion - Complementary Filter:**
   - Pitch angle estimation from gyro + accel:
     - `pitch = 0.98 * (pitch + gyro_rate * dt) + 0.02 * accel_pitch`
     - Accel pitch: `atan2(accel_y, accel_z)`
     - Gyro integration with complementary filter (98% gyro, 2% accel)
   - Update rate: 200Hz (5ms interval)

6. **PID Controller Implementation:**
   - **Inner Loop (Pitch Stabilization):**
     - Target: 0° pitch (vertical)
     - PID gains: Kp, Ki, Kd (tune in Story 6.7)
     - Output: Motor torque command
   - **Outer Loop (Velocity Control):**
     - Target velocity from I2C register 0x11
     - Adjust pitch setpoint to achieve target velocity
     - PID output: Pitch setpoint offset
   - **PID Loop Frequency: 200Hz (5ms period)**

7. **ODrive Motor Control (UART):**
   - UART communication to ODrive (115200 baud)
   - ASCII protocol commands:
     - Set velocity: `v 0 <vel>\n` (Axis 0, velocity in rev/s)
     - Set velocity: `v 1 <vel>\n` (Axis 1)
   - Velocity command calculation from PID output:
     - `motor_vel_revs = pid_output / wheel_radius_m`
   - Differential drive for turning:
     - Left motor: `base_vel + angular_vel`
     - Right motor: `base_vel - angular_vel`

8. **Fall Detection:**
   - Pitch threshold: |pitch| > 45° → FALLEN state
   - When fallen:
     - Disable motors (send velocity 0 to ODrive)
     - Deploy kickstand (Story 6.8)
     - Set status register 0x02 to FALLEN
     - Require manual reset via I2C command

9. **Kickstand Servo Control:**
   - Feetech STS3215 servo for kickstand deployment
   - UART control (separate from ODrive, or shared bus if daisy-chained)
   - Positions:
     - Retracted: 0° (kickstand up, balancing mode)
     - Deployed: 90° (kickstand down, support robot)
   - Deploy on: Fall detected OR manual command (register 0x20)

10. **Performance:**
    - PID loop: 200Hz consistent (measured with timer)
    - I2C latency: <10ms response to Pi commands
    - Fall detection: <50ms from threshold to motor disable
    - Memory stable: No leaks over 30 minutes balancing

11. **Code Quality:**
    - Files:
      - `main.cpp`
      - `i2c_slave.cpp`
      - `mpu6050_driver.cpp`
      - `complementary_filter.cpp`
      - `pid_controller.cpp`
      - `odrive_uart.cpp`
      - `kickstand_control.cpp`
    - Comments explain PID gains, sensor fusion, fall detection logic
    - Code committed to `firmware/base/firmware/`

**Dependencies:** Story 6.3 (ODrive configured), Story 6.4 (IMU mounted)

**Estimated Effort:** 12-16 hours

**Note:** PID tuning happens in Story 6.7. Initial gains can be conservative (low Kp, zero Ki/Kd) for safety.

---

## Story 6.6: ROS2 Base Driver Node

**As a** software developer,
**I want** ROS2 driver node translating ROS2 topics/commands into I2C commands for Base Module,
**so that** the orchestrator and navigation stack can control balancing and movement.

### Acceptance Criteria:

1. **Node Structure:**
   - Python ROS2 node: `ros2/src/orchestrator/ros2_nodes/hardware_drivers/base_driver.py`
   - Node name: `/olaf/base_driver`
   - Uses `rclpy` and `smbus2`

2. **I2C Communication:**
   - Opens I2C bus 1: `bus = SMBus(1)`
   - Helper functions for register read/write to address **0x0B**

3. **ROS2 Subscriptions:**
   - `/cmd_vel` (geometry_msgs/Twist) - Standard ROS2 velocity command
     - Callback converts to I2C register writes:
       - `linear.x` → register 0x11 (forward velocity, m/s × 100)
       - `angular.z` → register 0x12 (angular velocity, rad/s × 100)
   - `/olaf/base/enable_balancing` (std_msgs/Bool)
     - Callback writes to register 0x10 (enable balancing)
   - `/olaf/base/deploy_kickstand` (std_msgs/Bool)
     - Callback writes to register 0x20 (kickstand deploy/retract)

4. **ROS2 Publications:**
   - `/olaf/base/status` (std_msgs/String) at 10Hz
     - Reads status register 0x02, publishes "READY", "BALANCING", "ERROR", "FALLEN"
   - `/olaf/base/imu` (sensor_msgs/Imu) at 50Hz
     - Reads pitch angle and rate from registers 0x30-0x33
     - Publishes IMU data for monitoring/debugging
   - `/odom` (nav_msgs/Odometry) at 20Hz
     - Reads wheel velocities from registers 0x34-0x37
     - Computes odometry (x, y, theta) from wheel velocities
     - Publishes for navigation stack (Epic 10)

5. **Odometry Calculation:**
   - Read left and right wheel velocities (m/s)
   - Compute linear velocity: `v = (v_left + v_right) / 2`
   - Compute angular velocity: `omega = (v_right - v_left) / wheel_base`
   - Integrate to get pose: `x += v * cos(theta) * dt`, `y += v * sin(theta) * dt`, `theta += omega * dt`
   - Publish as nav_msgs/Odometry with covariance

6. **Module Health Check:**
   - On startup: Read register 0x00, verify response is 0x0B
   - If not responding: Log error, publish ERROR status
   - Periodic health check: Every 30s verify module responsive

7. **Error Handling:**
   - I2C timeout handling (module busy in PID loop)
   - Retry logic: 3 attempts with 10ms delay
   - Persistent failure: Publish ERROR, log details
   - Fall detection: When status = FALLEN, publish warning, stop sending velocity commands

8. **Testing:**
   - Node launches: `ros2 run orchestrator base_driver`
   - Topics appear: `/cmd_vel`, `/odom`, `/olaf/base/status`
   - **Balancing Test:**
     - Publish enable balancing: `ros2 topic pub /olaf/base/enable_balancing std_msgs/Bool "data: true"`
     - Verify status changes to "BALANCING"
   - **Velocity Test:**
     - Publish cmd_vel: `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"`
     - Verify base moves forward at 0.2 m/s
   - **Odometry Test:**
     - Subscribe to odom: `ros2 topic echo /odom`
     - Verify x position increments as robot moves forward

9. **Code Quality:**
   - File: `base_driver.py`
   - Comments explain odometry calculation, I2C register mapping
   - Code committed to `ros2/src/orchestrator/ros2_nodes/hardware_drivers/`

**Dependencies:** Story 6.5 (Base firmware)

**Estimated Effort:** 6-8 hours

**Note:** Odometry critical for navigation. Wheel velocities must be accurate for SLAM in Epic 10.

---

## Story 6.7: PID Tuning and Balancing Validation

**As a** quality assurance tester,
**I want** PID controller tuned for stable balancing and validated with 10-minute continuous balance test,
**so that** the base reliably self-balances without falling.

### Acceptance Criteria:

1. **PID Tuning Setup:**
   - Base platform on level surface (hardwood floor or concrete)
   - Battery fully charged (consistent voltage for testing)
   - Serial monitor connected to Base Module ESP32 for real-time tuning
   - Safety: Soft landing area (pillows/foam) in case of fall

2. **Initial PID Gains (Conservative):**
   - Kp = 20 (proportional gain, conservative starting point)
   - Ki = 0 (integral gain, start at zero)
   - Kd = 5 (derivative gain, damping)

3. **Tuning Procedure (Ziegler-Nichols Method):**
   - **Step 1: Find Kp critical (Ku):**
     - Set Ki=0, Kd=0
     - Increase Kp until robot oscillates continuously at constant amplitude
     - Record Ku (critical gain)
   - **Step 2: Measure oscillation period (Tu):**
     - Time oscillation period in seconds
   - **Step 3: Calculate PID gains:**
     - Kp = 0.6 × Ku
     - Ki = 1.2 × Ku / Tu
     - Kd = 0.075 × Ku × Tu
   - **Step 4: Fine-tune:**
     - Adjust Kp: Increase for faster response, decrease if overshooting
     - Adjust Ki: Increase to eliminate steady-state error (lean), decrease if oscillating
     - Adjust Kd: Increase for damping, decrease if too sluggish

4. **Balance Stability Criteria:**
   - Robot balances for >10 seconds without kickstand
   - Pitch angle remains within ±3° of vertical
   - No sustained oscillations (damped response)
   - Recovery from external disturbances (gentle push)

5. **Disturbance Rejection Test:**
   - Apply gentle forward push: Robot should recover to vertical within 2 seconds
   - Apply gentle backward push: Same recovery
   - Apply lateral tap: Robot should not tip over (within stability range)

6. **10-Minute Continuous Balance Test:**
   - Enable balancing via ROS2: `ros2 topic pub /olaf/base/enable_balancing std_msgs/Bool "data: true"`
   - Robot balances continuously for 10 minutes
   - **Success Criteria:**
     - No falls (kickstand not deployed)
     - Pitch angle: ±5° maximum deviation from vertical
     - No motor stalling or overheating
     - Battery voltage stable (no excessive discharge)

7. **Velocity Control Test:**
   - Command forward velocity: 0.2 m/s
     - `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"`
   - Verify: Robot moves forward while maintaining balance
   - Command reverse velocity: -0.2 m/s
   - Verify: Robot moves backward while maintaining balance
   - Command turn: angular velocity 0.5 rad/s
   - Verify: Robot turns in place while maintaining balance

8. **Fall Detection Validation:**
   - Tilt robot beyond 45° manually
   - Verify: Motors disable, kickstand deploys, status = FALLEN
   - Verify: Robot doesn't attempt to balance when fallen

9. **PID Gain Documentation:**
   - Final PID gains recorded in `firmware/base/config.h`:
     - `const float KP_BALANCE = <final Kp>;`
     - `const float KI_BALANCE = <final Ki>;`
     - `const float KD_BALANCE = <final Kd>;`
   - Tuning notes in `docs/testing/pid-tuning-notes.md`

10. **Performance Metrics:**
    - PID loop frequency: 200Hz verified (5ms period)
    - IMU sample rate: 200Hz
    - Response time: <500ms from initial lean to vertical recovery
    - Steady-state error: <1° pitch deviation

**Dependencies:** Story 6.5 (firmware), Story 6.6 (ROS2 driver)

**Estimated Effort:** 8-12 hours (includes iterative tuning)

**Note:** PID tuning is iterative. Expect multiple tuning sessions. Safety first - always have soft landing area.

---

## Story 6.8: Kickstand Servo Integration

**As a** hardware builder,
**I want** kickstand servo deployed automatically on fall detection or manual command,
**so that** the robot has fall protection and safe parking.

### Acceptance Criteria:

1. **Component Preparation:**
   - Feetech STS3215 servo (30 kg·cm torque for kickstand)
   - Kickstand hardware from Story 6.1: `kickstand-base.stl`, `kickstand-foot.stl`, `kickstand-servo-bracket.stl`
   - M3 screws for servo mounting

2. **Kickstand Mechanical Assembly:**
   - Servo mounted in `kickstand-servo-bracket.stl`
   - Bracket attached to base platform
   - `kickstand-base.stl` attached to servo horn
   - `kickstand-foot.stl` attached to kickstand arm (contact point with ground)
   - Kickstand pivot mechanism tested manually (smooth deployment/retraction)

3. **Servo Wiring:**
   - Servo power: 5V from power distribution (Epic 5)
   - Servo control: UART or PWM from Base Module ESP32
     - Option A: UART (daisy-chain with ODrive if possible)
     - Option B: PWM (dedicated GPIO pin)

4. **Firmware Integration:**
   - Kickstand control functions in `firmware/base/firmware/kickstand_control.cpp`
   - Servo positions calibrated:
     - Retracted: 0° (kickstand up, balancing mode)
     - Deployed: 90° (kickstand down, supports robot weight)

5. **Automatic Deployment on Fall:**
   - Fall detection (|pitch| > 45°) triggers kickstand deployment
   - Kickstand deploys within 200ms of fall detection
   - Motors disabled when kickstand deployed

6. **Manual Control via ROS2:**
   - Publish deploy command: `ros2 topic pub /olaf/base/deploy_kickstand std_msgs/Bool "data: true"`
   - Verify: Kickstand deploys
   - Publish retract command: `ros2 topic pub /olaf/base/deploy_kickstand std_msgs/Bool "data: false"`
   - Verify: Kickstand retracts (only if balancing enabled and robot vertical)

7. **Safety Interlocks:**
   - Cannot retract kickstand if balancing disabled
   - Cannot retract kickstand if pitch > 10° (not vertical enough)
   - Kickstand automatically deploys if battery voltage < 32V (low battery protection)

8. **Load Testing:**
   - Deploy kickstand, place full robot weight on it
   - Verify kickstand supports weight without bending or servo stalling
   - Verify servo doesn't overheat after 5 minutes supporting weight

9. **Documentation:**
   - Photos: Kickstand assembly, deployed/retracted positions
   - Save to `hardware/wiring/base-kickstand-assembly/`

**Dependencies:** Story 6.5 (firmware), Story 6.7 (balancing validated)

**Estimated Effort:** 4-6 hours

**Note:** Kickstand is safety-critical. Must deploy reliably on fall to prevent robot damage.

---

## Story 6.9: Odometry Integration for SLAM

**As a** software developer,
**I want** accurate odometry published to /odom topic with proper covariance,
**so that** SLAM and navigation stack can use wheel encoder data.

### Acceptance Criteria:

1. **Odometry Calculation Enhancement:**
   - Wheel velocities read from Base Module (registers 0x34-0x37)
   - Wheel base (distance between wheels): Measured and configured (typically ~0.3m)
   - Wheel radius: Measured and configured (6.5" = 0.165m diameter → 0.0825m radius)

2. **Differential Drive Kinematics:**
   - Linear velocity: `v = (v_left + v_right) / 2`
   - Angular velocity: `omega = (v_right - v_left) / wheel_base`
   - Position integration:
     - `x += v * cos(theta) * dt`
     - `y += v * sin(theta) * dt`
     - `theta += omega * dt`

3. **Odometry Message Format:**
   - Topic: `/odom` (nav_msgs/Odometry)
   - Frame IDs:
     - `header.frame_id = "odom"` (global odometry frame)
     - `child_frame_id = "base_link"` (robot base frame)
   - Pose: x, y, theta (2D odometry)
   - Twist: v, omega (linear/angular velocity)
   - Covariance: Estimated based on wheel slip characteristics

4. **Covariance Matrix:**
   - Diagonal covariance for simplicity:
     - Pose covariance: 0.01 (x), 0.01 (y), 0.02 (theta)
     - Twist covariance: 0.05 (v), 0.1 (omega)
   - Increase covariance when:
     - Turning (higher slip on turns)
     - Accelerating (wheel slip during acceleration)

5. **TF Broadcasting:**
   - Publish TF transform: `odom` → `base_link`
   - TF timestamp synchronized with odometry message timestamp
   - Use `tf2_ros.TransformBroadcaster`

6. **Odometry Reset:**
   - Service: `/olaf/base/reset_odometry` (std_srvs/Empty)
   - Resets x, y, theta to (0, 0, 0)
   - Used when robot repositioned or SLAM reinitializes

7. **Testing:**
   - **Straight Line Test:**
     - Drive robot forward 1 meter (measure with tape)
     - Check odometry: x should be ~1.0m (±5% error acceptable)
   - **Rotation Test:**
     - Rotate robot 360° (full circle)
     - Check odometry: theta should return to ~0° (±10° error acceptable)
   - **TF Verification:**
     - `ros2 run tf2_tools view_frames.py`
     - Verify: `odom → base_link` transform exists

8. **Integration with Nav2 (Epic 10 Preparation):**
   - Verify odometry message format compatible with Nav2
   - Verify TF tree structure correct
   - Document odometry accuracy characteristics

9. **Code Quality:**
   - Odometry calculation in `base_driver.py`
   - Comments explain differential drive kinematics
   - Wheel parameters (wheel_base, wheel_radius) configurable via ROS2 parameters

**Dependencies:** Story 6.6 (base driver)

**Estimated Effort:** 4-6 hours

**Note:** Odometry accuracy critical for SLAM. Wheel slip on smooth floors will introduce drift - SLAM will correct in Epic 10.

---

## Story 6.10: Base Integration Testing & Validation

**As a** quality assurance tester,
**I want** complete base system validated with 30-minute autonomous balancing and movement test,
**so that** Epic 6 is production-ready before Epic 7 integration.

### Acceptance Criteria:

1. **Test Setup:**
   - All hardware from Epic 6 assembled
   - Battery fully charged
   - Open space for movement (hallway or room, ~5m × 5m minimum)
   - Safety: Soft obstacles in case of collision

2. **System Integration Checklist:**
   - Base Module (ESP32 0x0B): Firmware running, I2C responsive
   - ODrive: Both motors calibrated, velocity control functional
   - IMU: MPU6050 reading accurate pitch/rate
   - Kickstand: Deploys/retracts reliably
   - ROS2 driver: Publishing odometry, accepting velocity commands

3. **30-Minute Autonomous Balancing Test:**
   - **Test Procedure:**
     - Enable balancing: `ros2 topic pub /olaf/base/enable_balancing std_msgs/Bool "data: true"`
     - Robot balances autonomously for 30 minutes
     - Command random movements every 2 minutes:
       - Forward 0.3 m/s for 5 seconds
       - Backward 0.3 m/s for 5 seconds
       - Turn 0.5 rad/s for 3 seconds
       - Stop (0 velocity) for 10 seconds
   - **Success Criteria:**
     - No falls (kickstand not deployed except manual parking)
     - Pitch angle: ±10° maximum deviation
     - No motor stalling or overheating
     - Battery voltage > 33V at end of test
     - Odometry drift < 10% of distance traveled

4. **Performance Metrics:**
   - **PID Loop Frequency:**
     - Log to serial: Average loop frequency over 30 minutes
     - Target: 200Hz ±2Hz
   - **Fall Count:**
     - Zero falls in 30-minute test
   - **Battery Consumption:**
     - Start voltage: ~42V
     - End voltage: >35V (7V drop = ~16% capacity used)
   - **Odometry Accuracy:**
     - Drive 3m forward, measure actual distance
     - Error < 15cm (5%)

5. **Disturbance Rejection Test:**
   - While balancing, apply disturbances:
     - Gentle push forward: Recovery < 2 seconds
     - Gentle push backward: Recovery < 2 seconds
     - Place 1kg weight on top: Maintains balance
   - Verify robot doesn't fall from reasonable disturbances

6. **Navigation Command Test:**
   - Publish navigation goals (manual cmd_vel):
     - Move to position (1, 0): Drive forward 1m
     - Move to position (1, 1): Drive forward, turn, drive left
     - Return to origin (0, 0): Navigate back
   - Verify: Odometry tracks movement (±20% error acceptable without SLAM)

7. **Fail-Safe Testing:**
   - **Low Battery Test:**
     - Discharge battery to 32V
     - Verify: Kickstand deploys automatically, motors disable
   - **Manual Fall Test:**
     - Tilt robot beyond 45°
     - Verify: Kickstand deploys, motors disable, status = FALLEN
   - **Recovery Test:**
     - After fall, send reset command
     - Verify: Kickstand retracts, balancing resumes (if vertical)

8. **Thermal Testing:**
   - After 30-minute test, measure temperatures:
     - ODrive heatsink: <70°C
     - Motors: <60°C (touch test)
     - Battery: <45°C
     - Base Module ESP32: <65°C

9. **Documentation:**
   - Test results: `tests/integration/epic6_base_validation_results.md`
   - Performance metrics table
   - Video: 30-second clip showing balancing, movement, and recovery from push
   - Photos: Complete base assembly

**Dependencies:** All Epic 6 stories (6.1-6.9)

**Estimated Effort:** 3-4 hours test execution + 2-3 hours documentation + 4-8 hours debugging if issues found

**Note:** This is the Epic 6 safety gate. Must pass before Epic 7 (full robot integration with moving base).

---

## Story 6.11: Epic 6 Documentation & Build-in-Public Content

**As a** future contributor or follower,
**I want** comprehensive Epic 6 documentation and build-in-public content,
**so that** I can replicate the self-balancing base and follow the journey.

### Acceptance Criteria:

1. **Technical Documentation:**
   - `firmware/base/README.md` created:
     - PID controller architecture
     - MPU6050 sensor fusion (complementary filter)
     - ODrive UART protocol
     - I2C register map (0x10-0x37)
     - Firmware build/upload process
   - `hardware/base-assembly/README.md` created:
     - Motor mounting procedure
     - Wheel alignment process
     - ODrive wiring diagram
     - IMU orientation guide
     - Kickstand assembly
   - `ros2/src/orchestrator/ros2_nodes/README.md` updated:
     - Base driver usage
     - Odometry calculation formulas
     - cmd_vel topic specification

2. **Hardware Documentation:**
   - BOM: `hardware/bom/epic6_bom.csv`
     - 2× Hoverboard motors
     - 2× Hoverboard wheels (6.5")
     - ODrive v3.6
     - MPU6050 IMU
     - Feetech STS3215 (kickstand servo)
     - 3D printed parts list with print times
     - Supplier links, costs

3. **Wiring Diagrams:**
   - `hardware/wiring/base-motor-wiring.png` (ODrive → Motors)
   - `hardware/wiring/base-complete-wiring.png` (Full base system)

4. **Assembly Guides:**
   - `hardware/assembly/base-assembly-guide.md`

5. **Photos/Videos:**
   - Photos of all assemblies
   - Videos: Balancing demo, PID tuning process, kickstand deployment

6. **GitHub Milestone:**
   - All Epic 6 stories closed
   - Milestone marked complete

7. **Build-in-Public Content:**
   - LinkedIn post: "Olaf Can Balance! ⚖️🤖"

**Dependencies:** All Epic 6 stories complete (6.1-6.10)

**Estimated Effort:** 8-10 hours

---

## Epic 6 Summary

**Total Stories:** 11

**Estimated Total Effort:** 82-116 hours (3-4 weeks for solo builder)

**Key Deliverables:**
- ✅ Complete 3D-printed base platform
- ✅ **Base Module:** ESP32-S3 + MPU6050 IMU + ODrive motor control at I2C 0x0B
- ✅ Self-balancing system: 200Hz PID loop, complementary filter sensor fusion
- ✅ Hoverboard motors + wheels: Dual motor differential drive
- ✅ ODrive v3.6: Hall sensor commutation, velocity control
- ✅ Kickstand servo: Automatic fall protection
- ✅ ROS2 base driver: cmd_vel control, odometry publishing
- ✅ Odometry integration: Wheel encoder data for SLAM (Epic 10)
- ✅ 30-minute balancing validation
- ✅ Complete documentation + build-in-public content

**Architecture Validated:**
- ✅ Base Module I2C integration (4 modules: Head 0x08, Ears+Neck 0x09, Body 0x0A, Base 0x0B)
- ✅ Real-time PID control (200Hz loop on ESP32-S3)
- ✅ IMU sensor fusion (complementary filter)
- ✅ Differential drive kinematics
- ✅ Odometry for navigation stack
- ✅ Fall detection and kickstand deployment
- ✅ ROS2 cmd_vel standard interface

**Success Criteria Met:**
- Self-balancing reliable (30-minute test without falls)
- Velocity control responsive (cmd_vel → movement <500ms)
- Odometry accurate (±5-10% error without SLAM)
- Fall protection functional (kickstand deploys on fall)
- Foundation ready for Epic 7 (full robot integration)
- Navigation-ready for Epic 10 (SLAM integration)

**Next Epic:** Epic 7: Full Robot Integration & Cable Management

**Note:** Epic 6 establishes mobility. Epic 7 integrates head (Epic 3) + torso (Epic 5) + base (Epic 6) into complete robot.

---

**Related Documents:**
- [PRD Epic List](epic-list.md) - Overall project roadmap
- [Epic 1: Foundation](epic-01-foundation.md) - I2C architecture foundation
- [Epic 3: Complete Head Assembly](epic-03-complete-head-assembly.md) - Head/Ears/Neck modules
- [Epic 5: Core Torso & Power System](epic-05-core-torso-power-system.md) - Torso/power/Body module
- [Architecture - Components](../architecture/components.md) - Base module specifications
- [Tech Stack](../architecture/tech-stack.md) - ESP32-S3, MPU6050, ODrive, motors
