# Epic 4: Base Module Build - Story Index

**Epic Goal:** Complete the Base module (I2C 0x0B) with self-balancing two-wheel platform using hoverboard components (36V battery, BLDC motors) mounted on skateboard suspension (iron trucks), ODrive motor controller, MPU6050 IMU, 200Hz PID control loop, power distribution system (36V → 12V/5V/3.3V), ESP32 firmware, ROS2 driver node, and assembly to robot frame. This module provides mobility, balancing, and powers all other modules.

**Status:** Not Started
**Total Stories:** 12
**Estimated Effort:** 5-7 weeks (part-time)

---

## Stories

### [Story 4.1: Source and Disassemble Hoverboard for Parts](story-4.1-source-hoverboard.md)
**Priority:** High | **Effort:** 8-12 hours
- Purchase or source hoverboard (used/new)
- Extract 2× hoverboard BLDC motors (typically 350W each, 6.5" wheels)
- Extract 36V battery pack (10S Li-ion, 4-5Ah)
- Document motor specifications (voltage, KV, hall sensor wiring, phase wires)
- Document battery specifications (voltage, capacity, discharge rating, BMS)
- Test extracted parts (motors spin freely, battery holds voltage)
- **Outcome:** Hoverboard components ready for integration

### [Story 4.2: Breadboard Base Components and Test Connectivity](story-4.2-breadboard-base.md)
**Priority:** High | **Effort:** 12-16 hours
- Breadboard ESP32 with power supply
- Wire MPU6050 IMU (I2C) and verify 200Hz readings
- Wire ODrive S1 to 36V battery (power switch and fuse)
- Connect 1× hoverboard motor to ODrive (phase wires, hall sensors)
- Establish UART connection between ESP32 and ODrive (115200 baud)
- Test ODrive commanding motor velocity and position
- Verify IMU 200Hz timing
- **Outcome:** All components verified working before PCB design

### [Story 4.3: Design Power Distribution System for 36V Battery](story-4.3-design-power-distribution.md)
**Priority:** High | **Effort:** 12-16 hours
- Calculate power budget for all modules
- Select buck converters (36V → 12V, 36V → 5V, 36V → 3.3V)
- Design power distribution topology
- Select wiring gauge based on current and cable length
- Design battery protection (fuse, low-voltage cutoff, voltage monitoring)
- Determine charge port location and type
- Include emergency stop switch in design
- Document design with schematic and power budget table
- **Outcome:** Power distribution system designed and documented

### [Story 4.4: Design Base Custom PCB in Fritzing](story-4.4-design-base-pcb.md)
**Priority:** High | **Effort:** 12-16 hours
- Create Fritzing schematic (ESP32, MPU6050, ODrive UART, buck converters, power distribution)
- Design PCB layout with HEAVY power traces for 36V/12V/5V rails
- Size power traces for maximum current draw (use 2oz copper)
- Include connectors for ODrive, motors, IMU, module power outputs, battery, charge port, I2C bus
- Include fuse holder footprint on 36V line
- Document pin assignments in wiring.md
- Review for electrical correctness (thermal relief, EMI considerations)
- Export Gerber files for manufacturing
- **Outcome:** PCB design ready for Elecrow order

### [Story 4.5: Order and Receive Base PCB from Elecrow](story-4.5-order-base-pcb.md)
**Priority:** High | **Effort:** 2-3 weeks lead time
- Upload Gerber files to Elecrow
- Configure PCB specifications (layers, thickness, color, quantity, 2oz copper)
- Place order and track shipment
- Visual inspection on arrival
- Continuity test (no shorts between power rails)
- **Outcome:** Manufactured PCB received and inspected

### [Story 4.6: Assemble and Test Base PCB](story-4.6-assemble-base-pcb.md)
**Priority:** High | **Effort:** 10-14 hours
- Solder all components (ESP32, buck converters, connectors, fuse holder, passives)
- Visual inspection (proper solder joints)
- CRITICAL SAFETY TEST: Power-on with bench power supply (36V, 1A current limit) to verify no shorts
- Power-on test WITH battery: verify stable 12V, 5V, 3.3V rails
- I2C address configuration (0x0B)
- Component-level tests (IMU readings, ODrive responds, motors spin, all power rails stable under load)
- **Outcome:** Functional PCB ready for firmware

### [Story 4.7: Assemble Base Platform with Skateboard Suspension](story-4.7-assemble-base-platform.md)
**Priority:** High | **Effort:** 12-16 hours
- Source skateboard suspension/trucks (iron construction)
- Mount hoverboard wheels to skateboard trucks
- Design/fabricate base platform for PCB, ODrive, battery
- Create battery mounting bracket with secure retention
- Include mounting points for Torso module connection
- Ensure proper weight distribution (battery low, centered)
- Plan cable routing for motor phase wires, power distribution, I2C bus
- Mount all components (PCB, ODrive, battery) to platform
- Secure platform to skateboard suspension
- Verify mechanical stability, wheel alignment, no wobble
- **Outcome:** Base platform mechanically assembled

### [Story 4.8: Configure ODrive for Hoverboard Motors](story-4.8-configure-odrive.md)
**Priority:** High | **Effort:** 8-12 hours
- Update ODrive firmware to latest stable version
- Configure motor parameters (pole pairs, resistance, inductance)
- Configure encoder (hall sensor mode, CPR)
- Configure current limits (continuous and peak)
- Configure velocity limits (maximum RPM)
- Test control modes (velocity control and position control)
- Complete motor calibration (CLOSED_LOOP_CONTROL state)
- Document UART communication protocol
- Save configuration to ODrive non-volatile memory
- **Outcome:** ODrive configured for hoverboard motors

### [Story 4.9: Develop Base ESP32 Firmware with 200Hz Balancing PID](story-4.9-develop-base-firmware.md)
**Priority:** High | **Effort:** 20-30 hours
- Create PlatformIO project
- Implement I2C slave interface (address 0x0B)
- Implement MPU6050 driver (200Hz hardware timer interrupt)
- Implement complementary filter or Kalman filter for angle estimation
- Implement PID controller (tilt angle error → motor velocity commands)
- Implement ODrive UART driver (send velocity commands at 200Hz)
- Implement basic PID tuning (balance for 10+ seconds)
- Implement I2C command interface (SET_TARGET_VELOCITY, ENABLE/DISABLE_BALANCING, GET_IMU_DATA, GET_ODOMETRY)
- Implement safety features (tilt threshold shutdown, watchdog timer)
- Test firmware (robot balances when powered on)
- **Outcome:** ESP32 firmware with working balancing capability

### [Story 4.10: Fine-Tune Self-Balancing PID Parameters](story-4.10-tune-pid.md)
**Priority:** High | **Effort:** 8-12 hours
- Document PID tuning methodology
- Tune proportional gain (Kp) for responsive tilt correction
- Tune integral gain (Ki) to eliminate steady-state error
- Tune derivative gain (Kd) for oscillation damping
- Test balancing on flat surface (60+ seconds consistently)
- Test recovery from small external disturbances
- Implement battery voltage compensation (optional)
- Document final PID parameters
- Record balancing behavior video
- **Outcome:** Stable, responsive balancing achieved

### [Story 4.11: Create Base ROS2 Driver Node](story-4.11-create-base-ros2-driver.md)
**Priority:** High | **Effort:** 8-12 hours
- Create Python ROS2 node in olaf_drivers/base_driver
- Implement topic subscription (/base/cmd_vel for navigation)
- Implement topic publishing (/base/imu, /base/odom)
- Implement I2C communication using smbus2
- Translate ROS2 messages to I2C register writes
- Test with manual topic publishes (robot moves while balancing)
- **Outcome:** ROS2 driver node controlling module via I2C

### [Story 4.12: Mount Base Module and Connect All Modules](story-4.12-mount-base-module.md)
**Priority:** High | **Effort:** 10-14 hours
- Mount both hoverboard motors securely with wheels attached
- Mount 36V battery pack securely with access to charge port
- Mount Base PCB and ODrive controller to platform
- Connect all motor phase wires and hall sensor cables to ODrive
- Route power cables from Base power distribution to all 4 modules
- Route I2C bus cables from Pi (in Torso) to all 4 modules including Base ESP32
- Ensure main power switch and emergency stop are accessible
- Ensure cable management (no wires near wheels or moving parts)
- Power up Base module and verify balancing mode
- Verify all modules powered and responding (i2cdetect shows 0x08-0x0B)
- Test robot balancing autonomously when placed upright
- Conduct battery runtime test (measure vs. 2-4 hour target)
- **Outcome:** Base module fully integrated, robot balances autonomously

---

## Epic Completion Criteria

- [ ] Hoverboard components sourced and tested
- [ ] All components breadboarded and tested
- [ ] Power distribution system designed and documented
- [ ] PCB designed in Fritzing and Gerber files exported (2oz copper)
- [ ] PCB ordered from Elecrow and received
- [ ] PCB assembled with all components soldered
- [ ] CRITICAL SAFETY TESTS passed (no shorts on 36V rail)
- [ ] Base platform assembled with skateboard suspension
- [ ] Hoverboard wheels mounted with proper alignment
- [ ] ODrive configured and calibrated for hoverboard motors
- [ ] ESP32 firmware implements I2C slave (0x0B)
- [ ] 200Hz PID balancing loop operational and tuned
- [ ] ROS2 driver node created and functional
- [ ] All 4 modules connected and powered from Base
- [ ] Robot balances autonomously when placed upright
- [ ] Robot responds to navigation commands: `ros2 topic pub /base/cmd_vel ...`
- [ ] Battery runtime meets 2-4 hour target

---

## Key Components

- [ ] 2× Hoverboard BLDC motors (typically 350W each, 6.5" wheels)
- [ ] 36V hoverboard battery pack (10S Li-ion, 4-5Ah)
- [ ] ODrive S1 motor controller (or equivalent, 36V input, dual motor control)
- [ ] ESP32-WROOM-32 development board
- [ ] MPU6050 IMU (I2C) for 200Hz balancing loop
- [ ] Buck converters (36V → 12V, 36V → 5V, 36V → 3.3V)
- [ ] Skateboard suspension/trucks (iron construction)
- [ ] Fuse/circuit breaker for 36V line
- [ ] Emergency stop switch
- [ ] Power connectors for 4 modules
- [ ] Charge port for battery

---

## Dependencies

**Before Epic 4:**
- Epic 0: ROS2 Foundation Setup ✅

**After Epic 4:**
- Base module provides power distribution to all other modules
- Base module provides mobility and self-balancing capability
- Epic 1, 2, 3 can receive power and be fully integrated
- Epic 5 (End-to-End Demo) requires Base module completion

---

## Notes

- **Build Order:** Base can be built first (after Epic 0) to establish power distribution, enabling other modules to be powered incrementally
- **SAFETY CRITICAL:** Base module handles 36V battery and high current. Follow all safety protocols for testing and assembly
- **PCB Lead Time:** 2-3 weeks from Elecrow (China shipping). Specify 2oz copper for high-current power traces
- **Power Distribution:** Base module is the power hub. All other modules receive power from Base
- **Hoverboard Components:** Proven platform for self-balancing robots. Motors and battery are widely available used or new
- **Balancing:** 200Hz PID loop provides basic balancing. Advanced features (adaptive gains, disturbance rejection) deferred to Phase 2
- **Testing:** Base module can be tested standalone before integrating other modules
- **Battery Runtime:** Target 2-4 hours under typical load. Actual runtime depends on usage patterns
- **Skateboard Suspension:** Iron trucks provide durable mounting system for hoverboard wheels

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
