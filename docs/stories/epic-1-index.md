# Epic 1: Head+Ears Module Build - Story Index

**Epic Goal:** Complete the Head+Ears module (I2C 0x08) with full hardware integration, ESP32 firmware, ROS2 driver node, and physical mounting. Module will control 2× OLED eyes (128×64 SPI), 2× articulated ears (2-DOF UART servos each), floor projector power/focus control, and integrate OAK-D-Pro RGBD camera.

**Status:** Not Started
**Total Stories:** 9
**Estimated Effort:** 3-5 weeks (part-time)

---

## Stories

### [Story 1.1: Breadboard Head+Ears Components and Test Connectivity](story-1.1-breadboard-head-ears.md)
**Priority:** High | **Effort:** 8-12 hours
- Breadboard ESP32 with all components
- Test 2× OLED displays (SPI, animations)
- Test 4× ear servos (2-DOF per ear, Feetech UART)
- Test projector power circuit (optocoupler)
- Test projector focus servo
- Verify OAK-D-Pro camera on separate Pi test
- **Outcome:** All components verified working before PCB design

### [Story 1.2: Design Head+Ears Custom PCB in Fritzing](story-1.2-design-pcb.md)
**Priority:** High | **Effort:** 6-10 hours
- Create Fritzing schematic (ESP32, OLEDs, servos, optocoupler, focus servo)
- Design PCB layout with proper trace routing
- Document pin assignments in wiring.md
- Export Gerber files for manufacturing
- **Outcome:** PCB design ready for Elecrow order

### [Story 1.3: Order and Receive Head+Ears PCB from Elecrow](story-1.3-order-pcb.md)
**Priority:** High | **Effort:** 2-3 weeks lead time
- Upload Gerber files to Elecrow
- Configure PCB specifications (layers, thickness, color, quantity)
- Place order and track shipment
- Visual inspection on arrival
- **Outcome:** Manufactured PCB received and inspected

### [Story 1.4: Assemble and Test Head+Ears PCB](story-1.4-assemble-pcb.md)
**Priority:** High | **Effort:** 6-8 hours
- Solder all components to PCB
- Power-on test (voltage rails verification)
- Component-level tests (OLEDs, servos, projector circuit)
- I2C address configuration (0x08)
- **Outcome:** Functional PCB ready for firmware

### [Story 1.5: Design and 3D Print Head+Ears Enclosure](story-1.5-design-enclosure.md)
**Priority:** Medium | **Effort:** 12-16 hours
- Design enclosure in OnShape (PCB, OLED mounts, ear brackets, projector mount, camera mount)
- Export STL files
- 3D print parts with appropriate settings
- Test fit all components
- **Outcome:** 3D printed enclosure ready for assembly

### [Story 1.6: Develop Head+Ears ESP32 Firmware](story-1.6-develop-firmware.md)
**Priority:** High | **Effort:** 16-24 hours
- Create PlatformIO project
- Implement I2C slave interface (address 0x08)
- Implement OLED driver (30+ FPS animations)
- Implement ear servo driver (4 servos via UART)
- Implement projector control (GPIO power, PWM/UART focus)
- Test I2C commands from Pi
- **Outcome:** ESP32 firmware responding to I2C commands

### [Story 1.7: Create Head+Ears ROS2 Driver Node](story-1.7-create-ros2-driver.md)
**Priority:** High | **Effort:** 8-12 hours
- Create Python ROS2 node in olaf_drivers/head_ears_driver
- Implement topic subscriptions (/head_ears/eyes, /ears, /projector)
- Implement I2C communication using smbus2
- Translate ROS2 messages to I2C register writes
- Test with manual topic publishes
- **Outcome:** ROS2 driver node controlling module via I2C

### [Story 1.8: Mount Head+Ears Module to Robot Frame](story-1.8-mount-module.md)
**Priority:** High | **Effort:** 4-6 hours
- Mount module to robot frame using 3D printed brackets
- Position OLEDs for "eyes" appearance
- Mount ear servos with full range of motion
- Mount projector and camera
- Connect I2C and power cables from Base module
- Verify ROS2 control after mounting
- **Outcome:** Head+Ears module integrated on robot

### [Story 1.9: Mount Projector with 3.7V Buck Converter](story-1.9-mount-projector-buck-converter.md)
**Priority:** High | **Effort:** 6-10 hours
- Select and source 3.7V buck converter (adjustable, 1-2A)
- Fabricate buck converter mounting bracket (metal or 3D printed)
- Adjust buck converter output to exactly 3.7V
- Fabricate projector power cable with proper connector
- Integrate optocoupler circuit for ESP32 on/off control
- Mount and connect all components with thermal management
- Test projector operation and video display from Pi
- **Outcome:** Projector operational with regulated 3.7V power

---

## Epic Completion Criteria

- [ ] All components breadboarded and tested
- [ ] PCB designed in Fritzing and Gerber files exported
- [ ] PCB ordered from Elecrow and received
- [ ] PCB assembled with all components soldered
- [ ] 3D enclosure designed and printed
- [ ] ESP32 firmware implements I2C slave (0x08)
- [ ] Firmware controls OLEDs (30 FPS), ears (4 servos), projector (power/focus)
- [ ] ROS2 driver node created and functional
- [ ] 3.7V buck converter mounted and adjusted for projector power
- [ ] Projector operational with regulated power and ESP32 control
- [ ] Module mounted to robot frame
- [ ] Module responds to ROS2 topic publishes: `ros2 topic pub /head_ears/eyes ...`

---

## Key Components

- [ ] ESP32-WROOM-32 development board
- [ ] 2× OLED displays (128×64, SPI interface)
- [ ] 4× Feetech servos for ears (2-DOF per ear, UART)
- [ ] Floor projector (HDMI from Pi, power/focus control from ESP32)
- [ ] 3.7V buck converter (adjustable, 1-2A) for projector power
- [ ] Optocoupler for projector power switching
- [ ] Linear servo for projector focus (PWM/UART)
- [ ] OAK-D-Pro RGBD camera (USB to Pi)

---

## Dependencies

**Before Epic 1:**
- Epic 0: ROS2 Foundation Setup ✅

**After Epic 1:**
- Module can be used independently with ROS2 commands
- Provides visual feedback for testing other modules
- Epic 2, 3, 4 can proceed in any order

---

## Notes

- **Build Order:** Head+Ears recommended first for visible progress (eyes blinking!)
- **PCB Lead Time:** 2-3 weeks from Elecrow (China shipping)
- **Camera Integration:** OAK-D-Pro connects to Pi via USB, not ESP32
- **Projector Control:** Pi sends HDMI video, ESP32 controls power and focus
- **Testing:** Module can be tested standalone before full robot assembly

---

**Created:** 2025-12-16
**Last Updated:** 2025-12-16
