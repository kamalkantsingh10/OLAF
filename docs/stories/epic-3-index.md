# Epic 3: Torso Module Build - Story Index

**Epic Goal:** Complete the Torso module (I2C 0x0A) housing Raspberry Pi 5 + Hailo AI Kit, 2.8" square heart display (SPI), and thermal printer. Enclosure assembled from durable kitchen bin reinforced with metal flat bars. This module is the compute hub for the robot, hosting the ROS2 orchestration layer and personality display elements. Power is received from Base module via cable.

**Status:** Not Started
**Total Stories:** 9
**Estimated Effort:** 3-4 weeks (part-time)

---

## Stories

### [Story 3.1: Breadboard Torso Components and Test Connectivity](story-3.1-breadboard-torso.md)
**Priority:** High | **Effort:** 6-8 hours
- Breadboard ESP32 with all components
- Test 2.8" square display (SPI, beating heart animations)
- Test thermal printer (UART or parallel interface)
- **Outcome:** All components verified working before PCB design

### [Story 3.2: Design Torso Custom PCB in Fritzing](story-3.2-design-torso-pcb.md)
**Priority:** High | **Effort:** 4-6 hours
- Create Fritzing schematic (ESP32, 2.8" display, thermal printer)
- Design PCB layout with proper trace routing
- Include power input connector (5V/3.3V from Base)
- Include I2C connector (SDA, SCL, GND)
- Document pin assignments in wiring.md
- Export Gerber files for manufacturing
- **Outcome:** PCB design ready for Elecrow order

### [Story 3.3: Order and Receive Torso PCB from Elecrow](story-3.3-order-torso-pcb.md)
**Priority:** High | **Effort:** 2-3 weeks lead time
- Upload Gerber files to Elecrow
- Configure PCB specifications (layers, thickness, color, quantity)
- Place order and track shipment
- Visual inspection on arrival
- **Outcome:** Manufactured PCB received and inspected

### [Story 3.4: Assemble and Test Torso PCB](story-3.4-assemble-torso-pcb.md)
**Priority:** High | **Effort:** 4-6 hours
- Solder all components to PCB
- Power-on test using bench power supply (5V, 3.3V)
- Component-level tests (display, printer)
- I2C address configuration (0x0A)
- **Outcome:** Functional PCB ready for firmware

### [Story 3.5: Assemble Torso Enclosure with Kitchen Bin and Metal Reinforcement](story-3.5-assemble-torso-enclosure.md)
**Priority:** High | **Effort:** 12-16 hours
- Select kitchen bin with appropriate size
- Source metal flat bars for reinforcement
- Cut ventilation holes for Pi and Hailo thermal management
- Make cutouts for display, printer output, power input, cables
- Attach metal flat bars for structural reinforcement
- Fabricate mounting brackets for robot frame attachment
- Create internal mounting system for Pi, PCB, display, printer
- Test fit all components
- **Outcome:** Durable enclosure ready for component installation

### [Story 3.6: Develop Torso ESP32 Firmware](story-3.6-develop-torso-firmware.md)
**Priority:** High | **Effort:** 12-16 hours
- Create PlatformIO project
- Implement I2C slave interface (address 0x0A)
- Implement heart display driver (animated beating heart with variable rhythm)
- Implement thermal printer driver (text, simple graphics)
- Test I2C commands from Pi
- **Outcome:** ESP32 firmware responding to I2C commands

### [Story 3.7: Create Torso ROS2 Driver Node](story-3.7-create-ros2-driver.md)
**Priority:** High | **Effort:** 6-8 hours
- Create Python ROS2 node in olaf_drivers/torso_driver
- Implement topic subscriptions (/torso/heart, /torso/print)
- Implement I2C communication using smbus2
- Translate ROS2 messages to I2C register writes
- Test with manual topic publishes
- **Outcome:** ROS2 driver node controlling module via I2C

### [Story 3.8: Install and Configure Raspberry Pi 5 in Torso Module](story-3.8-install-pi5-torso.md)
**Priority:** High | **Effort:** 8-12 hours
- Mount Raspberry Pi 5 securely in Torso enclosure
- Install Hailo AI Kit on PCIe slot
- Route power cable from Base module to Pi power input
- Route OAK-D-Pro camera USB cable to Pi
- Connect speakerphone USB cable to Pi
- Connect HDMI cable from Pi to Head+Ears module
- Connect I2C bus (Pi GPIO to Torso PCB)
- Install SD card with ROS2 workspace
- Verify Pi boots and all USB devices recognized
- Verify I2C bus scan shows Torso ESP32 at 0x0A
- **Outcome:** Raspberry Pi 5 fully integrated in Torso module

### [Story 3.9: Mount Torso Module to Robot Frame](story-3.9-mount-torso-module.md)
**Priority:** High | **Effort:** 6-8 hours
- Mount module to robot frame (connects Neck above and Base below)
- Position heart display front-facing and visible
- Ensure thermal printer output slot is accessible
- Connect power cable from Base module
- Connect I2C bus cable to Base module I2C distribution
- Verify cable management (no loose wires, good airflow)
- Verify ROS2 control after mounting
- Monitor Pi thermal performance
- **Outcome:** Torso module integrated on robot

---

## Epic Completion Criteria

- [ ] All components breadboarded and tested
- [ ] PCB designed in Fritzing and Gerber files exported
- [ ] PCB ordered from Elecrow and received
- [ ] PCB assembled with all components soldered
- [ ] Kitchen bin enclosure assembled with metal reinforcement
- [ ] ESP32 firmware implements I2C slave (0x0A)
- [ ] Firmware controls heart display and thermal printer
- [ ] ROS2 driver node created and functional
- [ ] Raspberry Pi 5 + Hailo AI Kit installed in Torso
- [ ] Module mounted to robot frame
- [ ] Module responds to ROS2 topic publishes: `ros2 topic pub /torso/heart ...`
- [ ] Thermal printer prints on command: `ros2 topic pub /torso/print ...`
- [ ] Pi remains cool under normal operation

---

## Key Components

- [ ] ESP32-WROOM-32 development board
- [ ] 2.8" square display (SPI interface) for heart animations
- [ ] Thermal printer (UART or parallel interface)
- [ ] Raspberry Pi 5 16GB
- [ ] Hailo AI Kit (26 TOPS, PCIe)
- [ ] Kitchen bin enclosure (durable plastic)
- [ ] Metal flat bars (steel or aluminum) for structural reinforcement
- [ ] Speakerphone (USB)
- [ ] OAK-D-Pro RGBD camera (USB to Pi, camera mounted in Head+Ears)

---

## Dependencies

**Before Epic 3:**
- Epic 0: ROS2 Foundation Setup ✅

**After Epic 3:**
- Torso module provides compute hub (Raspberry Pi 5 + Hailo AI Kit)
- Heart display enables personality expression
- Thermal printer enables physical output
- Epic 1, 2, 4 can proceed in any order
- Epic 4 (Base) must provide power to Torso

---

## Notes

- **Build Order:** Torso can be built in any order after Epic 0. Recommended after Epic 4 (Base) to ensure power distribution is ready
- **PCB Lead Time:** 2-3 weeks from Elecrow (China shipping)
- **Enclosure:** Kitchen bin provides durable, practical housing. Metal flat bars add structural strength
- **Thermal Management:** Critical for Raspberry Pi 5 + Hailo AI Kit. Ensure adequate ventilation and consider heat sinks/fans
- **Power:** Torso receives power from Base module (5V for Pi, 5V/3.3V for ESP32 and peripherals)
- **I2C Master:** Raspberry Pi 5 in Torso acts as I2C master for all modules (0x08-0x0B)
- **Testing:** Module can be tested standalone with bench power supply before Base module is complete

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
