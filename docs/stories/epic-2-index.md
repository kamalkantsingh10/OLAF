# Epic 2: Neck Module Build - Story Index

**Epic Goal:** Complete the Neck module (I2C 0x09) with 3-DOF servo array (pan/tilt/roll), kickstand servo (daisy-chained via UART), 2× presence sensors for 360° human detection, ESP32 firmware, ROS2 driver node, and physical mounting. This module enables expressive head gestures and stationary/balancing mode transitions.

**Status:** Not Started
**Total Stories:** 9
**Estimated Effort:** 3-4 weeks (part-time)

---

## Stories

### [Story 2.1: Breadboard Neck Components and Test Connectivity](story-2.1-breadboard-neck.md)
**Priority:** High | **Effort:** 8-12 hours
- Breadboard ESP32 with all components
- Test 4× Feetech STS3215 servos (daisy-chained UART)
- Test 2× presence sensors (mmWave or PIR)
- Verify servo daisy-chain communication (unique IDs)
- Document servo ID configuration
- **Outcome:** All components verified working before PCB design

### [Story 2.2: Design Neck Custom PCB in Fritzing](story-2.2-design-neck-pcb.md)
**Priority:** High | **Effort:** 6-10 hours
- Create Fritzing schematic (ESP32, servo UART bus, presence sensors)
- Design PCB layout with proper trace routing
- Size power traces for servo current draw
- Document pin assignments in wiring.md
- Export Gerber files for manufacturing
- **Outcome:** PCB design ready for Elecrow order

### [Story 2.3: Order and Receive Neck PCB from Elecrow](story-2.3-order-neck-pcb.md)
**Priority:** High | **Effort:** 2-3 weeks lead time
- Upload Gerber files to Elecrow
- Configure PCB specifications (layers, thickness, color, quantity)
- Place order and track shipment
- Visual inspection on arrival
- **Outcome:** Manufactured PCB received and inspected

### [Story 2.4: Assemble and Test Neck PCB](story-2.4-assemble-neck-pcb.md)
**Priority:** High | **Effort:** 6-8 hours
- Solder all components to PCB
- Power-on test (voltage rails verification)
- Component-level tests (servos, presence sensors)
- I2C address configuration (0x09)
- **Outcome:** Functional PCB ready for firmware

### [Story 2.5: Design and 3D Print Neck Enclosure and Servo Mounts](story-2.5-design-neck-enclosure.md)
**Priority:** Medium | **Effort:** 12-16 hours
- Design enclosure in OnShape (PCB, 3-DOF servo mounting, kickstand mount, sensor mounts)
- Design mechanical linkages for pan/tilt/roll motion
- Ensure full range of motion without interference
- Export STL files
- 3D print parts with appropriate settings
- Test fit all components
- **Outcome:** 3D printed enclosure ready for assembly

### [Story 2.6: Design Kickstand Mechanism with Single Servo](story-2.6-design-kickstand-mechanism.md)
**Priority:** High | **Effort:** 12-16 hours
- Design kickstand mechanism in OnShape (servo mount, linkage, ground pad)
- Design for single servo deploy/retract motion
- Calculate load bearing capacity
- Specify metal components (aluminum/steel rod, metal ground pad)
- Export STL files for 3D printed components
- Fabricate/source metal parts
- Test mechanism with servo under load
- **Outcome:** Kickstand mechanism designed and tested

### [Story 2.7: Develop Neck ESP32 Firmware](story-2.7-develop-neck-firmware.md)
**Priority:** High | **Effort:** 12-16 hours
- Create PlatformIO project
- Implement I2C slave interface (address 0x09)
- Implement servo driver (4 servos on daisy-chain UART)
- Implement presence sensor driver
- Implement smooth motion curves (optional)
- Test I2C commands from Pi
- **Outcome:** ESP32 firmware responding to I2C commands

### [Story 2.8: Create Neck ROS2 Driver Node](story-2.8-create-neck-ros2-driver.md)
**Priority:** High | **Effort:** 6-8 hours
- Create Python ROS2 node in olaf_drivers/neck_driver
- Implement topic subscriptions (/neck/position, /neck/kickstand)
- Implement topic publishing (/neck/presence)
- Implement I2C communication using smbus2
- Translate ROS2 messages to I2C register writes
- Test with manual topic publishes
- **Outcome:** ROS2 driver node controlling module via I2C

### [Story 2.9: Mount Neck Module to Robot Frame](story-2.9-mount-neck-module.md)
**Priority:** High | **Effort:** 6-8 hours
- Mount module to robot frame (connecting Torso to Head+Ears)
- Position servos for full range of motion
- Verify kickstand deploys/retracts without obstruction
- Position presence sensors for 360° coverage
- Connect I2C and power cables from Base module
- Mechanical load test (support Head+Ears weight)
- Verify ROS2 control after mounting
- **Outcome:** Neck module integrated on robot

---

## Epic Completion Criteria

- [ ] All components breadboarded and tested
- [ ] PCB designed in Fritzing and Gerber files exported
- [ ] PCB ordered from Elecrow and received
- [ ] PCB assembled with all components soldered
- [ ] 3D enclosure and servo mounts designed and printed
- [ ] Kickstand mechanism designed with metal components and tested
- [ ] ESP32 firmware implements I2C slave (0x09)
- [ ] Firmware controls 4 servos (pan/tilt/roll + kickstand) and reads presence sensors
- [ ] ROS2 driver node created and functional
- [ ] Module mounted to robot frame
- [ ] Module responds to ROS2 topic publishes: `ros2 topic pub /neck/position ...`
- [ ] Kickstand deploys/retracts on command: `ros2 topic pub /neck/kickstand ...`
- [ ] Presence sensors publish detection data: `ros2 topic echo /neck/presence`

---

## Key Components

- [ ] ESP32-WROOM-32 development board
- [ ] 4× Feetech STS3215 servos (3 for pan/tilt/roll + 1 for kickstand, daisy-chained UART)
- [ ] 2× Presence sensors (mmWave or PIR) for 360° human detection
- [ ] Kickstand mechanism (3D printed parts + metal rod + metal ground pad)
- [ ] 3D printed enclosure, servo mounting brackets, linkages

---

## Dependencies

**Before Epic 2:**
- Epic 0: ROS2 Foundation Setup ✅

**After Epic 2:**
- Neck module provides expressive head gestures
- Kickstand enables stationary/balancing mode transitions
- Presence sensors enable 360° human detection
- Epic 1, 3, 4 can proceed in any order

---

## Notes

- **Build Order:** Neck can be built in any order after Epic 0. Recommended after Epic 1 (Head+Ears) or Epic 4 (Base)
- **PCB Lead Time:** 2-3 weeks from Elecrow (China shipping)
- **Servo Communication:** Feetech STS3215 use TTL serial (half-duplex UART) for daisy-chain control
- **Kickstand Design:** Includes both 3D printed components and metal parts (aluminum/steel rod, metal ground pad) for structural strength
- **Presence Sensors:** Position for 360° coverage (front + back or left + right)
- **Testing:** Module can be tested standalone before full robot assembly

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
