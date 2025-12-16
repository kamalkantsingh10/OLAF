# OLAF Modules

Complete subsystems for OLAF's four smart peripheral modules.

## Module Overview

Each module is a **self-contained unit** with its own:
- **Firmware** (ESP32 code)
- **Hardware** (PCB designs, 3D models)
- **Tests** (Module-specific integration tests)
- **Diagnostics** (Debug and calibration tools)
- **Scripts** (Build, flash, test automation)
- **Documentation** (Wiring, assembly guides)

## Modules

| Module | I2C Address | Key Components | Status |
|--------|-------------|----------------|--------|
| **[head-ears](head-ears/)** | 0x08 | 2× OLED eyes, 2× articulated ears, floor projector control | In Development |
| **[neck](neck/)** | 0x09 | 3-DOF servo array, 2× presence sensors | Designed |
| **[torso](torso/)** | 0x0A | Heart display, thermal printer, battery, Pi housing | In Development |
| **[base](base/)** | 0x0B | ODrive motors, MPU6050 IMU, self-balancing | Designed |

## Quick Start

### Build a Module

```bash
cd modules/head-ears/firmware
pio run
```

### Flash a Module

```bash
cd modules/head-ears/firmware
pio run --target upload
```

### Test a Module

```bash
cd modules/head-ears
pytest tests/
```

### Run Diagnostics

```bash
cd modules/head-ears
python3 diagnostics/projector_diagnostic.py
```

## Development Workflow

1. **Navigate to module:** `cd modules/head-ears`
2. **Everything is here:**
   - Firmware code: `firmware/src/`
   - Hardware designs: `hardware/pcb/`, `hardware/mechanical/`
   - Tests: `tests/`
   - Tools: `diagnostics/`, `scripts/`
   - Docs: `wiring.md`, `assembly.md`, `README.md`

3. **Build → Flash → Test:**
   ```bash
   cd firmware && pio run --target upload
   cd ../tests && pytest
   cd ../diagnostics && python3 module_health.py
   ```

## Shared Resources

**Location:** `modules/shared/`

- **Firmware:** I2C protocol, OTA handlers, utilities
- **Hardware:** KiCad libraries, common 3D parts

All modules include shared libraries:
```ini
# In platformio.ini
lib_extra_dirs = ../../shared/firmware
```

## Module Architecture

Each module follows the **Smart Peripheral Pattern**:

1. **I2C Slave Interface:** Receives commands from Raspberry Pi
2. **Hardware Drivers:** Direct control of sensors/actuators
3. **Animation Engine:** Local execution of motion/display patterns
4. **Real-Time Control:** Critical timing loops (e.g., 200Hz PID on base)

**Example:** Head+Ears receives `PROJECTOR_ON` via I2C, then autonomously controls power (optocoupler) and focus (servo) without further Pi intervention.

## System Integration

Modules communicate via:
- **I2C bus:** Commands from Pi, sensor data to Pi
- **ROS2 topics:** High-level semantic interface (via driver nodes in `ros2/src/olaf_drivers/`)
- **Shared ground:** All modules share common electrical ground

See `hardware/wiring/` for inter-module wiring diagrams.

## Adding a New Module

1. Create directory: `modules/new-module/`
2. Copy structure from existing module (e.g., `head-ears/`)
3. Update I2C address in `firmware/platformio.ini`
4. Create ROS2 driver in `ros2/src/olaf_drivers/new_module_driver/`
5. Document in `new-module/README.md`

## Resources

- **System Architecture:** See `docs/architecture.md`
- **I2C Protocol:** See `modules/shared/firmware/i2c-protocol/`
- **Wiring Guide:** See `docs/guides/wiring-guide.md` and `hardware/wiring/`
- **Module Documentation:** See each module's `README.md`
