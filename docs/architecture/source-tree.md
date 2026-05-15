# Source Tree - Quick Reference

> **For complete repository structure details, see:** [repository-structure.md](./repository-structure.md)

---

## Quick Navigation

### Module Development
```
modules/{module}/firmware/     # ESP32 firmware (src/, test/)
modules/{module}/hardware/     # PCB + 3D models + BOM
modules/{module}/assembly.md   # Assembly and testing guide
modules/{module}/wiring.md     # Wiring diagrams and pin assignments
```

### ROS2 Development
```
ros2/src/olaf_drivers/         # Hardware driver logic (head_ears/neck/base/torso)
ros2/src/olaf_bringup/         # Launch files
ros2/src/expression_engine/    # Phase 2: subscribes to olaf_companion's 4
                               #   canonical topics, renders on the body
                               #   (replaced olaf_personality — Phase 2 SCP)
ros2/src/olaf_ai/              # (deferred) STT/agents now in olaf_companion
```

### System Tools
```
tools/diagnostics/             # System health checks
tools/calibration/             # Cross-module calibration
scripts/build/                 # Build automation
scripts/flash/                 # Flash automation
```

### Documentation
```
docs/architecture/             # Architecture docs
docs/prd/                      # Requirements
docs/stories/                  # User stories
```

---

## Module I2C Map

| Module | Address | Components |
|--------|---------|------------|
| **Head+Ears** | `0x08` | 2× OLED eyes, 2× ear servos, RGBD camera, projector |
| **Neck** | `0x09` | 3× neck servos, kickstand, 2× mmWave sensors |
| **Torso** | `0x0A` | Heart LCD, thermal printer, power LEDs |
| **Base** | `0x0B` | BNO085 AHRS IMU, ODrive motor control |

---

## File Naming Conventions

| Type | Pattern | Example |
|------|---------|---------|
| Python | `snake_case.py` | `personality_coordinator.py` |
| C++ header | `snake_case.h` | `animation_engine.h` |
| C++ impl | `snake_case.cpp` | `animation_engine.cpp` |
| Arduino | `module.ino` | `head_ears.ino` |
| YAML | `kebab-case.yaml` | `ros2-params.yaml` |
| Markdown | `kebab-case.md` | `tech-stack.md` |
| 3D models | `module-part-v1.stl` | `head-housing-v1.2.stl` |

---

**For detailed structure:** See [repository-structure.md](./repository-structure.md)

**Last Updated:** 2025-12-18
