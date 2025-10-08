# Modules

Each module is an independent hardware component powered by an ESP32 microcontroller acting as a ROS2 node.

## Module Overview

| Module | Function | DOF | Status |
|--------|----------|-----|--------|
| **Head** | Vision (RGBD camera with IMU), human presence detection | 0 | 🔴 Not Started |
| **Ears** | Expression, directional attention (Chappie-inspired) | 2 per ear (4 total) | 🔴 Not Started |
| **Neck** | Head orientation, expressive gestures | 3 (pan, tilt, roll) | 🔴 Not Started |
| **Projector** | Floor projection for information display | 0 | 🔴 Not Started |
| **Body** | LED status indicators, housing | 0 | 🔴 Not Started |
| **Base** | Mobility (hoverboard platform), navigation | 0 (differential drive) | 🔴 Not Started |

## Architecture Principles

### MECE (Mutually Exclusive, Collectively Exhaustive)
- Each module owns exactly one functional domain
- No overlapping responsibilities
- Together, modules provide complete functionality

### Independent Operation
- Each module testable in isolation
- Module failures don't cascade to others
- Development can proceed in parallel

### ROS2 Communication
- Each ESP32 hosts a ROS2 node
- Standardized topics for inter-module messaging
- I2C for hardware connectivity

## Development Status

Modules will be developed over 4 months in weekend sprints. Check individual module folders for detailed specifications, code, and build guides as they're completed.
