# Coding Standards

## Overview

This document defines coding standards for OLAF development across all languages and platforms. These standards ensure code quality, maintainability, safety, and consistency across the distributed embedded robotics system with hybrid AI intelligence.

**Scope:** Python (ROS2 orchestrator), C++ (ESP32 firmware), shell scripts, configuration files, documentation.

**Philosophy:** Prioritize safety, readability, and real-time performance. OLAF is a safety-critical system (self-balancing robot with motors) requiring defensive coding practices and clear documentation.

---

## General Principles

### Core Values

1. **Safety First** - Prevent hardware damage, battery over-discharge, unstable balancing, and motor control failures
2. **Real-Time Awareness** - Respect timing constraints (200Hz PID loops, <500ms expression sync, <100ms I2C latency)
3. **Defensive Coding** - Assume sensors fail, I2C timeouts occur, API calls fail, and modules disconnect
4. **Clear Over Clever** - Readable code beats micro-optimizations; future maintainers (including yourself) will thank you
5. **Test Everything** - Unit tests for logic, integration tests for communication, manual tests for physical behavior
6. **Document Intent** - Comments explain WHY, not WHAT; code should be self-documenting for WHAT

### Cross-Language Standards

**Naming Conventions:**
- **Files:** `snake_case.ext` (e.g., `personality_coordinator.py`, `balancing_controller.cpp`)
- **Constants:** `UPPER_SNAKE_CASE` (e.g., `MAX_VELOCITY_MPS`, `I2C_TIMEOUT_MS`)
- **Configuration files:** `kebab-case.yaml` (e.g., `module-config.yaml`, `ros2-params.yaml`)

**Magic Numbers:**
- ❌ **Never use magic numbers** directly in code
- ✅ **Always define named constants** with units in comments

```python
# Bad
if voltage < 30:
    shutdown()

# Good
BATTERY_CUTOFF_VOLTAGE_V = 30  # Volts - prevents over-discharge damage
if voltage < BATTERY_CUTOFF_VOLTAGE_V:
    shutdown()
```

**Error Handling:**
- **Always handle errors explicitly** - no silent failures
- **Log errors with context** (module name, operation, values)
- **Fail safe** - prefer safe stop over undefined behavior

**Version Control:**
- **Commit messages:** Follow Conventional Commits format
  - `feat: add Whisper STT integration with Hailo accelerator`
  - `fix: resolve I2C timeout in neck module during rapid pan movements`
  - `docs: update servo calibration procedure for STS3215`
  - `refactor: extract PID constants to config file`
  - `test: add unit tests for emotion intensity mapping`
- **Branch naming:** `feature/short-description`, `fix/issue-number-description`, `docs/topic`
- **Pull requests:** Include test results, hardware validation notes, before/after behavior

---

## Python Standards (ROS2 Orchestrator)

### Style Guide: PEP 8 with Specific Extensions

**Baseline:** Follow [PEP 8](https://peps.python.org/pep-0008/) with the following OLAF-specific rules.

**Line Length:**
- **Maximum 100 characters** (not 79) - allows readable ROS2 topic names and function signatures
- Break long lines at logical boundaries (after commas, before operators)

**Imports:**
- **Order:** Standard library → Third-party → ROS2 → Local modules
- **Style:** Absolute imports preferred, relative imports only within same package
- **One import per line** (except `from x import a, b` for related items)

```python
# Good
import os
import sys
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from olaf.personality.emotion import EmotionType, IntensityLevel
from olaf.hardware.i2c_client import I2CClient
```

**Naming:**
- **Classes:** `PascalCase` (e.g., `PersonalityCoordinator`, `HeadDriverNode`)
- **Functions/methods:** `snake_case` (e.g., `express_emotion()`, `get_battery_voltage()`)
- **Variables:** `snake_case` (e.g., `target_velocity_mps`, `emotion_intensity`)
- **Private members:** `_leading_underscore` (e.g., `_i2c_bus`, `_pid_gains`)
- **Constants:** `UPPER_SNAKE_CASE` (e.g., `MAX_EXPRESSION_INTENSITY`, `I2C_HEAD_ADDRESS`)

**Type Hints:**
- **Required for all public functions and methods**
- Use modern syntax (`list[int]` not `List[int]` in Python 3.11+)
- Optional for private methods (encouraged)

```python
# Good
def express_emotion(
    emotion_type: EmotionType,
    intensity: IntensityLevel,
    duration_ms: int = 2000
) -> bool:
    """Coordinate expression across eyes, ears, neck, beeps.

    Args:
        emotion_type: One of 7 emotion types (happy, curious, etc.)
        intensity: Level 1-5 (1=subtle, 5=extreme)
        duration_ms: Expression duration in milliseconds

    Returns:
        True if all modules synchronized within 500ms, False otherwise

    Raises:
        I2CTimeoutError: If any module fails to respond
    """
    # Implementation
```

**Docstrings:**
- **Required for all public classes, functions, modules**
- **Format:** Google style (used by ROS2 community)
- **Include:** Purpose, parameters, return values, exceptions, examples for complex functions

**Async/Await:**
- Use `async`/`await` for I2C operations to prevent blocking ROS2 event loop
- Timeout all async operations (never indefinite waits)

```python
# Good
async def read_sensor_data(i2c_address: int, timeout_ms: int = 100) -> Optional[bytes]:
    """Read sensor data from I2C module with timeout."""
    try:
        return await asyncio.wait_for(
            self._i2c_client.read(i2c_address, num_bytes=16),
            timeout=timeout_ms / 1000.0
        )
    except asyncio.TimeoutError:
        self.get_logger().error(f"I2C timeout reading from 0x{i2c_address:02X}")
        return None
```

**Error Handling:**
- **Catch specific exceptions**, not bare `except:`
- **Log with context** using ROS2 logger
- **Fail gracefully** - degrade functionality, don't crash orchestrator

```python
# Bad
try:
    servo_position = read_servo()
except:
    pass  # Silent failure!

# Good
try:
    servo_position = read_servo(NECK_PAN_SERVO_ID)
except I2CTimeoutError:
    self.get_logger().warning(
        f"Neck pan servo unresponsive, using cached position {self._last_pan_deg}°"
    )
    servo_position = self._last_pan_deg
except ValueError as e:
    self.get_logger().error(f"Invalid servo position data: {e}")
    servo_position = 0  # Safe default: centered
```

**Logging:**
- Use ROS2 logger (`self.get_logger()` in nodes)
- **Levels:**
  - `DEBUG`: Verbose I2C traffic, loop iterations (disabled in production)
  - `INFO`: State changes, module connections, command acknowledgments
  - `WARNING`: Recoverable errors, degraded functionality, timeouts
  - `ERROR`: Unrecoverable errors requiring intervention
- **Include units** in log messages (`"Battery: 34.2V"` not `"Battery: 34.2"`)

### ROS2-Specific Standards

**Node Naming:**
- **Format:** `<module>_<function>_node` (e.g., `head_driver_node`, `personality_coordinator_node`)
- **Namespace:** All nodes under `/olaf/` (e.g., `/olaf/head_driver`, `/olaf/ai_agent`)

**Topic Naming:**
- **Format:** `/<namespace>/<module>/<data_type>` (e.g., `/olaf/head/expression`, `/olaf/base/odometry`)
- **Use semantic names** not hardware-specific (e.g., `/olaf/emotion` not `/olaf/i2c_0x08`)

**Parameter Naming:**
- **Format:** `<module>.<parameter>` (e.g., `head.i2c_address`, `personality.sync_timeout_ms`)
- **Declare all parameters** with defaults and descriptions in launch files

**Message Design:**
- Prefer standard ROS2 messages (`geometry_msgs`, `sensor_msgs`) over custom when possible
- Custom messages: Use clear field names with units in comments

```python
# olaf_interfaces/msg/Expression.msg
uint8 emotion_type  # 0=neutral, 1=happy, 2=curious, 3=thinking, 4=confused, 5=sad, 6=excited
uint8 intensity     # 1-5 (1=subtle, 5=extreme)
uint16 duration_ms  # Expression duration in milliseconds
```

**Publisher/Subscriber Patterns:**
- **QoS profiles:** Use appropriate profiles (default for most, sensor data for high-frequency)
- **Callback thread safety:** Protect shared state with locks if callbacks modify data

### Testing Standards

**Framework:** pytest 7.4+

**Test Organization:**
- **Location:** `tests/unit/ros2/orchestrator/<module>/`
- **Naming:** `test_<module>.py` (e.g., `test_personality_coordinator.py`)
- **Structure:** One test class per production class

**Coverage:**
- **Target:** 70%+ for critical paths (personality, AI integration, navigation)
- **Required tests:**
  - Unit tests for emotion mapping logic
  - Integration tests for I2C communication (with mocked hardware)
  - Timeout/failure scenarios

**Fixtures:**
- Use pytest fixtures for ROS2 node setup, mocked I2C clients, test data
- Place shared fixtures in `tests/conftest.py`

```python
# tests/unit/ros2/orchestrator/personality/test_emotion_mapper.py
import pytest
from olaf.personality.emotion import EmotionMapper, EmotionType, IntensityLevel

class TestEmotionMapper:
    """Unit tests for emotion type and intensity mapping."""

    def test_sentiment_to_emotion_happy(self):
        """Positive sentiment with high confidence maps to happy emotion."""
        mapper = EmotionMapper()
        emotion = mapper.sentiment_to_emotion(sentiment=0.8, confidence=0.9)
        assert emotion == EmotionType.HAPPY

    def test_intensity_mapping_extreme(self):
        """Maximum intensity (1.0) maps to level 5."""
        mapper = EmotionMapper()
        intensity = mapper.normalize_intensity(value=1.0)
        assert intensity == IntensityLevel.EXTREME  # 5
```

---

## C++ Standards (ESP32 Firmware)

### Style Guide: Modified Google C++ Style Guide

**Baseline:** [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with embedded-specific modifications.

**Language Version:** C++17 (ESP-IDF and Arduino support)

**Line Length:** 100 characters maximum

**Naming:**
- **Classes/Structs:** `PascalCase` (e.g., `AnimationEngine`, `I2CSlave`)
- **Functions:** `camelCase` (e.g., `expressEmotion()`, `readSensorData()`)
- **Variables:** `snake_case` (e.g., `target_angle_deg`, `i2c_buffer`)
- **Constants:** `kPascalCase` or `UPPER_SNAKE_CASE` for macros (e.g., `kMaxServoAngle`, `I2C_SLAVE_ADDRESS`)
- **Private members:** `trailing_underscore_` (e.g., `servo_position_`, `last_update_millis_`)

**File Organization:**
- **Header files:** `.h` extension (e.g., `animation_engine.h`)
- **Implementation files:** `.cpp` extension (e.g., `animation_engine.cpp`)
- **Arduino sketches:** `.ino` extension (e.g., `head_controller.ino`)

**Header Guards:**
- Use `#pragma once` (simpler, compiler-optimized)

```cpp
// animation_engine.h
#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

class AnimationEngine {
 public:
  AnimationEngine(uint8_t screen_width, uint8_t screen_height);
  void renderEmotion(uint8_t emotion_type, uint8_t intensity);

 private:
  Adafruit_SSD1306 display_;
  uint32_t last_frame_millis_;
};
```

**Includes:**
- **Order:** System headers → Library headers → Local headers
- **Use angle brackets** for system/library (`<Arduino.h>`), quotes for local (`"animation_engine.h"`)

**Namespaces:**
- Avoid namespaces in Arduino sketches (not well-supported)
- If using ESP-IDF, use namespaces: `namespace olaf { namespace head { ... } }`

### Embedded-Specific Standards

**Memory Management:**
- ❌ **NEVER use dynamic allocation in real-time loops** (malloc, new, std::vector::push_back)
- ✅ **Use fixed-size arrays** or pre-allocate during setup()
- ✅ **Check available heap** with `ESP.getFreeHeap()` during development

```cpp
// Bad - dynamic allocation in 200Hz loop
void loop() {
  int* data = new int[100];  // Heap fragmentation risk!
  processPID(data);
  delete[] data;
}

// Good - pre-allocated buffer
constexpr size_t kBufferSize = 100;
int sensor_buffer[kBufferSize];  // Stack or global, allocated once

void loop() {
  readSensors(sensor_buffer, kBufferSize);
  processPID(sensor_buffer);
}
```

**Timing:**
- **Never use `delay()` in time-critical code** - breaks real-time responsiveness
- **Use `millis()` or hardware timers** for non-blocking timing
- **Document timing requirements** in comments (e.g., "Must execute in <5ms")

```cpp
// Bad - blocks entire CPU
void loop() {
  updateServos();
  delay(5);  // Blocks I2C interrupt handling!
}

// Good - non-blocking
constexpr uint32_t kUpdateIntervalMs = 5;
uint32_t last_update_millis = 0;

void loop() {
  uint32_t now = millis();
  if (now - last_update_millis >= kUpdateIntervalMs) {
    updateServos();
    last_update_millis = now;
  }
  handleI2CCommands();  // Can run every loop iteration
}
```

**Watchdog Timers:**
- **Enable watchdog** on safety-critical modules (Base balancing)
- **Feed watchdog** at end of each successful loop iteration
- **Timeout:** 1-2 seconds (long enough for normal operation, short enough to catch hangs)

```cpp
#include <esp_task_wdt.h>

constexpr uint32_t kWatchdogTimeoutSec = 2;

void setup() {
  esp_task_wdt_init(kWatchdogTimeoutSec, true);  // Enable panic on timeout
  esp_task_wdt_add(NULL);  // Add current task to watchdog
}

void loop() {
  updateBalancingPID();
  updateOdometry();

  esp_task_wdt_reset();  // Feed watchdog - "I'm alive!"
}
```

**I2C Slave Interrupt Handling:**
- **Keep interrupt handlers SHORT** (<10 microseconds)
- **Set flags, don't process** - defer work to main loop
- **Use `volatile` for shared variables** between ISR and main code

```cpp
// Good I2C interrupt pattern
volatile bool i2c_command_received = false;
volatile uint8_t i2c_command_buffer[16];

void onI2CReceive(int num_bytes) {
  // ISR - keep minimal!
  for (int i = 0; i < num_bytes && i < 16; i++) {
    i2c_command_buffer[i] = Wire.read();
  }
  i2c_command_received = true;  // Set flag, return FAST
}

void loop() {
  if (i2c_command_received) {
    processI2CCommand(i2c_command_buffer);  // Process in main loop
    i2c_command_received = false;
  }
}
```

**Constants and Configuration:**
- Define hardware-specific constants at top of file
- Use `constexpr` for compile-time constants (preferred over `#define`)

```cpp
// Head module configuration
constexpr uint8_t kI2CSlaveAddress = 0x08;
constexpr uint8_t kOLEDWidth = 128;
constexpr uint8_t kOLEDHeight = 64;
constexpr uint32_t kAnimationFPS = 30;
constexpr uint32_t kFrameTimeMs = 1000 / kAnimationFPS;  // 33ms
```

### Error Handling

**Serial Logging:**
- **Levels:** Use prefixes `[DEBUG]`, `[INFO]`, `[WARN]`, `[ERROR]`
- **Include module name** and context

```cpp
void setup() {
  Serial.begin(115200);

  if (!initOLEDDisplay()) {
    Serial.println("[ERROR] Head: Failed to initialize OLED display (check SPI wiring)");
    // Enter safe mode - blink LED error code
    enterSafeMode(ErrorCode::OLED_INIT_FAILED);
  }

  Serial.println("[INFO] Head: Initialization complete, I2C slave ready at 0x08");
}
```

**Fail-Safe Behaviors:**
- **Hardware init failures:** Enter safe mode (blink LED error pattern, respond to I2C status requests)
- **Invalid commands:** Ignore, log warning, respond with error code via I2C
- **Sensor failures:** Use last known good value, report degraded status

### Testing Standards

**Framework:** Arduino Unit Test (PlatformIO integration)

**Test Organization:**
- **Location:** `firmware/<module>/test/`
- **Naming:** `test_<component>.cpp` (e.g., `test_animation_engine.cpp`)

**Hardware-in-the-Loop:**
- Mock I2C master for testing slave responses
- Validate timing with oscilloscope (200Hz PID loop consistency)
- Servo calibration tests (verify angle ranges: pan ±90°, tilt ±45°, roll ±30°)

**Required Tests:**
- Watchdog timer triggers on infinite loop
- I2C command parsing (valid/invalid commands)
- Animation frame rate consistency (30+ FPS)
- PID loop stability (Base module: no oscillations within ±2° of target)

---

## Configuration and Data Files

### YAML Configuration

**Formatting:**
- **Indentation:** 2 spaces (no tabs)
- **Key naming:** `snake_case`
- **Comments:** Explain purpose and valid ranges

```yaml
# firmware/head/config.yaml
module:
  name: head
  i2c_address: 0x08

display:
  width: 128
  height: 64
  spi_speed_mhz: 20  # 20 MHz for 30-60 FPS animation

expressions:
  default_emotion: neutral  # One of: neutral, happy, curious, thinking, confused, sad, excited
  default_intensity: 2      # 1-5 (1=subtle, 5=extreme)
  sync_timeout_ms: 500      # Max time to coordinate with other modules
```

### Environment Variables

**File:** `.env` (excluded via `.gitignore`)

**Template:** `config/api-keys.template.env` (committed to repo)

```bash
# config/api-keys.template.env
# Copy to .env and fill in actual values

# Anthropic Claude API
ANTHROPIC_API_KEY=sk-ant-xxxxxxxxxxxxxxxxxxxxx

# OpenAI GPT-4 API (future)
# OPENAI_API_KEY=sk-xxxxxxxxxxxxxxxxxxxxx

# ROS2 Domain ID (0-101, isolates from other ROS2 systems on network)
ROS_DOMAIN_ID=42
```

**Loading:**
```python
from dotenv import load_dotenv
import os

load_dotenv()  # Load .env file
api_key = os.getenv("ANTHROPIC_API_KEY")
if not api_key:
    raise ValueError("ANTHROPIC_API_KEY not set in .env file")
```

---

## Documentation Standards

### Code Comments

**Purpose:** Explain WHY, not WHAT (code should be self-documenting for WHAT)

```cpp
// Bad - describes what code does (obvious from reading)
int battery_voltage = analogRead(BATTERY_PIN);  // Read battery voltage

// Good - explains why and provides context
int battery_voltage = analogRead(BATTERY_PIN);  // Monitor for auto-shutdown at 30V (over-discharge protection)
```

**Required Comments:**
- Hardware connections (pin assignments, I2C addresses)
- Timing constraints (loop frequency, timeout values)
- Safety-critical logic (emergency stops, fail-safe behaviors)
- Magic numbers/constants (even with named constants, explain units and rationale)
- Complex algorithms (PID tuning, emotion mapping formulas)

### README Files

**Required for each module:**
- **Purpose:** What the module does
- **Hardware:** Components, wiring diagram link, BOM link
- **Setup:** Installation, calibration, testing
- **API:** I2C register map, ROS2 topics/services
- **Troubleshooting:** Common issues and solutions

**Template:**
```markdown
# Head Module

## Purpose
Controls OLED eye displays, RGBD camera, microphone array, mmWave presence sensor.

## Hardware
- ESP32-DevKitC (ESP32-WROOM-32)
- 2× Adafruit SSD1306 OLED displays (128x64, SPI)
- Luxonis OAK-D Pro RGBD camera (USB to Pi)
- Microphone array (USB to Pi)
- mmWave sensor (24GHz, UART)

See [wiring diagram](../../hardware/wiring/head-module.png) and [BOM](../../hardware/bom/head-module.csv).

## Setup
1. Flash firmware: `pio run -t upload`
2. Calibrate displays: `python tools/calibration/calibrate_eyes.py`
3. Test standalone: `olaf-test head --emotion happy --intensity 3`

## I2C Register Map
| Address | Name | Type | Description |
|---------|------|------|-------------|
| 0x00 | STATUS | R | Module status (0=OK, 1=ERROR) |
| 0x01 | COMMAND | W | Command register (see below) |
| 0x10 | EXPRESSION_TYPE | W | Emotion type (0-6) |
| 0x11 | EXPRESSION_INTENSITY | W | Intensity (1-5) |

## Troubleshooting
- **OLED displays blank:** Check SPI wiring (MOSI, SCK, CS pins)
- **I2C timeouts:** Verify pull-up resistors (4.7kΩ on SDA/SCL)
```

### Build Guides

**Required documentation:**
- 3D print settings (layer height, infill, supports)
- Assembly order with photos
- Calibration procedures (servo ranges, PID tuning)
- Troubleshooting common build issues

---

## Git Workflow and Code Review

### Branch Strategy

**Main branches:**
- `main` - Stable releases only, always deployable
- `develop` - Integration branch, latest working code

**Feature branches:**
- `feature/<module>-<description>` (e.g., `feature/head-oled-animations`)
- `fix/<issue>-<description>` (e.g., `fix/23-i2c-timeout`)

**Workflow:**
1. Create feature branch from `develop`
2. Commit frequently with clear messages
3. Test locally (unit + integration + hardware validation)
4. Open pull request to `develop`
5. Code review (see checklist below)
6. Merge to `develop`, delete feature branch
7. Periodically merge `develop` → `main` for releases

### Code Review Checklist

**Reviewer must verify:**
- [ ] Tests pass (unit + integration)
- [ ] Hardware validated (if applicable)
- [ ] Coding standards followed
- [ ] No magic numbers (all constants named)
- [ ] Error handling present (no silent failures)
- [ ] Logging includes context (module, operation, values)
- [ ] Documentation updated (README, comments)
- [ ] No secrets committed (API keys, passwords)
- [ ] Memory management safe (no dynamic allocation in real-time code)
- [ ] Timing constraints met (profile critical loops if needed)

---

## Performance and Optimization

### Python Optimization

**Profiling:**
- Use `cProfile` for CPU profiling: `python -m cProfile -o profile.stats ros2/src/orchestrator/main.py`
- Visualize with `snakeviz`: `snakeviz profile.stats`

**ROS2 Performance:**
- Avoid synchronous service calls in loops (use async or actions)
- Batch I2C operations where possible (single transaction > multiple)
- Use QoS profiles appropriately (don't force reliability on sensor data)

### C++ Optimization

**Profiling:**
- Measure loop timing with `micros()` at start/end
- Log worst-case execution time for critical loops

```cpp
void loop() {
  uint32_t start_micros = micros();

  updateBalancingPID();  // Must complete in <5ms (200Hz)

  uint32_t elapsed_micros = micros() - start_micros;
  if (elapsed_micros > 5000) {  // 5ms = 5000 microseconds
    Serial.printf("[WARN] Base: PID loop exceeded 5ms budget: %lu us\n", elapsed_micros);
  }
}
```

**Compiler Flags:**
- Development: `-O0 -g` (no optimization, debug symbols)
- Production: `-O2` (optimize for speed without breaking debugging)
- Critical paths: Profile before using `-O3` or `-Ofast` (can break timing assumptions)

---

## Security and Safety

### API Key Management

- ✅ **Always use `.env` files** (never hardcode)
- ✅ **Add `.env` to `.gitignore`**
- ✅ **Provide `.template.env`** with placeholder values
- ❌ **Never commit secrets** to version control
- ✅ **Use pre-commit hooks** to prevent accidental commits:

```bash
# .git/hooks/pre-commit
#!/bin/bash
if git diff --cached --name-only | grep -q "\.env$"; then
  echo "ERROR: Attempted to commit .env file!"
  exit 1
fi
```

### Safety-Critical Code

**Base Module Balancing (200Hz PID):**
- Emergency stop if `|pitch| > 45°` (fall detected)
- Watchdog timer (2 sec timeout)
- Motor current limiting (prevent battery overdraw)
- Kickstand deployment on emergency stop

**Motor Control:**
- Velocity ramp limits (prevent jerky acceleration)
- Obstacle detection (stop if RGBD sees collision within 0.5m)
- Manual E-stop (physical button or emergency ROS2 topic)

**Battery Protection:**
- Voltage monitoring (shutdown at 30V cutoff)
- Current limiting (prevent BMS overcurrent trip)
- Low battery warnings (visual + audible at 32V)

---

## AI-Assisted Development Guidelines

**When using AI (Claude, GPT-4, Copilot) for code generation:**

1. **Review all generated code** - AI can make subtle errors (timing, memory management)
2. **Validate hardware assumptions** - AI may suggest incorrect pin assignments or I2C addresses
3. **Test rigorously** - AI-generated code requires same testing as human-written
4. **Document AI-generated sections** - Note in comments if substantial code is AI-generated (helps future debugging)
5. **Iterate with AI** - Provide error messages, timing measurements, hardware constraints for refinement

**Example AI workflow:**
1. Describe desired functionality with constraints (timing, memory, hardware)
2. Review generated code against these standards
3. Test on hardware, measure timing with oscilloscope/profiling
4. Provide feedback to AI if issues found
5. Commit final human-reviewed version

---

## Enforcement and Tooling

### Linting and Formatting

**Python:**
- **Linter:** `ruff` (modern, fast) or `pylint`
  - Config: `.ruff.toml` or `pyproject.toml` with line-length=100, PEP 8 rules
- **Formatter:** `black` with `--line-length 100`
- **Type checker:** `mypy` (strict mode for production code)

**C++:**
- **Formatter:** `clang-format` with Google style + modifications
  - Config: `.clang-format` in repo root
- **Linter:** `clang-tidy` (checks for common embedded pitfalls)

### Pre-commit Hooks

Install pre-commit framework: `pip install pre-commit`

**`.pre-commit-config.yaml`:**
```yaml
repos:
  - repo: https://github.com/psf/black
    rev: 23.3.0
    hooks:
      - id: black
        args: [--line-length=100]

  - repo: https://github.com/charliermarsh/ruff-pre-commit
    rev: v0.0.270
    hooks:
      - id: ruff

  - repo: local
    hooks:
      - id: no-secrets
        name: Check for .env files
        entry: bash -c 'git diff --cached --name-only | grep -q "\.env$" && exit 1 || exit 0'
        language: system
```

Run `pre-commit install` to enable automatic checks on every commit.

### CI/CD (Future)

**V1:** Manual testing before major commits

**V2:** GitHub Actions workflow
- Run unit tests on all PRs
- Check formatting (black, clang-format)
- Run linters (ruff, clang-tidy)
- Build ESP32 firmware (ensure no compilation errors)
- Hardware-in-the-loop tests (if feasible with remote hardware)

---

## References

**Python:**
- [PEP 8 - Style Guide for Python Code](https://peps.python.org/pep-0008/)
- [ROS 2 Python Style Guide](https://docs.ros.org/en/humble/Contributing/Code-Style-Language-Versions.html)
- [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html)

**C++:**
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [ESP32 Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [Arduino Style Guide](https://docs.arduino.cc/learn/contributions/arduino-library-style-guide)

**ROS2:**
- [ROS 2 Design Patterns](https://design.ros2.org/)
- [ROS 2 Quality Guide](https://docs.ros.org/en/humble/Contributing/Quality-Guide.html)

**Git:**
- [Conventional Commits](https://www.conventionalcommits.org/)
- [Git Branching Model](https://nvie.com/posts/a-successful-git-branching-model/)

---

## Change Log

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-10-12 | v1.0 | Initial coding standards document | Winston (Architect Agent) |

