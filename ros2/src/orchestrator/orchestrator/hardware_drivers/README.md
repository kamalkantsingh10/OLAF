# OLAF Hardware Driver Nodes

## Overview

Hardware driver nodes act as **ROS2 ↔ I2C bridges**, translating ROS2 topics into I2C register writes for ESP32 modules. Each driver node manages one hardware module.

**Architecture Pattern:**
```
ROS2 Topic → Hardware Driver Node → I2C Register Write → ESP32 Module → Hardware
```

**Benefits:**
- **Abstraction:** Application nodes don't need I2C knowledge
- **Decoupling:** Hardware changes don't affect application logic
- **Monitoring:** Drivers publish module status for health monitoring
- **Portability:** Drivers run only on Pi (with I2C hardware access)

## Available Driver Nodes

| Driver Node | Module | I2C Address | Hardware Controlled |
|-------------|--------|-------------|---------------------|
| `head_driver` | Head Module | 0x08 | 2× GC9A01 eye displays |
| `ears_neck_driver` | Ears+Neck Module | 0x09 | 4× ear servos, 3× neck servos |
| `body_driver` | Body Module | 0x0A | Heart LCD, projector, LEDs |
| `base_driver` | Base Module | 0x0B | Self-balancing, motors, kickstand |

**Epic 1 Status:**
- ✅ `head_driver` - Implemented (Story 1.5)
- ⏳ Others - Planned for future epics

## Running Driver Nodes

### Prerequisites

1. **I2C Enabled on Raspberry Pi:**
   ```bash
   sudo raspi-config
   # Navigate to: Interface Options → I2C → Enable
   ```

2. **User in i2c Group:**
   ```bash
   sudo usermod -a -G i2c $USER
   # Log out and log back in, OR:
   newgrp i2c
   ```

3. **Python Dependencies:**
   ```bash
   pip3 install smbus2
   ```

4. **ESP32 Modules Connected:**
   - Verify with: `sudo i2cdetect -y 1`
   - Should show devices at 0x08, 0x09, 0x0A, 0x0B

### Launch All Drivers

```bash
cd ~/olaf/ros2
source install/setup.bash
ros2 launch orchestrator drivers_only.launch.py
```

### Launch Single Driver (Development)

```bash
# Head driver only
ros2 run orchestrator head_driver
```

### Launch with Custom I2C Bus

```bash
ros2 launch orchestrator drivers_only.launch.py i2c_bus:=0
```

## Head Driver Node

**File:** `head_driver.py`

**Purpose:** Controls eye expressions on dual GC9A01 displays via I2C commands to Head module ESP32.

### Topics

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/olaf/head/expression` | `std_msgs/String` | Expression command (format: `"emotion,intensity"`) |

**Expression Format:**
```
"<emotion>,<intensity>"
```

**Emotion Types:**
- `neutral`, `happy`, `curious`, `thinking`, `confused`, `sad`, `excited`

**Intensity Levels:**
- `1` = Subtle
- `2` = Light
- `3` = Moderate (recommended default)
- `4` = Strong
- `5` = Extreme

**Examples:**
```bash
# Happy with moderate intensity
ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'happy,3'"

# Excited with extreme intensity
ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'excited,5'"

# Thinking with subtle intensity
ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'thinking,1'"
```

#### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/olaf/head/status` | `std_msgs/String` | 10 Hz | Module status (format: `"module_id,status_byte,error_code"`) |

**Status Format:**
```
"<module_id>,<status>,<error>"
```

**Status Byte Flags:**
- `0x01` (1) = READY - Module ready for commands
- `0x02` (2) = BUSY - Processing/animating
- `0x04` (4) = ERROR - Error occurred (check error_code)

**Examples:**
```bash
# Monitor status
ros2 topic echo /olaf/head/status

# Expected output:
data: '8,1,0'  # Module ID 8 (0x08), READY (0x01), No Error (0)
```

### I2C Communication Flow

1. **Startup Health Check:**
   ```
   Node starts → Read REG_MODULE_ID (0x00) → Verify returns 0x08 → Success
   ```

2. **Expression Command:**
   ```
   ROS2 message → Parse emotion & intensity →
   Write REG_EXPRESSION_TYPE (0x10) →
   Write REG_EXPRESSION_INTENSITY (0x11) →
   ESP32 animates eye expression
   ```

3. **Status Monitoring (10 Hz):**
   ```
   Read REG_MODULE_ID (0x00) →
   Read REG_STATUS (0x02) →
   Read REG_ERROR_CODE (0x03) →
   Publish to /olaf/head/status
   ```

### I2C Register Map (Head Module)

| Register | Address | Type | Description |
|----------|---------|------|-------------|
| Module ID | 0x00 | Read | Returns 0x08 (head module identifier) |
| Firmware Version | 0x01 | Read | Firmware version (0x01 = v0.1) |
| Status | 0x02 | Read | Status flags (READY=0x01, BUSY=0x02, ERROR=0x04) |
| Error Code | 0x03 | Read | Last error code (0x00 = no error) |
| Command | 0x04 | Write | Generic command register (reserved) |
| Expression Type | 0x10 | Write | Emotion type (0-6) |
| Expression Intensity | 0x11 | Write | Intensity level (1-5) |

**Expression Type Values:**
- 0 = NEUTRAL
- 1 = HAPPY
- 2 = CURIOUS
- 3 = THINKING
- 4 = CONFUSED
- 5 = SAD
- 6 = EXCITED

### Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | 1 | I2C bus number (/dev/i2c-N) |
| `i2c_address` | 0x08 | Head module I2C address |
| `i2c_timeout_ms` | 100 | I2C operation timeout |
| `status_update_hz` | 10.0 | Status publishing frequency |
| `retry_attempts` | 3 | I2C retry count before failure |

## Testing Driver Nodes

### Manual I2C Testing (Without ROS2)

```bash
# Verify head module detected
sudo i2cdetect -y 1
# Should show: 08 at row 0, column 8

# Read module ID
sudo i2cget -y 1 0x08 0x00
# Expected: 0x08

# Read status
sudo i2cget -y 1 0x08 0x02
# Expected: 0x01 (READY)

# Set happy expression, intensity 3
sudo i2cset -y 1 0x08 0x10 0x01  # HAPPY (1)
sudo i2cset -y 1 0x08 0x11 0x03  # Intensity 3
```

### ROS2 Node Testing

```bash
# Terminal 1: Launch driver
ros2 run orchestrator head_driver

# Terminal 2: Monitor status
ros2 topic echo /olaf/head/status

# Terminal 3: Send expression commands
ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'happy,3'"
ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'sad,2'"
ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'excited,5'"
```

### Verify End-to-End Pipeline

```bash
# 1. Check ESP32 running (serial monitor)
pio device monitor -d firmware/head
# Should show: "I2C Slave initialized at 0x08"

# 2. Check I2C connection
sudo i2cdetect -y 1
# Should show device at 0x08

# 3. Launch driver
ros2 run orchestrator head_driver
# Should show: "✓ Head module detected (ID: 0x08)"

# 4. Send test expression
ros2 topic pub --once /olaf/head/expression std_msgs/String "data: 'happy,3'"
# Eyes should display happy expression on both displays

# 5. Monitor logs
# Driver logs: Expression changes
# ESP32 serial: I2C register writes detected
```

## Troubleshooting

### "Permission denied: '/dev/i2c-1'"

**Cause:** User not in `i2c` group

**Solution:**
```bash
sudo usermod -a -G i2c $USER
newgrp i2c  # Or log out/in
groups  # Verify 'i2c' appears
```

### "No module detected at address 0x08"

**Cause:** ESP32 not responding on I2C bus

**Solutions:**
1. Check ESP32 firmware running: `pio device monitor`
2. Verify wiring: Pi GPIO2→ESP32 SDA, Pi GPIO3→ESP32 SCL, GND→GND
3. Test I2C: `sudo i2cdetect -y 1` (should show 08)
4. Check I2C enabled: `ls /dev/i2c-*` (should exist)

### "Module status shows ERROR (0x04)"

**Cause:** ESP32 firmware error

**Solutions:**
1. Read error code: `sudo i2cget -y 1 0x08 0x03`
2. Check ESP32 serial monitor for error messages
3. Common errors:
   - Display initialization failed
   - Invalid command received
   - I2C buffer overflow

### Driver Node Crashes on Invalid Expression

**Cause:** Malformed expression command

**Expected Format:** `"emotion,intensity"` (e.g., `"happy,3"`)

**Invalid Examples:**
- `"happy"` (missing intensity)
- `"5"` (missing emotion)
- `"happy,10"` (intensity out of range 1-5)

**Solution:** The driver validates input and logs warnings for invalid commands.

## Development Notes

### Adding New Driver Nodes

**Template:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from smbus2 import SMBus

class NewDriverNode(Node):
    def __init__(self):
        super().__init__('new_driver')
        self.i2c_bus = SMBus(1)
        self.i2c_address = 0x09  # Module address

        # Subscribe to commands
        self.sub = self.create_subscription(
            String,
            '/olaf/module/command',
            self.command_callback,
            10
        )

        # Publish status
        self.pub = self.create_publisher(String, '/olaf/module/status', 10)
        self.timer = self.create_timer(0.1, self.publish_status)  # 10 Hz

    def command_callback(self, msg):
        # Parse command and write I2C registers
        pass

    def publish_status(self):
        # Read I2C status registers and publish
        pass

def main():
    rclpy.init()
    node = NewDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Steps:**
1. Create `new_driver.py` in `orchestrator/hardware_drivers/`
2. Add entry point in `setup.py`
3. Update `drivers_only.launch.py` to include new node
4. Document I2C register map in this README

### I2C Best Practices

1. **Always check common ground** between Pi and ESP32
2. **Use pull-up resistors** (Pi has internal 1.8kΩ, usually sufficient)
3. **Keep I2C wires short** (<30cm) for 400kHz operation
4. **Validate I2C address** on module startup (read module ID register)
5. **Implement retries** for transient I2C errors (3 retries recommended)
6. **Log all I2C errors** with context (address, register, operation)

### Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| I2C Latency | <100ms | Single register write round-trip |
| Status Update Rate | 10 Hz | Module health monitoring frequency |
| Command Processing | <500ms | Expression change including animation |
| Error Recovery | <1s | Retry and fallback logic |

## References

- [Story 1.5: ROS2 Head Driver Node](../../../../docs/stories/1.5.ros2-head-driver-node.md)
- [Architecture: ROS2 Node Architecture](../../../../docs/architecture/ros2-node-architecture.md)
- [Architecture: Data Models (I2C Register Maps)](../../../../docs/architecture/data-models.md)
- [Firmware: Head Module](../../../../firmware/head/README.md)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [smbus2 Python Library](https://pypi.org/project/smbus2/)

## Change Log

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-11-02 | v1.0 | Initial hardware drivers documentation | Gilfoyle (Dev Agent) |
