# Logger - UDP Syslog WiFi Debug Logger

Multi-module UDP logging library for OLAF robotics platform. Broadcasts debug messages from all modules (Head, Neck, Torso, Base) to a single PC listener over WiFi.

## Features

- **Drop-in Serial replacement** - Familiar API: `logger.println()`, `logger.printf()`
- **Multi-level logging** - DEBUG, INFO, WARN, ERROR with filtering
- **Module identification** - Each message tagged with module name
- **Fire-and-forget UDP** - Minimal performance impact, no blocking
- **Graceful degradation** - Fails silently when WiFi unavailable

## Quick Start

### 1. In Your Module Firmware

```cpp
#include <Logger.h>

Logger logger("NECK");  // Module name

void setup() {
  // After WiFi connects
  logger.begin("192.168.1.100");  // Your PC's IP
  logger.info("Module started");
}

void loop() {
  logger.infof("Servo position: %d", servoPos);
  logger.error("I2C timeout!");
}
```

### 2. On Your PC

```bash
# Listen for UDP syslog on port 514
nc -lu 514

# Or with timestamps
nc -lu 514 | while read line; do echo "$(date '+%H:%M:%S') $line"; done
```

## API Reference

### Initialization

```cpp
Logger logger("MODULE_NAME");
bool begin(IPAddress serverIP, uint16_t port = 514);
bool begin(const char* serverIP, uint16_t port = 514);
```

### Log Levels

```cpp
logger.debug("Debug message");     // Level 0
logger.info("Info message");       // Level 1 (default)
logger.warn("Warning message");    // Level 2
logger.error("Error message");     // Level 3

logger.setLevel(Logger::DEBUG);    // Show all messages
logger.setLevel(Logger::ERROR);    // Only errors
```

### Formatted Output

```cpp
logger.infof("Servo %d at %d degrees", servoId, angle);
logger.errorf("Timeout after %d attempts", retries);
```

### Serial-Compatible API

```cpp
logger.print("Hello");
logger.println("World");
logger.printf("Value: %d\n", value);
```

## Message Format

```
[MODULE][LEVEL] message
```

**Example:**
```
[NECK][INFO] Servo 1 detected
[HEAD][WARN] OLED init slow
[BASE][ERROR] IMU read failed
```

## Advanced Usage

### Check if Active

```cpp
if (logger.isActive()) {
  // WiFi connected and logger ready
}
```

### Filter by Level

```cpp
logger.setLevel(Logger::WARN);  // Only show WARN and ERROR
```

## PC Listener Options

**Simple:**
```bash
nc -lu 514
```

**With timestamps:**
```bash
nc -lu 514 | ts '[%Y-%m-%d %H:%M:%S]'
```

**Filter by module:**
```bash
nc -lu 514 | grep "\[NECK\]"
```

**Color-coded (Linux):**
```bash
nc -lu 514 | while read line; do
  case "$line" in
    *ERROR*) echo -e "\e[31m$line\e[0m" ;;  # Red
    *WARN*)  echo -e "\e[33m$line\e[0m" ;;  # Yellow
    *INFO*)  echo -e "\e[32m$line\e[0m" ;;  # Green
    *)       echo "$line" ;;
  esac
done
```

## Performance

- **UDP overhead:** ~40 bytes per message
- **Non-blocking:** <1ms per log call
- **No memory leaks:** Fixed buffer allocation
- **WiFi failure:** Silently skips, no crashes

## Troubleshooting

**No messages appearing:**
1. Check PC IP address is correct
2. Verify WiFi connected: `logger.isActive()`
3. Check firewall allows UDP 514
4. Try different port: `logger.begin(serverIP, 6666)`

**Messages truncated:**
- Max message size: 512 bytes
- Longer messages automatically truncated

**High CPU on PC:**
- Messages coming too fast
- Use log level filtering: `logger.setLevel(Logger::WARN)`
