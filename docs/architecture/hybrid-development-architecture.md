# Hybrid PC+Pi Development Architecture

## Overview

OLAF supports a **hybrid development architecture** that allows developers to write and test code on their PC while the Raspberry Pi handles hardware interfacing. This architecture leverages ROS2's network transparency to enable seamless communication between nodes running on different machines over WiFi.

**Key Benefit:** Develop with your PC's power and familiar tools, deploy to Pi when ready—**zero code changes required**.

---

## Architecture Diagram

### Development Phase: PC + Pi Split

```
┌─────────────────────────────────────────────────────────┐
│  DEVELOPMENT PC (Application Logic)                     │
│  ┌────────────────────────────────────────────────────┐ │
│  │ ROS2 Humble Nodes                                  │ │
│  │                                                     │ │
│  │  ┌──────────────────────────────────────┐          │ │
│  │  │ personality_coordinator_node         │          │ │
│  │  │ - Publishes: /olaf/head/expression   │          │ │
│  │  │ - Publishes: /olaf/ears_neck/gesture │          │ │
│  │  └──────────────────────────────────────┘          │ │
│  │                                                     │ │
│  │  ┌──────────────────────────────────────┐          │ │
│  │  │ ai_agent_node                        │          │ │
│  │  │ - Claude API client                  │          │ │
│  │  │ - Personality generation             │          │ │
│  │  └──────────────────────────────────────┘          │ │
│  │                                                     │ │
│  │  ┌──────────────────────────────────────┐          │ │
│  │  │ Other application nodes              │          │ │
│  │  │ - Navigation coordinator             │          │ │
│  │  │ - State management                   │          │ │
│  │  └──────────────────────────────────────┘          │ │
│  │                                                     │ │
│  └────────────────────────────────────────────────────┘ │
│                                                          │
│  Launch: ros2 launch olaf_orchestrator app_nodes.launch.py
└──────────────────────────────────────────────────────────┘
                            │
                            │ WiFi Network
                            │ ROS2 DDS Protocol
                            │ (Topic Pub/Sub)
                            ↓
┌─────────────────────────────────────────────────────────┐
│  RASPBERRY PI 5 (Hardware Bridge)                       │
│  ┌────────────────────────────────────────────────────┐ │
│  │ ROS2 Humble Driver Nodes                           │ │
│  │                                                     │ │
│  │  ┌──────────────────────────────────────┐          │ │
│  │  │ head_driver_node                     │          │ │
│  │  │ - Subscribes: /olaf/head/expression  │          │ │
│  │  │ - I2C Master → 0x08                  │──────┐   │ │
│  │  └──────────────────────────────────────┘      │   │ │
│  │                                                 │   │ │
│  │  ┌──────────────────────────────────────┐      │   │ │
│  │  │ ears_neck_driver_node                │      │   │ │
│  │  │ - Subscribes: /olaf/ears_neck/gesture│      │   │ │
│  │  │ - I2C Master → 0x09                  │──────┤   │ │
│  │  └──────────────────────────────────────┘      │   │ │
│  │                                                 │   │ │
│  │  ┌──────────────────────────────────────┐      │   │ │
│  │  │ body_driver_node                     │      │   │ │
│  │  │ - I2C Master → 0x0A                  │──────┤   │ │
│  │  └──────────────────────────────────────┘      │   │ │
│  │                                                 │   │ │
│  │  ┌──────────────────────────────────────┐      │   │ │
│  │  │ base_driver_node                     │      │   │ │
│  │  │ - I2C Master → 0x0B                  │──────┘   │ │
│  │  └──────────────────────────────────────┘          │ │
│  │                                                     │ │
│  └────────────────────────────────────────────────────┘ │
│                                                          │
│  Launch: ros2 launch olaf_orchestrator drivers_only.launch.py
└──────────────────────────────────────────────────────────┘
                            │
                            │ I2C Bus @ 400kHz-1MHz
                            │ /dev/i2c-1 (GPIO 2/3)
                            ↓
┌─────────────────────────────────────────────────────────┐
│  ESP32-S3 MODULES (Smart I2C Slaves)                    │
│                                                          │
│  ┌─────────┐  ┌────────────┐  ┌─────────┐  ┌─────────┐ │
│  │  Head   │  │ Ears+Neck  │  │  Body   │  │  Base   │ │
│  │  0x08   │  │   0x09     │  │  0x0A   │  │  0x0B   │ │
│  │         │  │            │  │         │  │         │ │
│  │ Eyes    │  │ 4× Servos  │  │ Heart   │  │ Motors  │ │
│  │ mmWave  │  │ 3× Servos  │  │ LEDs    │  │ IMU     │ │
│  └─────────┘  └────────────┘  └─────────┘  └─────────┘ │
└─────────────────────────────────────────────────────────┘
```

---

### Production Phase: All Nodes on Pi

```
┌─────────────────────────────────────────────────────────┐
│  RASPBERRY PI 5 (Complete System)                       │
│  ┌────────────────────────────────────────────────────┐ │
│  │ ROS2 Humble Nodes (All Local)                      │ │
│  │                                                     │ │
│  │  Application Nodes:                                │ │
│  │  ┌──────────────────────────────────────┐          │ │
│  │  │ personality_coordinator_node         │          │ │
│  │  │ ai_agent_node                        │          │ │
│  │  │ navigation_node                      │          │ │
│  │  └──────────────────────────────────────┘          │ │
│  │            ↓ Local ROS2 Topics (fast)              │ │
│  │  Driver Nodes:                                     │ │
│  │  ┌──────────────────────────────────────┐          │ │
│  │  │ head_driver_node     → I2C 0x08      │          │ │
│  │  │ ears_neck_driver_node → I2C 0x09     │          │ │
│  │  │ body_driver_node     → I2C 0x0A      │          │ │
│  │  │ base_driver_node     → I2C 0x0B      │          │ │
│  │  └──────────────────────────────────────┘          │ │
│  │                                                     │ │
│  └────────────────────────────────────────────────────┘ │
│                                                          │
│  Launch: ros2 launch olaf_orchestrator olaf_full.launch.py
└──────────────────────────────────────────────────────────┘
                            │
                            │ I2C Bus (same as before)
                            ↓
                      ESP32 Modules
```

---

## Communication Flow Example

### Scenario: PC Triggers Happy Expression

**Step-by-step flow:**

1. **PC: personality_coordinator_node**
   ```python
   # Running on PC
   expression_msg = Expression()
   expression_msg.emotion_type = 1  # HAPPY
   expression_msg.intensity = 3
   expression_msg.duration_ms = 2000

   self.expression_pub.publish(expression_msg)
   # Topic: /olaf/head/expression
   ```

2. **ROS2 DDS (WiFi)**
   - Message serialized and sent via DDS multicast
   - Travels over WiFi from PC to Pi
   - Latency: ~10-50ms (network dependent)

3. **Pi: head_driver_node**
   ```python
   # Running on Pi
   def expression_callback(self, msg):
       # Received from PC over WiFi
       emotion = msg.emotion_type
       intensity = msg.intensity

       # Translate to I2C register writes
       self.i2c_bus.write_byte_data(0x08, 0x10, emotion)
       self.i2c_bus.write_byte_data(0x08, 0x11, intensity)
       # Latency: 5-20ms (I2C)
   ```

4. **ESP32 Head Module**
   ```cpp
   // I2C interrupt handler
   void onI2CReceive(int numBytes) {
       uint8_t reg = Wire.read();
       uint8_t value = Wire.read();

       if (reg == 0x10) {  // Emotion type
           current_emotion = value;
       }
       if (reg == 0x11) {  // Intensity
           current_intensity = value;
       }
   }

   // Main loop - render animation
   void loop() {
       renderEmotion(current_emotion, current_intensity);
       // Update GC9A01 displays at 60 FPS
   }
   ```

**Total latency: 15-70ms (PC pub → ESP32 render start)**

---

## Network Configuration

### ROS2 Domain ID

Both PC and Pi must use the **same ROS2 Domain ID** to see each other's topics.

**Setup:**

```bash
# On PC (~/.bashrc)
export ROS_DOMAIN_ID=42

# On Pi (~/.bashrc) - set by install script
export ROS_DOMAIN_ID=42
```

**Verify:**
```bash
# On PC
echo $ROS_DOMAIN_ID  # Should show: 42

# On Pi
echo $ROS_DOMAIN_ID  # Should show: 42
```

**Why Domain ID 42?**
- Isolates OLAF from other ROS2 systems on the network
- Valid range: 0-101
- Can be changed, but must match on all machines

---

### DDS Discovery

ROS2 uses **DDS (Data Distribution Service)** for node discovery and communication.

**Default behavior:**
- Nodes advertise themselves via UDP multicast
- Other nodes on same domain discover automatically
- No manual IP configuration needed

**Verification:**

```bash
# On Pi: List ROS2 nodes
ros2 node list

# On PC: Should see Pi's nodes
ros2 node list
# Expected output:
# /olaf/head_driver
# /olaf/ears_neck_driver
# /olaf/body_driver
# /olaf/base_driver

# On PC: Start app nodes
ros2 launch olaf_orchestrator app_nodes.launch.py

# On Pi: Should now see PC's nodes too
ros2 node list
# Additional output:
# /olaf/personality_coordinator
# /olaf/ai_agent
```

---

### Firewall Configuration

**If nodes don't see each other, check firewall:**

**On Ubuntu PC:**
```bash
# Allow ROS2 DDS ports
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

**On Raspberry Pi OS:**
```bash
# Pi OS typically has no firewall by default
# If using ufw:
sudo ufw allow 7400:7500/udp
sudo ufw allow 7400:7500/tcp
```

---

## Development Workflow

### 1. Initial Setup (One-Time)

**Raspberry Pi:**
```bash
# Flash Pi OS, SSH in
ssh pi@raspberrypi.local

# Clone repo
git clone <repo-url> ~/olaf
cd ~/olaf

# Run setup script
./tools/setup/install_ros2_humble_pi.sh
# (Installs ROS2, I2C, sets ROS_DOMAIN_ID=42)

sudo reboot
```

**PC:**
```bash
# Install ROS2 Humble (see docs/PC_DEVELOPMENT_SETUP.md)
# Clone repo
git clone <repo-url> ~/olaf
cd ~/olaf

# Set domain ID
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Install dependencies
pip3 install -r ros2/src/orchestrator/requirements.txt
```

---

### 2. Daily Development Cycle

**Morning: Start Pi drivers**
```bash
# SSH to Pi (leave this running all day)
ssh pi@raspberrypi.local
cd ~/olaf
ros2 launch olaf_orchestrator drivers_only.launch.py
```

**On PC: Develop and test**
```bash
# Terminal 1: Edit code
cd ~/olaf
vim orchestrator/ros2_nodes/personality/personality_coordinator_node.py

# Terminal 2: Run app nodes
ros2 launch olaf_orchestrator app_nodes.launch.py

# Terminal 3: Monitor topics
ros2 topic echo /olaf/head/expression

# Terminal 4: Test commands
ros2 topic pub /olaf/head/expression olaf_interfaces/Expression \
  "{emotion_type: 1, intensity: 3, duration_ms: 2000}"
```

**Test iteration:**
```bash
# Edit code on PC
vim personality_coordinator_node.py

# Restart node (Ctrl+C, then relaunch)
ros2 launch olaf_orchestrator app_nodes.launch.py

# Test immediately - Pi drivers still running
```

---

### 3. Deploy to Pi

**When feature is ready:**
```bash
# On PC: Commit changes
git add .
git commit -m "feat: add happy emotion intensity scaling"
git push origin main

# On Pi: Pull changes
ssh pi@raspberrypi.local
cd ~/olaf
git pull origin main

# Test full system locally
ros2 launch olaf_orchestrator olaf_full.launch.py
```

---

## Launch File Strategy

### `drivers_only.launch.py` (Pi Only)

**Purpose:** Start hardware driver nodes that require I2C access.

**Runs on:** Raspberry Pi

**Nodes:**
- `head_driver_node` (I2C 0x08)
- `ears_neck_driver_node` (I2C 0x09)
- `body_driver_node` (I2C 0x0A)
- `base_driver_node` (I2C 0x0B)

**Usage:**
```bash
# On Pi
ros2 launch olaf_orchestrator drivers_only.launch.py
```

**When to use:**
- PC+Pi hybrid development
- Pi acts as hardware bridge
- PC runs application logic

---

### `app_nodes.launch.py` (PC or Pi)

**Purpose:** Start application-level nodes with no hardware dependencies.

**Runs on:** PC (development) or Pi (production)

**Nodes:**
- `personality_coordinator_node`
- `ai_agent_node` (optional, requires API keys)
- Future: navigation nodes, state manager, etc.

**Usage:**
```bash
# On PC (during development)
ros2 launch olaf_orchestrator app_nodes.launch.py

# OR on Pi (if testing full system)
ros2 launch olaf_orchestrator app_nodes.launch.py
```

**When to use:**
- PC development (with Pi drivers running)
- Testing application logic without deploying to Pi

---

### `olaf_full.launch.py` (Pi Production)

**Purpose:** Start complete OLAF system on Raspberry Pi.

**Runs on:** Raspberry Pi only

**Nodes:** All nodes (drivers + application)

**Usage:**
```bash
# On Pi
ros2 launch olaf_orchestrator olaf_full.launch.py
```

**When to use:**
- Production deployment
- Full integration testing on Pi
- Autonomous robot operation

**Benefits:**
- All communication local (faster than WiFi)
- No network dependency
- Lower latency

---

## Migration Path: PC → Pi

**The beauty of this architecture:** Same code runs on both!

### Example: personality_coordinator_node

**On PC (development):**
```bash
ros2 run olaf_orchestrator personality_coordinator_node
# Publishes to /olaf/head/expression
# Pi driver node (over WiFi) receives and drives I2C
```

**On Pi (production):**
```bash
ros2 run olaf_orchestrator personality_coordinator_node
# Same code!
# Publishes to /olaf/head/expression
# Pi driver node (same machine) receives and drives I2C
# Faster - local communication instead of WiFi
```

**No code changes. Just different launch location.**

---

## Performance Characteristics

### Development (PC ↔ Pi over WiFi)

| Metric | Value | Notes |
|--------|-------|-------|
| **Network latency** | 10-50ms | WiFi dependent |
| **Topic pub/sub** | 20-100ms | PC → Pi |
| **I2C latency** | 5-20ms | Pi → ESP32 |
| **End-to-end** | 35-170ms | PC pub → ESP32 action |
| **Acceptable for** | Development, testing, non-critical paths | |
| **NOT acceptable for** | Real-time control (200Hz PID) | Driver nodes must be on Pi |

### Production (All on Pi)

| Metric | Value | Notes |
|--------|-------|-------|
| **Local pub/sub** | <1ms | Same machine |
| **I2C latency** | 5-20ms | Pi → ESP32 |
| **End-to-end** | 6-21ms | Pi pub → ESP32 action |
| **Acceptable for** | All operations including real-time | |

**Key insight:** Critical real-time nodes (driver nodes) always run on Pi, so 200Hz PID loops are unaffected.

---

## Advantages of Hybrid Architecture

### Development Experience

✅ **Use PC tools:** Your IDE, debuggers, linters, automation
✅ **Faster iteration:** Edit → test without deploying
✅ **Better performance:** PC likely faster than Pi for development
✅ **Familiar environment:** Your existing workflows

### Testing

✅ **Incremental testing:** Test nodes individually before full integration
✅ **Safe failure:** PC node crash doesn't affect hardware
✅ **Easy debugging:** Full ROS2 tooling (rqt, rviz) on PC

### Deployment

✅ **Zero code changes:** Same nodes, different machine
✅ **Gradual migration:** Move nodes to Pi one at a time
✅ **Production ready:** Final deployment is just a launch file change

---

## Limitations and Considerations

### Network Dependency

⚠️ **WiFi required during development**
- PC and Pi must be on same network
- Network latency affects responsiveness
- Corporate WiFi may block DDS multicast

**Solution:** Configure static DDS peers if multicast blocked.

### Latency Sensitivity

⚠️ **Some operations sensitive to WiFi latency**
- Expression synchronization target: <500ms
- WiFi adds 10-50ms, still acceptable
- Real-time PID (200Hz) unaffected - always on Pi

### Resource Constraints

⚠️ **Pi runs driver nodes even during development**
- Pi CPU/memory usage for 4 driver nodes: ~10-15%
- Leaves plenty of headroom for application nodes in production

---

## Troubleshooting

### PC Cannot See Pi Nodes

**Symptoms:**
```bash
# On PC
ros2 node list
# Only shows: /parameter_events, /rosout
# Missing: /olaf/head_driver, etc.
```

**Checklist:**
1. ✅ ROS_DOMAIN_ID matches on both machines
   ```bash
   echo $ROS_DOMAIN_ID  # Should be same (e.g., 42)
   ```

2. ✅ Both machines on same network
   ```bash
   # Can PC ping Pi?
   ping raspberrypi.local
   ```

3. ✅ Firewall allows DDS ports (7400-7500)
   ```bash
   sudo ufw allow 7400:7500/udp
   ```

4. ✅ Pi driver nodes are running
   ```bash
   # On Pi
   ros2 node list  # Should show driver nodes
   ```

---

### Topics Not Received on Pi

**Symptoms:**
- PC publishes to topic
- Pi driver node doesn't receive messages

**Debug:**
```bash
# On Pi: Echo topic
ros2 topic echo /olaf/head/expression

# On PC: Publish
ros2 topic pub /olaf/head/expression olaf_interfaces/Expression \
  "{emotion_type: 1, intensity: 3, duration_ms: 2000}"

# Should see message on Pi terminal
```

**If not working:**
- Check topic name exact match
- Verify message type matches
- Check QoS settings (default should work)

---

## Production Deployment Checklist

When moving from hybrid to production (all on Pi):

- [ ] Test full system on Pi: `ros2 launch olaf_orchestrator olaf_full.launch.py`
- [ ] Verify all nodes start: `ros2 node list`
- [ ] Test all ROS2 topics: `ros2 topic list`
- [ ] Measure latency: `ros2 topic echo --no-arr /olaf/head/expression`
- [ ] Create systemd service for auto-start on boot
- [ ] Test cold boot (power cycle Pi)
- [ ] Verify I2C communication: `sudo i2cdetect -y 1`
- [ ] Test API keys loaded from .env
- [ ] Verify log file rotation configured
- [ ] Document any Pi-specific configuration

---

## Summary

**Hybrid PC+Pi architecture enables:**

1. **Develop on PC** - Use your powerful development machine
2. **Test with hardware** - Pi provides I2C bridge to ESP32 modules
3. **Deploy seamlessly** - Same code, just move launch location
4. **Zero code changes** - ROS2 handles network transparency

**Key files:**
- `ros2/src/orchestrator/launch/drivers_only.launch.py` - Pi hardware drivers
- `ros2/src/orchestrator/launch/app_nodes.launch.py` - PC application nodes
- `ros2/src/orchestrator/launch/olaf_full.launch.py` - Production (all on Pi)

**Network requirement:** ROS_DOMAIN_ID must match (default: 42)

**Migration path:** PC development → Pi production (no code changes)

---

## Related Documentation

- [PC Development Setup Guide](../PC_DEVELOPMENT_SETUP.md) - Complete PC setup
- [Raspberry Pi Setup](../../tools/setup/README.md) - Pi installation
- [ROS2 Node Architecture](ros2-node-architecture.md) - Node design patterns
- [High-Level Architecture](high-level-architecture.md) - Overall system design
- [Tech Stack](tech-stack.md) - Technology choices

---

## Change Log

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-10-15 | v1.0 | Initial hybrid architecture documentation | James (Dev Agent) |
