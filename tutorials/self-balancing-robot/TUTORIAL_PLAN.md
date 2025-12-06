# DIY Self-Balancing Robot Tutorial - The Complete Journey

So you've got a dead hoverboard lying around. Maybe the battery died, or the control board fried. But those motors? Those beautiful brushless motors with hall sensors? They're gold. This tutorial shows you how to turn that scrap into a proper self-balancing robot - the kind that actually stays upright without faceplanting.

We're building this the smart way: progressive steps, each one testing a specific piece. No "wire everything up and hope it works" nonsense. By the end, you'll have a robot that balances autonomously, and you'll actually understand how it works.

---

## What We're Building

A two-wheeled self-balancing platform that uses:
- **Salvaged hoverboard motors and battery** - free if you've got a dead hoverboard
- **ODrive motor controller** - the brain for the motors (~$120)
- **ESP32 microcontroller** - runs the PID control loop at 200Hz (~$10)
- **MPU6050 IMU** - tells us which way is up (~$3)
- **Kitchen bin chassis** - yes, really. Budget-friendly and works great (~$5)

Total cost if you have the hoverboard parts: **~$170**. Not bad for a self-balancing robot.

---

## The Build Journey: 6 Progressive Steps

Here's how we're doing this - each step builds on the last, and each step *proves something works* before moving on:

### **Step 1: The Physical Build**
*Time: 4-6 hours | Tools: Drill, screwdrivers, patience*

This is the fun part - we're building a Frankenstein robot from:
- Kitchen bin (main chassis - drill holes for motor mounts)
- Metal plates (structural reinforcement)
- Hoverboard motors (mounted to plates)
- Battery pack (secured inside bin)
- Buck converters (36V→12V and 36V→5V for safety)

**What you'll prove:** The mechanical assembly is solid. Wheels spin freely. Nothing falls apart.

**Files:** None yet - this is hands-on assembly with photos

---

### **Step 2: Motor Test via USB**
*Time: 30 minutes | Runs on: Your PC (Python)*

Before we touch the ESP32, let's prove the ODrive and motors work. We'll connect via USB and make those wheels spin.

**What you'll run:**
```bash
python tutorials/self-balancing-robot/step1_motors.py
```

This script:
- Connects to ODrive
- Calibrates the motors (one-time setup)
- Spins them forward, stops, spins backward
- Saves the calibration permanently

**What you'll prove:** Motors are wired correctly. ODrive can control them. Hall sensors work.

**What you'll see:** Your wheels spinning at 2 rev/s (about 120 RPM). Satisfying.

---

### **Step 3: ESP32 Takes Control**
*Time: 1 hour | Runs on: ESP32 via UART*

Now we cut the USB cord. The ESP32 talks to the ODrive via UART (3 wires), no PC needed.

**Prerequisites:**
1. Run `python tools/diagnostics/configure_odrive_uart.py` (enables UART on ODrive)
2. Wire GPIO16/17 from ESP32 to ODrive GPIO1/GPIO2

**What you'll upload:**
```bash
cd tutorials/self-balancing-robot/step2_esp32_uart
pio run -t upload
```

The ESP32 sends the same motor commands as Step 2, but via UART. Simple commands like:
```cpp
odriveSerial.print("v 0 2.0\n");  // Axis 0, 2 rev/s
```

**What you'll prove:** ESP32 can control ODrive without a PC. UART works. Motors respond.

**What you'll see:** Same wheel spinning as Step 2, but controlled by the ESP32. The robot is getting smarter.

---

### **Step 4: Adding the IMU**
*Time: 1-2 hours | New hardware: MPU6050*

Here's where it gets interesting. We add the MPU6050 IMU and start reading the pitch angle - how far forward or backward the robot is tilted.

**Wiring:**
- MPU6050 VCC → 5V (from buck converter)
- MPU6050 GND → GND
- MPU6050 SDA → ESP32 GPIO8
- MPU6050 SCL → ESP32 GPIO9

**What you'll upload:**
```bash
cd tutorials/self-balancing-robot/step3_imu_test
pio run -t upload
pio device monitor
```

Open the serial monitor and tilt the robot forward/backward. You'll see:
```
Pitch: +15.3° (tilted forward)
Pitch: -8.7° (tilted backward)
Pitch: +0.2° (almost vertical)
```

**What you'll prove:** IMU works. Tilt angle is accurate. Sensor fusion (gyro + accel) smooths the readings.

**What you'll learn:** The complementary filter. It's magic - combines fast gyro readings with stable accelerometer data.

---

### **Step 5: Elementary PID - First Balance!**
*Time: 2-3 hours | The moment of truth*

This is it. We combine the IMU readings with motor control. If the robot tilts forward, we drive the motors forward to "catch up". If it tilts backward, we reverse.

We start with a **P-only controller** (Proportional) because it's simple:
```cpp
float error = targetAngle - currentPitch;
float motorVelocity = Kp * error;  // Kp = 20.0 (initial guess)
```

If the robot tilts forward +5°, we drive motors at velocity = 20.0 * 5 = 100 rev/s (way too fast, but we'll tune it).

**What you'll upload:**
```bash
cd tutorials/self-balancing-robot/step4_simple_pid
pio run -t upload
```

**What you'll see:** The robot will probably oscillate wildly and fall over. That's normal! You've created your first (terrible) balancing controller.

**What you'll prove:** The control loop works. IMU → PID → Motors. The pieces are connected.

**Safety tip:** Have the kickstand deployed or hold the robot. It WILL try to run away.

---

### **Step 6: Tuned PID - Stable Balance**
*Time: 3-4 hours | Mostly tuning*

Now we add the **I** (Integral) and **D** (Derivative) terms and tune the gains. This is an art.

**The full PID equation:**
```cpp
error = targetAngle - currentPitch;
integral += error * dt;
derivative = (error - lastError) / dt;
output = Kp*error + Ki*integral + Kd*derivative;
```

**Tuning process:**
1. Start with Kp=20, Ki=0, Kd=0 (P-only)
2. Increase Kp until oscillations start, then back off 20%
3. Add Kd to dampen oscillations (try Kd = Kp/4)
4. Add tiny Ki to eliminate steady-state error (try Ki = 0.1)

**What you'll upload:**
```bash
cd tutorials/self-balancing-robot/step5_tuned_pid
pio run -t upload
```

**What you'll see:** The robot balances! It might wobble a bit, but it stays upright. You can give it a gentle push and it recovers.

**What you'll feel:** Pure joy. You built a self-balancing robot.

**Typical final gains:**
- Kp = 25-35 (depends on your robot's weight/wheelbase)
- Ki = 0.05-0.2 (very small)
- Kd = 5-8 (1/4 to 1/5 of Kp)

---

### **Step 7: Taking It Outside**
*Time: However long you want | The victory lap*

Your robot balances indoors. Now take it outside. Film it. Show it off.

**What you'll upload:** Production code with safety features
```bash
cd tutorials/self-balancing-robot/step6_full_balance
pio run -t upload
```

**New features in production code:**
- Watchdog timer (auto-stop if firmware hangs)
- Battery voltage monitoring (stops at 30V)
- Emergency stop if tilt > 45° (prevents faceplants)
- LED status indicators

**What you'll prove:** It works in the real world. Uneven ground, outdoor surfaces, sunlight.

**What you'll share:** Epic video of your robot balancing on the sidewalk. Blog post views incoming.

---

## The Hardware Shopping List

### Free (from dead hoverboard)
- ✅ 2× Brushless motors (~350W each, hall sensors)
- ✅ 2× Wheels (6.5" solid rubber)
- ✅ 36V battery pack (10S Li-ion, typically 4Ah)
- ✅ Battery charger (42V smart charger)

### Things to Buy
| Item | Price | Why You Need It |
|------|-------|-----------------|
| ODrive v3.6 | ~$120 | Controls both motors with closed-loop feedback |
| ESP32-S3-DevKitC-1 | ~$10 | Runs 200Hz PID loop (fast!) |
| MPU6050 IMU | ~$3 | Measures tilt (gyro + accelerometer) |
| Buck 36V→12V @ 10A | ~$8 | Powers future servos (optional now) |
| Buck 36V→5V @ 10A | ~$8 | Powers ESP32 and IMU safely |
| Metal plates | ~$15 | Motor mounts, structural support |
| Kitchen bin | ~$5 | Main chassis (seriously, it works) |
| M4 bolts/nuts | ~$5 | Motor mounting hardware |
| Jumper wires | ~$5 | All the connections |
| XT60 connectors | ~$5 | Power distribution |

**Total if you have hoverboard parts: $170-200**

---

## Power System: Why Two Buck Converters?

The hoverboard battery is **36V** (10S Li-ion). That's great for motors, terrible for electronics.

Here's how we distribute power safely:

```
┌─────────────────────┐
│  36V Hoverboard     │
│  Battery (10S)      │
└─────┬───────────────┘
      │
      ├──> ODrive (36V direct) ──> Motors (350W each)
      │
      ├──> Buck 36V→12V @ 10A ──> Future servos, peripherals
      │
      └──> Buck 36V→5V @ 10A  ──> ESP32, MPU6050, logic
```

**Why not connect ESP32 to 36V directly?** Because you'd have a very expensive sparkler. The ESP32 needs 3.3V (it has a regulator that accepts 5V max).

**Why not use the hoverboard's existing 5V regulator?** You could, but it's usually rated for low current. Buck converters give you 10A headroom.

---

## The Control Flow (200 Times Per Second)

Every 5 milliseconds (200Hz), this happens:

1. **Read IMU** → Current pitch angle (e.g., +3.5°)
2. **Calculate Error** → Target (0°) - Current (3.5°) = -3.5°
3. **Run PID** → Output velocity from Kp, Ki, Kd terms
4. **Send to Motors** → `"v 0 2.5\n"` via UART
5. **Wait 5ms** → Repeat

It's a feedback loop. The robot constantly adjusts to stay vertical. Like how you balance on a bike - constant tiny corrections.

---

## File Structure (for the Organized Among Us)

```
tutorials/self-balancing-robot/
├── TUTORIAL_PLAN.md              ← You are here
├── README.md                     ← Tutorial overview
├── step1_motors.py               ← Python USB test
├── step2_esp32_uart/             ← PlatformIO project
│   ├── platformio.ini
│   ├── src/main.cpp
│   └── README.md
├── step3_imu_test/
│   ├── platformio.ini
│   ├── src/main.cpp
│   └── README.md
├── step4_simple_pid/
│   ├── platformio.ini
│   ├── src/main.cpp
│   ├── src/config.h
│   └── README.md
├── step5_tuned_pid/
│   ├── platformio.ini
│   ├── src/main.cpp
│   ├── src/config.h
│   └── README.md
└── step6_full_balance/
    ├── platformio.ini
    ├── src/main.cpp              ← Production code
    ├── src/config.h              ← Pin definitions
    ├── src/odrive_uart.cpp/h     ← ODrive comms
    ├── src/imu_fusion.cpp/h      ← Complementary filter
    ├── src/pid_controller.cpp/h  ← PID logic
    └── README.md
```

Each step is a complete PlatformIO project. You can build and test independently.

---

## Alignment with OLAF Project

This tutorial is based on **OLAF Epic 6: Self-Balancing Base**. Same hardware, same architecture:

**From OLAF's architecture docs:**
- ✅ ESP32-S3 with 16MB flash, 8MB PSRAM
- ✅ MPU6050 (6-axis IMU) at 200Hz
- ✅ ODrive v3.6 motor controller
- ✅ 36V power system (10S Li-ion)
- ✅ Buck converters for voltage regulation
- ✅ Complementary filter: 98% gyro, 2% accel
- ✅ GPIO16/17 for UART, GPIO8/9 for I2C

**OLAF's Epic 6 Stories mapped to tutorial:**
- Story 6.3 (ODrive setup) → Tutorial Steps 2-3
- Story 6.4 (IMU integration) → Tutorial Step 4
- Story 6.5 (ESP32 firmware) → Tutorial Steps 5-6
- Story 6.7 (PID tuning) → Tutorial Step 6

You're essentially building OLAF's base module as a standalone robot.

---

## Development Tools You'll Need

**Software:**
- **Python 3.11+** with `odrive` library (`pip install odrive`)
- **PlatformIO** (VS Code extension or CLI: `pip install platformio`)
- **Serial monitor** (built into PlatformIO)

**PlatformIO Quick Reference:**
```bash
# Build
pio run

# Upload to ESP32
pio run -t upload

# Monitor serial output
pio device monitor

# All in one
pio run -t upload && pio device monitor
```

**Libraries we'll use:**
- `Wire.h` - I2C communication (Arduino built-in)
- `MPU6050` - Adafruit or Jeff Rowberg library
- `HardwareSerial` - UART (Arduino built-in)

---

## Safety First (No Joke)

36V can hurt you. Here's how to not get hurt:

**Electrical Safety:**
- [ ] Add 30A fuse on main battery line
- [ ] Use insulated tools when working with battery
- [ ] Never short battery terminals (boom)
- [ ] Buck converters prevent ESP32 overvoltage death

**Mechanical Safety:**
- [ ] Test with kickstand deployed initially
- [ ] Clear 2m radius when testing (it will move fast)
- [ ] Emergency stop: tilt > 45° = motors disabled
- [ ] Battery cutoff: < 30V = auto-shutdown

**Testing Safety:**
- [ ] Start on smooth, flat floor (not carpet, not gravel)
- [ ] Have someone nearby to catch it if needed
- [ ] Upload code incrementally - don't skip steps!

---

## Next Actions for Creating Tutorial

**Completed:**
- ✅ Created `config.h` with MPU6050, 36V power system
- ✅ Created `configure_odrive_uart.py` for UART setup
- ✅ Created `step1_motors.py` for USB motor test
- ✅ Created architecture-aligned tutorial plan

**Up Next:**
- ⏳ Create PlatformIO projects (steps 2-6)
- ⏳ Write code for each progressive step
- ⏳ Test on actual hardware
- ⏳ Film demonstration videos
- ⏳ Write blog post with narrative + code snippets

---

## Blog Post Outline (When We Get There)

**Title:** *"I Built a Self-Balancing Robot from a Dead Hoverboard (and You Can Too)"*

**Structure:**
1. **The Hook** - "That old hoverboard in your garage? It's worth more than you think."
2. **The Teardown** - Salvaging motors, battery, wheels (photos)
3. **The Build** - Kitchen bin chassis assembly (honest mistakes included)
4. **Step-by-Step** - Progressive tutorial (Motors → UART → IMU → PID)
5. **The First Balance** - Wobbly video, then stable video
6. **The Outdoor Test** - Epic finale video
7. **What I Learned** - PID tuning, sensor fusion, motor control
8. **Next Steps** - Ideas for upgrades (autonomy, sensors, SLAM)

**Target Platforms:**
- Personal blog (long-form)
- Hackster.io (project page)
- Reddit r/robotics (discussion)
- YouTube (demo videos)

---

**Author:** Gilfoyle Bertram (Dev Agent)
**Date:** 2025-12-02
**Status:** Planning complete, ready to build code
**Alignment:** OLAF Epic 6 Stories 6.3-6.7
