# Getting Your ODrive and Hoverboard Motors Running

**A Complete Guide: From First Power-Up to ESP32 Control**

Hey there! So you've got an ODrive motor controller and some hoverboard motors, and you want to get them spinning. You're in the right place. I'm going to walk you through the entire process, from initial setup to controlling your motors from an ESP32.

By the end of this guide, you'll have motors that respond to your commands and you'll understand exactly what's happening under the hood.

---

## What We're Building

We're going to:
1. **Power up and calibrate** your ODrive with hoverboard motors
2. **Test from your PC** using Python (prove everything works)
3. **Move to ESP32 control** via UART (untether from PC)
4. **Set up differential drive** (for turning and balancing)

**Total time:** About 30 minutes, including testing.

---

## Before We Start - What You Need

**Hardware checklist:**
- ✅ ODrive v3.6 motor controller
- ✅ 2× Hoverboard motors (those 350W BLDC motors with hall sensors)
- ✅ 2× Hoverboard wheels (typically 6.5")
- ✅ 12V power supply (I'm seeing you have 40V - even better!)
- ✅ USB cable (ODrive to computer)

**Wiring check** (you should have these already connected):
- Motor phase wires: 3 thick wires per motor → ODrive blue screw terminals (M0, M1)
- Hall sensors: 5 thin wires per motor → ODrive hall pins
- Power: 12V+ to ODrive XT60 connector

If you see a **green LED** on your ODrive, you're powered and ready to go!

---

## Step 1: The Magic Calibration Script

First things first - we need to tell the ODrive about your motors. Lucky for you, I've packaged everything into one simple script.

Open your terminal and run:

```bash
python tools/diagnostics/odrive_test_and_calibrate.py
```

**What's happening?** This script is:
1. Finding your ODrive on USB
2. Checking that hall sensors are connected
3. Measuring your motor's electrical properties (resistance, inductance)
4. Calibrating the hall sensor positions
5. Testing velocity control
6. Saving everything to the ODrive's memory

You'll see output like this:

```
============================================================
  ODrive Test & Calibration - Hoverboard Motors
============================================================

[1/6] Connecting to ODrive...
      ✓ Connected! Voltage: 40.3V
      Firmware: 0.5.1

[2/6] Checking hall sensors...
      Axis 0 hall state: 2
      Axis 1 hall state: 2
      ✓ Hall sensors working

[3/6] Configuring for hoverboard motors...
      ✓ Configuration applied

[4/6] Testing motor phase wire connections...
      ✓ Axis 0 phase wires connected (0.257Ω)
      ✓ Axis 1 phase wires connected (0.297Ω)
```

Your motors will **beep and spin** during calibration - that's normal! The ODrive is learning the hall sensor positions.

After about 60 seconds, you should see:

```
============================================================
  ✅ SUCCESS! ODrive Setup Complete
============================================================

  Summary:
    • Axis 0 (Left):  0.257Ω, Hall ready
    • Axis 1 (Right): 0.298Ω, Hall ready
    • Velocity control: Working
    • Configuration: Saved
```

**Congrats!** Your ODrive is now configured. This configuration is saved permanently, so you only need to run this once (unless you change your wiring).

---

## Step 2: Let's See What We've Got (Demo Time!)

Now for the fun part. Let's make those motors dance:

```bash
python tools/diagnostics/odrive_demo.py
```

This runs 5 automatic demos showing what your motors can do:

### Demo 1: Forward and Reverse
Both wheels spin forward, then reverse. Classic.

### Demo 2: Differential Turning
The left wheel spins faster than the right (turns right), then vice versa. This is how differential drive robots turn.

### Demo 3: Tank Turn (Spin in Place)
Left wheel forward, right wheel reverse - your robot would spin like a tank. Then it switches and spins the other way.

### Demo 4: Speed Ramping
Smoothly accelerates from 0 to 4 rev/s and back down. Great for testing acceleration curves.

### Demo 5: Individual Motor Test
Spins just the left motor, then just the right motor. Useful for troubleshooting.

**All of this runs automatically.** Just sit back and watch your motors prove they work!

---

## Step 3: Taking Control (Python from PC)

Okay, the scripts are cool, but what if you want to control the motors yourself? Let's write some Python.

Create a file called `my_odrive_test.py`:

```python
import odrive
from odrive.enums import *
import time

# Connect to ODrive
print("Finding ODrive...")
odrv = odrive.find_any()
print(f"Connected! Voltage: {odrv.vbus_voltage:.1f}V")

# Enable motors
print("Enabling motors...")
odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(0.5)

# Move forward
print("Moving forward at 2 rev/s...")
odrv.axis0.controller.input_vel = 2.0
odrv.axis1.controller.input_vel = 2.0
time.sleep(3)

# Stop
print("Stopping...")
odrv.axis0.controller.input_vel = 0
odrv.axis1.controller.input_vel = 0
time.sleep(1)

# Disable motors
print("Done!")
odrv.axis0.requested_state = AXIS_STATE_IDLE
odrv.axis1.requested_state = AXIS_STATE_IDLE
```

Run it:
```bash
python my_odrive_test.py
```

**Your motors spin!** Pretty simple, right?

### Making It Easier with Helper Functions

Let's make this cleaner:

```python
def set_velocity(odrv, axis, velocity):
    """Set velocity for one axis"""
    getattr(odrv, f'axis{axis}').controller.input_vel = velocity

def enable_motors(odrv):
    """Enable both motors for closed-loop control"""
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

def disable_motors(odrv):
    """Disable both motors"""
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    odrv.axis1.requested_state = AXIS_STATE_IDLE

# Now your code is much cleaner:
odrv = odrive.find_any()
enable_motors(odrv)

set_velocity(odrv, 0, 2.0)  # Left motor
set_velocity(odrv, 1, 2.0)  # Right motor
time.sleep(3)

set_velocity(odrv, 0, 0)
set_velocity(odrv, 1, 0)
```

**Pro tip:** Negative velocity = reverse direction. So `set_velocity(odrv, 0, -2.0)` spins the motor backwards.

---

## Step 4: Understanding What's Happening

You might be wondering: "What's this 2.0 velocity thing? Is that RPM? Speed?"

**Answer:** It's **revolutions per second (rev/s)**.

- `2.0` = 2 revolutions per second = 120 RPM
- `10.0` = 10 rev/s = 600 RPM (our configured max)

**To convert to linear speed:**

If your wheel diameter is 6.5 inches (0.165m), then:
- Circumference = π × 0.165m = 0.518m
- At 2 rev/s: Linear speed = 2 × 0.518m = **1.04 m/s** (about 2.3 mph)

So when you command `2.0 rev/s`, your robot moves forward at about walking speed.

---

## Step 5: Moving to ESP32 (Cutting the USB Cord)

Okay, Python from PC is great for testing, but your robot can't drag a laptop around. Time to move control to the ESP32.

### The UART Connection

The ODrive has two GPIO pins that speak UART (serial communication). We're going to wire those to your ESP32.

**Connections:**
```
ODrive          ESP32
──────          ─────
GPIO1 (TX)  →   GPIO16 (RX2)
GPIO2 (RX)  ←   GPIO17 (TX2)
GND         ─   GND
```

**Where are these pins?**
- On the ODrive: Look for small header pins labeled GPIO1, GPIO2 near the USB port
- On the ESP32: GPIO16 and GPIO17 are standard GPIO pins

**That's it.** Three wires and you're connected.

---

### Your First ESP32 Program

Open Arduino IDE and create a new sketch:

```cpp
#include <HardwareSerial.h>

// Create a serial connection on UART2
HardwareSerial odriveSerial(2);

void setup() {
  // Debug output
  Serial.begin(115200);

  // Connect to ODrive (RX=GPIO16, TX=GPIO17, 115200 baud)
  odriveSerial.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("ODrive UART connected!");
  delay(1000);

  // Enable motors
  Serial.println("Enabling motors...");
  odriveSerial.print("w axis0.requested_state 8\n");
  odriveSerial.print("w axis1.requested_state 8\n");
  delay(500);
}

void loop() {
  // Move forward
  Serial.println("Forward...");
  odriveSerial.print("v 0 2\n");   // Axis 0 at 2 rev/s
  odriveSerial.print("v 1 2\n");   // Axis 1 at 2 rev/s
  delay(3000);

  // Stop
  Serial.println("Stop...");
  odriveSerial.print("v 0 0\n");
  odriveSerial.print("v 1 0\n");
  delay(3000);
}
```

**Upload and watch!**

Your motors should spin forward for 3 seconds, stop for 3 seconds, and repeat.

### What Are These Commands?

The ODrive uses a simple **ASCII protocol**. Let me break it down:

**Enable motors:**
```cpp
"w axis0.requested_state 8\n"
```
- `w` = write command
- `axis0.requested_state` = the parameter we're setting
- `8` = CLOSED_LOOP_CONTROL state (motors enabled)
- `\n` = newline (tells ODrive the command is complete)

**Set velocity:**
```cpp
"v 0 2\n"
```
- `v` = velocity command (shortcut)
- `0` = axis number (0 or 1)
- `2` = velocity in rev/s
- `\n` = newline

**Super simple, right?**

---

## Step 6: Making It Cleaner (Helper Functions)

Let's organize that ESP32 code better:

```cpp
#include <HardwareSerial.h>

HardwareSerial odriveSerial(2);

void setup() {
  Serial.begin(115200);
  odriveSerial.begin(115200, SERIAL_8N1, 16, 17);
  enableMotors();
}

void enableMotors() {
  odriveSerial.print("w axis0.requested_state 8\n");
  odriveSerial.print("w axis1.requested_state 8\n");
  delay(500);
  Serial.println("Motors enabled");
}

void setVelocity(int axis, float velocity) {
  // Build command string: "v 0 2.5\n"
  String cmd = "v " + String(axis) + " " + String(velocity, 2) + "\n";
  odriveSerial.print(cmd);
}

void loop() {
  // Move forward
  setVelocity(0, 2.0);
  setVelocity(1, 2.0);
  delay(3000);

  // Stop
  setVelocity(0, 0);
  setVelocity(1, 0);
  delay(3000);
}
```

**Much cleaner!** Now you can just call `setVelocity(0, 2.0)` and forget about the protocol details.

---

## Step 7: Differential Drive (The Secret Sauce)

Here's where it gets interesting. For a balancing robot, you need **differential drive** - the ability to control each wheel independently for turning.

**The concept:**
- Both wheels same speed = go straight
- Left faster than right = turn right
- Right faster than left = turn left
- Opposite directions = spin in place

But we want to think in terms of **"move forward at X m/s while turning at Y rad/s"** rather than raw wheel speeds.

### The Math (Don't Worry, It's Pre-Coded)

```cpp
void setDifferentialVelocity(float linear_mps, float angular_radps) {
  // Your robot's measurements
  const float WHEEL_BASE = 0.30;      // Distance between wheels (30cm)
  const float WHEEL_RADIUS = 0.0825;  // 6.5" wheels = 8.25cm radius

  // Convert linear m/s to wheel rev/s
  float base_vel = linear_mps / (2 * PI * WHEEL_RADIUS);

  // Calculate differential from turning rate
  float diff = (angular_radps * WHEEL_BASE / 2.0) / (2 * PI * WHEEL_RADIUS);

  // Left wheel gets base + differential
  // Right wheel gets base - differential
  float left_vel = base_vel + diff;
  float right_vel = base_vel - diff;

  setVelocity(0, left_vel);
  setVelocity(1, right_vel);
}
```

### How to Use It

Now you can command motion naturally:

```cpp
void loop() {
  // Go straight forward at 0.5 m/s
  setDifferentialVelocity(0.5, 0);
  delay(3000);

  // Turn right while moving (0.3 m/s forward, 0.5 rad/s turn)
  setDifferentialVelocity(0.3, 0.5);
  delay(2000);

  // Spin in place (no forward motion, just turning)
  setDifferentialVelocity(0, 1.0);
  delay(2000);

  // Stop
  setDifferentialVelocity(0, 0);
  delay(3000);
}
```

**Real-world examples:**

| Code | What It Does |
|------|--------------|
| `setDifferentialVelocity(0.5, 0)` | Drive forward at half a meter per second |
| `setDifferentialVelocity(-0.3, 0)` | Reverse at 0.3 m/s |
| `setDifferentialVelocity(0, 0.5)` | Spin right in place |
| `setDifferentialVelocity(0.4, 0.2)` | Arc right while moving forward |

**This is exactly what you'll use for balancing!**

---

## Step 8: The Path to Balancing (Story 6.5 Preview)

You're probably thinking: "Okay, motors work. But how does this become a balancing robot?"

Great question! Here's the big picture for Story 6.5:

### You'll Add an IMU (MPU6050)

The IMU measures pitch angle (how far the robot is tilted).

```cpp
#include <MPU6050.h>

MPU6050 imu;
float pitch_angle;

void setup() {
  Wire.begin();
  imu.initialize();
  // ... ODrive setup
}

void loop() {
  // Read current pitch
  pitch_angle = readPitchAngle();  // How tilted are we?

  // If tilted forward, move motors forward to catch up
  // If tilted back, move motors backward to catch up
}
```

### You'll Implement PID Control

PID (Proportional-Integral-Derivative) is the algorithm that keeps your robot balanced.

```cpp
// PID gains (you'll tune these in Story 6.7)
float kp = 20.0;  // Proportional gain
float ki = 0.0;   // Integral gain
float kd = 5.0;   // Derivative gain

void loop() {
  static unsigned long last_time = 0;
  unsigned long now = millis();

  // Run at 200Hz (every 5ms)
  if (now - last_time >= 5) {
    last_time = now;

    // 1. Read IMU
    float pitch = readPitchAngle();

    // 2. Calculate error (we want 0° = vertical)
    float error = 0 - pitch;

    // 3. PID control (simplified P-only for now)
    float motor_velocity = kp * error;

    // 4. Send to motors
    setVelocity(0, motor_velocity);
    setVelocity(1, motor_velocity);
  }
}
```

**What happens:**
- Robot tilts forward (+5°) → Error = -5° → Motor velocity = -100 → Motors spin forward → Robot catches balance
- Robot tilts back (-3°) → Error = +3° → Motor velocity = +60 → Motors spin back → Robot catches balance

It's a continuous feedback loop running 200 times per second!

---

## Quick Command Reference

### ODrive ASCII Protocol

**Velocity control (what we use most):**
```
v 0 2.5     # Axis 0 to 2.5 rev/s
v 1 -1.0    # Axis 1 to -1.0 rev/s (reverse)
v 0 0       # Stop axis 0
```

**State control:**
```
w axis0.requested_state 8    # Enable motor (CLOSED_LOOP_CONTROL)
w axis0.requested_state 1    # Disable motor (IDLE)
```

**Read values (for debugging):**
```
r axis0.encoder.vel_estimate              # Current velocity
r axis0.motor.current_control.Iq_measured # Current draw
r vbus_voltage                            # Battery voltage
```

All commands **must end with `\n`** (newline character).

---

## Troubleshooting

### "ODrive not found" (PC)

**Check:**
1. USB cable plugged in?
2. Green LED on ODrive?
3. Run: `sudo usermod -a -G dialout $USER` then logout/login

### "Motors won't spin" (ESP32)

**Probably forgot to enable:**
```cpp
odriveSerial.print("w axis0.requested_state 8\n");
odriveSerial.print("w axis1.requested_state 8\n");
```

### "One motor spins, one doesn't"

**Check hall sensor wiring for the broken one:**
- 5 wires per motor
- VCC to 5V, GND to GND
- HA, HB, HC to correct pins

### "Motors beep but don't move"

**Phase wires loose:**
- Check blue screw terminals (M0, M1)
- Tighten screws
- Verify 3 thick wires per motor

---

## What's Next?

You've now got a fully working ODrive setup that you can control from PC (Python) or ESP32 (UART).

**Story 6.3:** ✅ Complete!

**Up next:**

**Story 6.4: Mount MPU6050 IMU**
- Wire IMU to ESP32 I2C bus
- Read pitch/roll angles
- Calibrate sensor

**Story 6.5: ESP32 Firmware - PID Balancing**
- Implement 200Hz control loop
- Integrate IMU readings with motor commands
- Get your first balance!

**Story 6.6: ROS2 Integration**
- Connect ESP32 to Raspberry Pi via I2C
- Subscribe to cmd_vel topics
- Publish odometry data

---

## Files in This Directory

- `odrive_test_and_calibrate.py` - One-shot calibration (run once)
- `odrive_demo.py` - 5 motor demonstrations
- `ESP32_CONTROL.md` - Quick ESP32 reference card
- `QUICK_START.md` - TL;DR version of this guide

---

**You made it!** 🎉

Your motors are calibrated, you understand the protocol, and you've got working code for both PC and ESP32 control. That's a huge milestone.

Time to add that IMU and start balancing! 🤖
