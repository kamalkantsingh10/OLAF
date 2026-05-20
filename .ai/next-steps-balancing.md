# OLAF Balancing — Next Steps

## Current State (2026-04-11)

### What's Working
- ESP32 firmware: BNO085 IMU (200Hz), ODrive torque/current mode, OLED HUD, OTA flashing
- Web dashboard at http://olaf-base.local: live charts, PID sliders, BALANCE/STOP/TARE/SAVE buttons, REC LOG
- Balance PID: Kp=0.18, Ki=0, Kd=1.0, Target=4.2° — robot balances, resists pushes
- Ziegler-Nichols auto-tuner (relay method) — did NOT produce usable gains, needs replacement
- Velocity outer loop code exists (velocity_controller.cpp) but is DISABLED and untested successfully

### Known Issues
- **Robot drifts** — balances but slowly creeps across the floor
- Velocity loop was attempted but didn't work — likely due to unfiltered 90 CPR Hall encoder noise
- Ki=0.1 caused backward drift/acceleration (integral windup) — keep Ki=0
- Ziegler-Nichols auto-tune produced gains that didn't work on hardware
- ODrive `f <axis>` velocity readback is very noisy at low speeds (~1 count/sec at drift speed)
- After safety trip (45°), motor current display freezes (cosmetic)

### Hardware Facts
- 2x hoverboard motors, 90 CPR Hall encoders (6 states x 15 pole pairs)
- Robot weighs ~10kg, motors rated ~50-60kg each = 10x torque headroom
- BNO085 on-chip AHRS, game rotation vector at 200Hz
- ODrive v3.6, torque/current control mode, ASCII UART protocol
- Wheel diameter 0.17m, circumference ~0.534m → 5.9mm per encoder count

### Research Summary (2026-04-11)
Reviewed multiple self-balancing projects to decide architecture:

| Project | Architecture | Drift Solution | Encoder | Relevant? |
|---------|-------------|----------------|---------|-----------|
| **mjbots hoverbot** | Velocity PI → Pitch PD → current | Velocity integral | Hall sensors | **Best match** — same motors, same control mode |
| RolfK (Hackster) | Position PID → Balance PID → stepper speed | Position outer loop + hold migration | Stepper pulses (exact) | Twiddle auto-tuner is valuable |
| heethesh Balance-Bot | Position → Velocity → Pitch → motor (4 PIDs) | Full triple cascade | Quadrature encoders | Endgame for SLAM, overkill for now |
| LuSeKa HoverBot | Pitch PD only (no outer loop) | None — drifts, human compensates | None | Not useful |
| SolidGeek ESP32 | Cascade (no encoder feedback) | Velocity estimated from commands | None | Not useful |

**Decision: Follow mjbots hoverbot architecture** — velocity PI → pitch PD cascade. This provides a velocity interface needed for SLAM/Nav2 (`/cmd_vel`), and the velocity integral handles position hold when stationary. Add RolfK's Twiddle auto-tuner for gain optimization.

---

## Phase 1: Fix Velocity Loop + Add Yaw Control (Priority — do first)

The velocity controller code exists and is structurally correct. It failed because of unfiltered encoder noise. The motor controller already has the differential split for yaw (`left = balance + yaw`, `right = balance - yaw`) but only does open-loop proportional turn — needs heading-hold PID like mjbots.

### 1a. Add exponential smoothing to velocity readback

In `velocity_controller.cpp`, filter the raw ODrive velocity before using it:

```cpp
// Add to VelocityController private members:
float filteredVel0_ = 0.0f;
float filteredVel1_ = 0.0f;

// In update(), replace raw reads:
float rawVel0 = odriveUart.readVelocity(0) * kAxis0Direction;
filteredVel0_ = kVelFilterAlpha * filteredVel0_ + (1.0f - kVelFilterAlpha) * rawVel0;

float rawVel1 = odriveUart.readVelocity(1) * kAxis1Direction;
filteredVel1_ = kVelFilterAlpha * filteredVel1_ + (1.0f - kVelFilterAlpha) * rawVel1;

measuredVelocity_ = (filteredVel0_ + filteredVel1_) * 0.5f;
```

Add to `config.h`:
```cpp
constexpr float kVelFilterAlpha = 0.85f;  // Smoothing factor — higher = smoother, slower response
```

Add a slider for alpha (range 0.5-0.99) so it can be tuned on hardware.

### 1b. Add velocity loop enable/disable toggle to web dashboard

- Add a toggle button: "VEL LOOP ON/OFF"
- WebSocket command: `CMD:VELON` / `CMD:VELOFF`
- Calls `velController.setEnabled(true/false)`
- Show enabled state in UI (green/grey indicator)
- When disabled, `getPitchOffset()` returns 0 (existing behavior)

### 1c. Retune velocity gains with filter active

Starting gains (from mjbots scaled to our setup):
- VelKp = 5.0 (keep current)
- VelKi = 2.0 (lower than before — let the filter help)
- VelOutputMaxDeg = 5.0 (keep, increase to 10 if needed)
- VelFilterAlpha = 0.85 (adjust via slider)

Tuning procedure:
1. Get balance PID stable first (current gains should work)
2. Enable velocity loop with VelKi=0 first
3. Increase VelKp until robot resists being pushed to a new position
4. Add VelKi slowly until robot returns to original position after push
5. If oscillation: increase filter alpha (more smoothing) or decrease gains

### 1d. Also read ODrive position (for telemetry / future use)

The `f <axis>` response returns `<pos> <vel>`. Currently we parse only velocity and discard position. Parse both:

```cpp
// In odrive_uart.cpp readVelocity(), also return position:
struct OdriveMotionData {
    float position;   // cumulative turns
    float velocity;   // turns/s
};
OdriveMotionData readMotion(uint8_t axis);
```

Display position on web dashboard. This data feeds into `/odom` for SLAM later (Story 2.8).

### 1e. Add yaw heading-hold PID (mjbots pattern)

Currently `motor_controller.cpp` has a simple proportional turn (`yawCurrent_ = radps * 0.5f`).
Replace with a heading-hold PID that actively maintains heading when yaw command is zero.

**New file: `yaw_controller.h / yaw_controller.cpp`**

Architecture (from mjbots hoverbot):
```
yaw_rate_cmd ──→ integrate ──→ yaw_target
                                    │
                    imu_yaw ──→ heading_error = imu_yaw - yaw_target
                                    │
                              yaw_torque = Kp_yaw * heading_error + Kd_yaw * (gyro_z - cmd_yaw_rate)
```

Implementation:
```cpp
class YawController {
public:
    void begin();
    void reset();                          // Capture current yaw as target
    void update(float dt);                 // Run heading PID
    void setYawRate(float radps);          // From joystick / cmd_vel
    float getYawTorque() const;            // Feed into motor controller

private:
    float yawTarget_;                      // Integrated heading target (degrees)
    float yawRateCmd_;                     // Commanded yaw rate (rad/s)
    float yawTorque_;                      // Output torque (Amps differential)
};
```

Key behaviors:
- On BALANCE start: `yawTarget_ = imu.getYaw()` — capture current heading
- Each cycle: `yawTarget_ += yawRateCmd_ * RAD_TO_DEG * dt` — integrate command
- PID: `yawTorque = Kp_yaw * wrap180(imu_yaw - yawTarget) + Kd_yaw * (gyro_z - yawRateCmd)`
- When yaw_rate = 0: **actively holds heading** — robot won't rotate if bumped sideways
- This is critical for SLAM (consistent orientation during mapping)

Config defaults (from mjbots, scaled ~40% of pitch gains):
```cpp
constexpr float kYawKp = 0.01f;           // Nm per degree heading error — start conservative
constexpr float kYawKd = 0.001f;          // Nm per deg/s rate error
constexpr float kYawOutputMaxAmps = 2.0f; // Don't let yaw destabilize balance
```

Add Yaw Kp/Kd sliders to web dashboard for tuning.

Integration in `motor_controller.cpp` — replace `setTurnRate()`:
```cpp
// In applyCurrents(), yawCurrent_ now comes from yawController.getYawTorque()
// instead of the simple proportional
float left = balanceCurrent_ + yawController.getYawTorque();
float right = balanceCurrent_ - yawController.getYawTorque();
```

### 1f. BNO085 yaw data

Verify `imu.cpp` exposes yaw angle and gyro Z:
- `imu.getYaw()` — heading in degrees from game rotation vector
- `imu.getGyroZ()` — yaw rate in rad/s (already used in telemetry)
- BNO085 game rotation vector does NOT have magnetometer correction → yaw drifts over time
- For short-term heading hold this is fine. For SLAM, wheel odometry provides yaw correction.
- If `getYaw()` doesn't exist yet, add it alongside existing `getPitch()`/`getRoll()`

---

## Phase 2: Replace Auto-Tuner with Twiddle

The current Ziegler-Nichols relay method didn't produce usable gains. Replace with RolfK's Twiddle algorithm which optimizes gains online.

### 2a. Implement Twiddle optimizer

Algorithm:
```
params = [Kp, Kd, Target, VelKp, VelKi, FilterAlpha]
dp     = [0.02, 0.1, 0.5, 1.0, 1.0, 0.05]  // initial step sizes

while sum(dp) > 0.03:
    for i in range(len(params)):
        params[i] += dp[i]
        run for N seconds, measure avg |pitch_error|
        if better:
            dp[i] *= 1.1  // grow step
            best_error = error
        else:
            params[i] -= 2 * dp[i]  // try other direction
            run for N seconds
            if better:
                dp[i] *= 1.1
                best_error = error
            else:
                params[i] += dp[i]  // restore
                dp[i] *= 0.9  // shrink step
```

### 2b. Integration with web dashboard

- Replace AUTO-TUNE button behavior (or add second button: "TWIDDLE")
- Show live: which parameter is being tweaked, current best error, convergence %
- Robot must be balancing — Twiddle runs during live balancing
- Safety: if robot falls, restore last-known-good params, restart
- Save best params to Preferences when converged

### 2c. Twiddle settings

- Run duration per trial: 4-6 seconds (enough for velocity loop to respond)
- Error metric: average |pitch_error| + weight * |velocity| (penalize both tilt error and drift)
- Convergence threshold: sum(dp) < 0.03
- Safety: abort if pitch > 30° during any trial

---

## Phase 3: Web Joystick + AP Mode (drive it from your phone)

Once velocity loop and yaw heading-hold are working, add phone control.

### 3a. Virtual joystick on web dashboard

Add a touch joystick to the existing dashboard at `http://olaf-base.local`:

- Use **nipplejs** library (lightweight, touch-friendly, MIT license) — or a simple CSS touch-drag div
- Joystick area below the PID sliders section
- Forward/back axis → `setDesiredVelocity()` (range ±1 m/s via `kMaxLinearVelocityMps`)
- Left/right axis → `setYawRate()` (range ±2 rad/s via `kMaxTurnRateRadps`)
- Dead zone in center (~10% of travel)
- Send as WebSocket messages: `JOY:<vel>,<yaw>` at 20Hz while active
- On release: send `JOY:0,0` (stop)

ESP32 side:
```cpp
// In onWsEvent(), parse joystick:
if (strncmp(msg, "JOY:", 4) == 0) {
    float vel, yaw;
    sscanf(msg + 4, "%f,%f", &vel, &yaw);
    velController.setDesiredVelocity(vel);
    yawController.setYawRate(yaw);
}
```

### 3b. AP mode (WiFi hotspot for outdoor use)

For outdoor testing where there's no WiFi network:

- `WiFi.softAP("OLAF-Base", "olaf1234")` → IP: 192.168.4.1
- mDNS still works: `olaf-base.local`
- Auto-detect: try station mode for 10 seconds, fallback to AP if no connection
- OTA still works over AP
- Add a config flag or button to switch modes

### 3c. Dashboard layout for phone

Reorganize the web dashboard for portrait phone use:
- Top: pitch/PID readouts (compact)
- Middle: joystick (large touch target, ~40% of screen)
- Bottom: BALANCE/STOP buttons (large, always visible)
- PID sliders on a separate tab/collapsible section (not needed during driving)

---

## Phase 4: Position Hold Enhancement (if needed after Phase 1-3)

If the velocity integral alone isn't enough for station keeping, add a soft position P-loop (from RolfK):

```
position_error = hold_position - current_position
desired_velocity = Kp_pos * position_error  // very soft: Kp_pos ~ 0.1
→ feeds into velocity PI as setDesiredVelocity()
```

With hold-position migration after disturbances:
```
hold_position = 0.9 * hold_position + 0.1 * current_position
```

This becomes the triple cascade: position P → velocity PI → pitch PD → current.

Only implement if Phase 1-3 don't solve drift sufficiently.

---

## Phase 5: ROS2 Integration (Story 2.8)

Once balancing + velocity + yaw + joystick are solid:

1. `/cmd_vel` subscriber → `setDesiredVelocity(linear.x)`, `setYawRate(angular.z)`
2. ODrive position → `/odom` publisher (wheel odometry)
3. IMU pitch/roll/yaw/gyro → `/imu` publisher
4. Balance enable/disable via I2C command register

The velocity + yaw interface maps directly to `/cmd_vel`:
- Nav2 sends `linear.x` → velocity PI → pitch PD → current
- Nav2 sends `angular.z` → yaw PID → differential current
- Both combine at motor level: `left = pitch_torque + yaw_torque`, `right = pitch_torque - yaw_torque`

---

## Phase 6: Gazebo Simulation

URDF model, Gazebo world, ROS2 integration for testing navigation before hardware.

---

## Phase 7: Skate Park Test

Prerequisites: balance 60s+, velocity loop stable, yaw heading-hold, web joystick, AP mode WiFi.

---

## File Reference

| File | Purpose | Status |
|---|---|---|
| `modules/base/firmware/src/config.h` | All constants, PID defaults, velocity/yaw config | **Needs** vel filter alpha, yaw gains |
| `modules/base/firmware/src/balancing_controller.cpp` | Inner PID loop — uses `velController.getPitchOffset()` | Working |
| `modules/base/firmware/src/velocity_controller.cpp` | Outer velocity PI loop | **Needs** exponential smoothing |
| `modules/base/firmware/src/velocity_controller.h` | Velocity controller class definition | **Needs** filter state vars |
| `modules/base/firmware/src/yaw_controller.cpp` | Yaw heading-hold PID | **NEW — create** |
| `modules/base/firmware/src/yaw_controller.h` | Yaw controller class definition | **NEW — create** |
| `modules/base/firmware/src/odrive_uart.cpp` | ODrive UART — `readVelocity()` | **Needs** position parsing |
| `modules/base/firmware/src/motor_controller.cpp` | Torque mode, differential split (already has `left = bal + yaw`) | **Needs** yaw controller integration |
| `modules/base/firmware/src/web_tuner.cpp` | Web dashboard — sliders, charts, telemetry | **Needs** vel toggle, yaw sliders, joystick |
| `modules/base/firmware/src/imu.cpp` | BNO085 driver, gyro Y = pitch, gyro Z = yaw | **Check** if `getYaw()` exists |
| `modules/base/firmware/src/main.cpp` | Boot, state machine, loop | **Needs** yaw controller init/update |
| `modules/base/firmware/platformio.ini` | Build config, OTA settings | OK |

## Reference Projects

| Project | URL | What to take from it |
|---|---|---|
| mjbots hoverbot | https://github.com/mjbots/hoverbot | Architecture, gains, velocity filtering |
| RolfK redesign | https://www.hackster.io/RolfK/two-wheeled-self-balancing-robot-redesign-2077d9 | Twiddle auto-tuner, position hold migration |
| heethesh Balance-Bot | https://github.com/heethesh/Balance-Bot | Triple cascade reference for SLAM endgame |
