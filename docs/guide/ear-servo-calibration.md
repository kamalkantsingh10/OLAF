# Ear Servo Calibration Guide

Calibration tool for the 4× Feetech SCS0009 ear servos via Waveshare Bus Servo Adapter.

**Script:** `ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/ears_calibration.py`
**Config:** `config/servo-ids.yaml`

## Prerequisites

- Waveshare Bus Servo Adapter (A) connected via USB to Pi
- Symlink `/dev/waveshare_ears` exists (configured in Story 0.6)
- 5V power from 36V→5V buck converter connected to adapter
- `scservo_sdk` available (vendored at `libs/scservo_sdk/`)

## Servo ID Assignments

| ID | Location | Function |
|----|----------|----------|
| 4 | Left ear | Base rotation (pan) |
| 5 | Left ear | Ear angle (tilt) |
| 6 | Right ear | Base rotation (pan) |
| 7 | Right ear | Ear angle (tilt) |

## Commands

All commands are run from the project root on the Pi:

```bash
cd ~/olaf
poetry run python ros2/src/olaf_drivers/head_ears_driver/head_ears_driver/ears_calibration.py <command>
```

### ping — Check servo connection

Verify a single servo responds on the bus.

```bash
poetry run python ears_calibration.py ping <servo_id>
```

Example:
```
$ poetry run python ears_calibration.py ping 4
[INFO] Port opened: /dev/waveshare_ears @ 1000000 bps
[OK] Servo ID 4 responded
```

### assign-all — Assign IDs to new servos

Interactive walkthrough to assign IDs 4-7 to fresh servos (factory default ID 1). Add one servo at a time to the daisy chain — no need to disconnect after each assignment.

```bash
poetry run python ears_calibration.py assign-all
```

**Flow:**
1. Connect first servo (factory ID 1) → script assigns it to ID 4 → leave on chain
2. Connect next servo (factory ID 1) → assigns to ID 5 → leave on chain
3. Repeat for IDs 6 and 7

### set-id — Change a single servo's ID

Manually change one servo's ID. Useful for replacements or reassignment.

```bash
poetry run python ears_calibration.py set-id <current_id> <new_id>
```

Example:
```
$ poetry run python ears_calibration.py set-id 1 4
[INFO] Servo found at ID 1
[OK] Servo ID changed: 1 -> 4
```

### set-center — Save current position as center

Physically position all ears at the desired center (straight up, fins parallel to eyeline), then run this command. It reads the raw servo positions and saves them as `center_position` in `config/servo-ids.yaml`.

```bash
poetry run python ears_calibration.py set-center
```

**Important:** Position ears manually before running. These saved values become the zero reference for all degree-based movement in the driver.

### go-center — Move ears to saved center

Moves all servos to their saved `center_position` from config. Use this to verify calibration or reset ear positions.

```bash
poetry run python ears_calibration.py go-center
```

## Typical Calibration Workflow

**First-time setup (new servos):**

```
1. Power on adapter (5V)
2. poetry run python ears_calibration.py assign-all
3. Position ears straight up, fins parallel to eyeline
4. poetry run python ears_calibration.py set-center
5. Move ears by hand, then verify:
   poetry run python ears_calibration.py go-center
```

**Re-calibration (servos already have IDs):**

```
1. Position ears at desired center
2. poetry run python ears_calibration.py set-center
3. Verify: poetry run python ears_calibration.py go-center
```

**Replacing a servo:**

```
1. Connect only the new servo (disconnect others or power off)
2. poetry run python ears_calibration.py set-id 1 <target_id>
3. Reconnect all servos
4. Re-run set-center if needed
```

## Troubleshooting

**"Failed to open port"** — Check USB cable and `/dev/waveshare_ears` symlink:
```bash
ls -la /dev/waveshare_ears
```

**"No servo found"** — Check power is on and servo is connected to the bus.

**"RX timeout" (comm_result -7)** — Servo not powered or wrong baud rate.

**Servo IDs got reset** — SCS0009 IDs are saved in EEPROM and persist through power cycles. If a servo responds at ID 1, it may be a new/replacement servo. Re-assign with `set-id`.
