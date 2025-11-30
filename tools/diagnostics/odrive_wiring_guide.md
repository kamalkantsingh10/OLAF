# ODrive Wiring Guide for Hoverboard Motors

## Current Status
- ✅ ODrive powered (40V)
- ✅ Hall sensors connected and working
- ❌ Motor phase wires NOT connected (causing Error 1)

## What Needs to Be Connected

### Left Motor (Axis 0) → ODrive M0

**Motor Phase Wires (3 thick wires):**
```
Left Motor Phase A → ODrive M0 terminal A (blue screw terminal)
Left Motor Phase B → ODrive M0 terminal B (blue screw terminal)
Left Motor Phase C → ODrive M0 terminal C (blue screw terminal)
```

**Hall Sensor Wires (5 thin wires) - ALREADY DONE ✅:**
```
Hall VCC → ODrive M0 5V
Hall GND → ODrive M0 GND
Hall A   → ODrive M0 HA
Hall B   → ODrive M0 HB
Hall C   → ODrive M0 HC
```

### Right Motor (Axis 1) → ODrive M1

**Motor Phase Wires (3 thick wires):**
```
Right Motor Phase A → ODrive M1 terminal A (blue screw terminal)
Right Motor Phase B → ODrive M1 terminal B (blue screw terminal)
Right Motor Phase C → ODrive M1 terminal C (blue screw terminal)
```

**Hall Sensor Wires (5 thin wires) - ALREADY DONE ✅:**
```
Hall VCC → ODrive M1 5V
Hall GND → ODrive M1 GND
Hall A   → ODrive M1 HA
Hall B   → ODrive M1 HB
Hall C   → ODrive M1 HC
```

## How to Identify the Wires

**Hoverboard motors typically have:**
- **3 THICK wires** = Motor phase wires (A, B, C)
  - Colors: Often yellow/green/blue OR red/yellow/blue
  - Thickness: ~16-18 AWG (thick)
  - These carry motor current (up to 20A)

- **5 THIN wires** = Hall sensor wires
  - Colors: Usually red/black + 3 signal wires
  - Thickness: ~22-24 AWG (thin)
  - These carry sensor signals

## Connection Steps

1. **Strip the 3 thick phase wires** from each motor (about 5-7mm)

2. **Open ODrive screw terminals** M0 (left motor) and M1 (right motor)
   - Use small flathead screwdriver
   - Turn counterclockwise to loosen

3. **Insert phase wires into terminals:**
   - Phase A → Terminal A
   - Phase B → Terminal B
   - Phase C → Terminal C
   - **Note:** Order doesn't matter initially - can swap later if motor spins backward

4. **Tighten screw terminals** firmly
   - Turn clockwise until wire is secure
   - Tug gently to verify connection

5. **Test by hand:**
   - Try spinning motor by hand
   - Should feel resistance (cogging) when phase wires connected

## After Connecting Phase Wires

Run this command to test:
```bash
python tools/calibration/odrive_quick_test.py
```

Motors should:
- Beep during calibration
- Spin smoothly
- No Error 1 anymore

## Safety Notes

⚠️ **Before connecting:**
- Ensure motors can spin freely (not blocked)
- Keep hands away during calibration (motors will spin)
- Have emergency power cutoff ready

## Visual Reference

```
ODrive v3.6 Layout:
┌─────────────────────────────────────┐
│                                     │
│   M0 (Left Motor)    M1 (Right)    │
│   ┌───────────┐     ┌───────────┐  │
│   │ A B C     │     │ A B C     │  │ ← Big blue terminals (PHASE WIRES)
│   └───────────┘     └───────────┘  │
│                                     │
│   ┌───────────┐     ┌───────────┐  │
│   │Hall pins  │     │Hall pins  │  │ ← Small pins (HALL SENSORS) ✅
│   └───────────┘     └───────────┘  │
│                                     │
│         XT60 Power Input            │ ← 12V power ✅
└─────────────────────────────────────┘
```

## What You Should See

**After connecting phase wires correctly:**
- Hall state: 1-7 (changes when motor rotates) ✅ Already good
- Motor error: 0 (no errors) ← Will be fixed after wiring
- Encoder ready: True ← Will be fixed after wiring
- Motors spin during calibration ← Will work after wiring
