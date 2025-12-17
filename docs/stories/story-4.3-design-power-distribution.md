# Story 4.3: Design Power Distribution System for 36V Battery

**Epic:** Epic 4 - Base Module Build
**Status:** Not Started
**Priority:** High
**Estimated Effort:** 4-6 hours

---

## User Story

**As a** builder,
**I want** a robust power distribution system that safely converts 36V battery power to multiple voltage rails,
**so that** all robot modules receive appropriate power levels with protection against overcurrent and reverse polarity.

---

## Acceptance Criteria

1. ✅ Power distribution schematic created showing all voltage rails
2. ✅ 36V → 5V/5A buck converter specified for Raspberry Pi 5
3. ✅ 36V → 12V/2A buck converter specified for ESP32s and peripherals
4. ✅ 36V direct connection to ODrive with 15A fuse protection
5. ✅ Reverse polarity protection implemented (diode or MOSFET)
6. ✅ Emergency stop circuit integrated (NC button cuts motor power)
7. ✅ Battery voltage monitoring circuit designed (voltage divider to ESP32 ADC)
8. ✅ Current sensing capability added (INA219 or shunt resistor)
9. ✅ Connector types specified: XT60 (battery), USB-C (Pi), JST (modules)
10. ✅ Wire gauge chart created for all power connections

---

## Implementation Steps

### 1. Define Power Requirements for All Modules

**Power budget analysis:**

```
Module Power Requirements:
─────────────────────────────────────────────────────
Module              Voltage    Current    Power    Notes
─────────────────────────────────────────────────────
Raspberry Pi 5      5V         5A max     25W      USB-C PD
Head+Ears ESP32     5V         0.5A       2.5W     Via 12V regulator
Torso ESP32         5V         0.5A       2.5W     Via 12V regulator
Neck ESP32          5V         0.5A       2.5W     Via 12V regulator
Base ESP32          5V         0.5A       2.5W     Via 12V regulator
OLEDs (×2)          3.3V       0.2A       0.7W     From ESP32
Servos (×6)         5V         2A peak    10W      From 12V rail
Heart Display       5V         0.3A       1.5W     From Torso ESP32
Thermal Printer     5V         2A peak    10W      From 12V rail
ODrive S1           36V        10A peak   360W     Direct from battery
Motors (×2)         36V        10A total  360W     Via ODrive
─────────────────────────────────────────────────────
Total Peak Power:                         ~782W
Typical Operation:                        ~200W
─────────────────────────────────────────────────────

Battery Capacity:
36V × 4.4Ah = 158Wh
Runtime at 200W average: ~45 minutes
Runtime at 100W cruise: ~90 minutes
```

### 2. Design Main Power Distribution Architecture

**Power tree diagram:**

```
                    36V Battery (4.4Ah)
                           |
                    Reverse Polarity Protection (MOSFET)
                           |
                    Main Power Switch (E-stop)
                           |
        ┌──────────────────┼──────────────────┐
        │                  │                  │
    15A Fuse           5A Fuse           2A Fuse
        │                  │                  │
    ODrive S1       Buck Converter 1   Buck Converter 2
    (36V direct)      (36V → 5V/5A)     (36V → 12V/2A)
        │                  │                  │
    Motors (×2)     Raspberry Pi 5      ┌─────┴─────┐
                                        │           │
                                   Servo Rail   ESP32s (×4)
                                   (12V → 5V)   (12V → 5V LDO)
```

### 3. Specify Buck Converters

**Buck Converter 1 (Raspberry Pi):**
```
Model: LM2596HVS or XL4016 (high voltage input)
Input: 36V (30-42V range)
Output: 5V
Current: 5A continuous, 8A peak
Efficiency: ~85%
Features:
  - Adjustable output (trim pot)
  - Over-current protection
  - Thermal shutdown
  - Short circuit protection
Cost: $8-12
Recommended: DROK LM2596HVS
```

**Buck Converter 2 (Peripherals):**
```
Model: LM2596 or similar
Input: 36V
Output: 12V
Current: 2A continuous
Efficiency: ~90%
Features:
  - Adjustable output
  - OCP, thermal shutdown
Cost: $5-8
Recommended: Generic LM2596 module
```

### 4. Design Reverse Polarity Protection

**MOSFET-based protection (preferred):**

```
Schematic:
Battery (+) ──→ P-Channel MOSFET Drain
                MOSFET Source ──→ System (+)
                MOSFET Gate ──→ Battery (-)

Battery (-) ──→ System (-)

MOSFET: IRF4905 or similar
- VDS: 55V (above 42V max battery voltage)
- ID: 20A continuous
- RDS(on): <0.02Ω (low voltage drop)
- Gate threshold: -4V

When battery connected correctly:
  - Gate pulled to GND → MOSFET conducts
When battery reversed:
  - Gate not pulled to GND → MOSFET blocks current

Advantages:
  - Very low voltage drop (~0.2V at 10A)
  - No heat dissipation
  - Fast switching
```

**Alternative: Diode protection (simpler but less efficient):**

```
Battery (+) ──→ Schottky Diode Anode
                Diode Cathode ──→ System (+)

Diode: MBRF20200CT or similar
- Forward voltage: 0.5V (power loss at 10A: 5W)
- Current: 20A
- Less efficient but simpler

Use diode if MOSFET circuit too complex
```

### 5. Design Emergency Stop Circuit

**E-stop wiring:**

```
Battery (+) ──→ E-Stop Button (NC contact)
                Button output ──→ Main Relay Coil (+)
                Relay Coil (-) ──→ Battery (-)

Main Relay Contacts:
  NO contact: Battery (+) to ODrive (+)

When E-stop pressed:
  - NC contact opens
  - Relay coil de-energized
  - NO contact opens
  - ODrive loses power
  - Motors stop immediately

When E-stop released:
  - NC contact closes
  - Relay energized
  - Power restored

Relay: 40A automotive relay
Button: 40mm red mushroom button (NC contact)
```

### 6. Design Battery Voltage Monitoring

**Voltage divider circuit:**

```
Battery (+) ──→ R1 (100kΩ) ──→ ADC input (ESP32)
                            │
                            R2 (10kΩ) ──→ GND

Voltage divider ratio: 11:1
Battery voltage range: 30-42V
ADC input range: 2.7-3.8V (safe for ESP32 3.3V ADC)

ESP32 ADC reading to voltage conversion:
  Vbattery = ADC_reading × (R1 + R2) / R2
  Vbattery = ADC_reading × 11

Use 1% resistors for accuracy
Add 0.1µF capacitor across R2 for noise filtering
```

### 7. Design Current Sensing

**Option A: INA219 current sensor module:**

```
INA219 module on 5V rail (Pi monitoring)
- I2C interface (address 0x40)
- Measures current and voltage
- Range: 0-3.2A
- Accuracy: ±0.5%
- Cost: $3-5

Connection:
  Battery (+) → INA219 V+ → Pi buck converter V+
  INA219 I2C → Pi I2C bus

Software reads current draw to estimate battery life
```

**Option B: Shunt resistor (ODrive monitoring):**

```
Shunt resistor: 0.01Ω, 3W
Placed in series with ODrive ground return
Voltage across shunt = Current × 0.01

At 10A: 0.1V drop
Amplify with op-amp (10×) → 1V
Feed to ESP32 ADC (0-3.3V range)

More complex but monitors high-current ODrive
```

### 8. Specify Connectors and Wire Gauges

**Connector specifications:**

```
Connection              Connector Type      Wire Gauge   Current
────────────────────────────────────────────────────────────────
Battery to main bus     XT60                12 AWG       15A
Main bus to ODrive      XT60                12 AWG       15A
Main bus to buck conv   XT30                16 AWG       5A
Buck 1 to Pi            USB-C cable         20 AWG       5A
Buck 2 to ESP32s        JST-XH 2-pin        22 AWG       2A
ESP32 to modules        Dupont 4-pin        24 AWG       1A
Ground returns          14 AWG              14 AWG       10A
────────────────────────────────────────────────────────────────

Wire types:
  - Silicone wire (flexible, high temp)
  - Stranded copper (not solid)
  - Pre-tinned if possible
```

### 9. Create Power Distribution PCB Layout Plan

**PCB requirements (for Story 4.4):**

```
Base Power Distribution PCB:
- Input: XT60 connector for battery
- Outputs:
  - XT60 for ODrive
  - Screw terminals for buck converters
  - USB-C port for Pi (or screw terminals)
  - 4× JST connectors for ESP32s (12V)
- Protection:
  - MOSFET reverse polarity protection
  - Fuse holders (×3): 15A, 5A, 2A
- Monitoring:
  - INA219 footprint
  - Voltage divider components
- Size: ~100mm × 150mm
- Copper weight: 2oz (thick traces for high current)
```

### 10. Document Power System

**Create `modules/base/power_distribution.md`:**

```markdown
# Base Module Power Distribution System

## Voltage Rails
- 36V: Battery → ODrive (motors)
- 5V: Pi, servos, displays
- 12V: ESP32s (via onboard regulators)
- 3.3V: Logic (from ESP32/Pi)

## Buck Converters
1. LM2596HVS: 36V → 5V, 5A (Pi)
2. LM2596: 36V → 12V, 2A (peripherals)

## Protection
- Reverse polarity: IRF4905 P-MOSFET
- Overcurrent: Fuses (15A, 5A, 2A)
- Emergency stop: 40A relay + NC button

## Monitoring
- Battery voltage: Voltage divider to ESP32 ADC
- Pi current: INA219 on 5V rail

## Connectors
- Battery: XT60
- ODrive: XT60
- Pi: USB-C
- ESP32s: JST-XH

## Wire Gauges
- Battery to bus: 12 AWG
- Buck converters: 16 AWG
- Module power: 22 AWG

## Power Budget
- Total peak: 782W
- Typical: 200W
- Battery runtime: ~45 minutes

## Schematic
[Insert schematic diagram]

## PCB Layout
[Include PCB design from Story 4.4]
```

---

## Testing & Validation

**Test 1: Voltage Rails Under Load**
```bash
# Connect all loads, measure voltages:
#   5V rail: 4.95-5.05V (tight tolerance for Pi)
#   12V rail: 11.8-12.2V
```

**Test 2: Reverse Polarity Protection**
```bash
# Connect battery backwards
# MOSFET should block current (no damage to system)
# Warning: Only test once! May damage components if protection fails
```

**Test 3: Current Monitoring**
```bash
# Load 5V rail with 1A, 2A, 3A
# INA219 should read accurate current within ±0.5%
```

**Test 4: Emergency Stop**
```bash
# Press E-stop during motor operation
# Motors should stop within 100ms
# Power should cut completely (no residual movement)
```

---

## Troubleshooting

**Issue 1: Buck Converter Output Voltage Drifts**
- **Solution:** Add output capacitor (100-470µF), check thermal performance, reduce load

**Issue 2: MOSFET Gets Hot**
- **Solution:** Check RDS(on), verify correct P-channel type, add heatsink

**Issue 3: Voltage Monitoring Inaccurate**
- **Solution:** Use 1% tolerance resistors, add capacitor for filtering, calibrate ADC

**Issue 4: E-stop Doesn't Cut Power**
- **Solution:** Verify NC contact (not NO), check relay coil voltage, test relay operation

---

## Dependencies

**Before this story:**
- Story 4.2: Breadboard Base Components ✅

**After this story:**
- Story 4.4: Design Base Custom PCB

---

## References

- [Buck Converter Design Guide](https://www.ti.com/lit/an/slva477b/slva477b.pdf)
- [Reverse Polarity Protection](https://www.ti.com/lit/an/slva139/slva139.pdf)
- [Wire Gauge Chart](https://www.powerstream.com/Wire_Size.htm)

---

## Notes

- **Safety First:** Reverse polarity protection mandatory - wrong battery connection can destroy electronics instantly
- **Fuse Sizing:** Choose fuses 20-30% above normal operating current for safety margin
- **Heat Management:** Buck converters generate heat - add heatsinks if output current >3A
- **Emergency Stop:** Critical safety feature - test thoroughly before full robot operation
- **Power Budget:** Design for peak load, not average - motors can draw 10A instantaneously
- **Future Upgrades:** Consider adding battery charging circuitry, fuel gauge IC (BQ34Z100), CAN bus for distributed power management

---

**Created:** 2025-12-17
**Last Updated:** 2025-12-17
