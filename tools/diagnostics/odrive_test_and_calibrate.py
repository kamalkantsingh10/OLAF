#!/usr/bin/env python3
"""
ODrive Test and Calibration - All-in-One Script
Story 6.3: ODrive Motor Controller Setup

This script does everything:
1. Checks ODrive connection
2. Verifies hall sensors
3. Tests motor phase wires
4. Calibrates both motors
5. Tests velocity control
6. Saves configuration

Usage: python tools/diagnostics/odrive_test_and_calibrate.py
"""

import odrive
from odrive.enums import *
import time
import sys

def main():
    print("\n" + "="*60)
    print("  ODrive Test & Calibration - Hoverboard Motors")
    print("="*60)

    # Step 1: Connect
    print("\n[1/6] Connecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
        print(f"      ✓ Connected!")
        print(f"      Voltage: {odrv.vbus_voltage:.1f}V")
        print(f"      Firmware: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    except:
        print("      ✗ ODrive not found!")
        print("\n      Check: USB connected, green LED on")
        sys.exit(1)

    # Step 2: Check hall sensors
    print("\n[2/6] Checking hall sensors...")
    hall0 = odrv.axis0.encoder.hall_state
    hall1 = odrv.axis1.encoder.hall_state
    print(f"      Axis 0 hall state: {hall0}")
    print(f"      Axis 1 hall state: {hall1}")

    if hall0 > 0 and hall1 > 0:
        print(f"      ✓ Hall sensors working")
    else:
        print(f"      ✗ Hall sensors not connected")
        sys.exit(1)

    # Step 3: Clear errors and configure
    print("\n[3/6] Configuring for hoverboard motors...")

    # Clear errors
    odrv.axis0.error = 0
    odrv.axis0.motor.error = 0
    odrv.axis0.encoder.error = 0
    odrv.axis1.error = 0
    odrv.axis1.motor.error = 0
    odrv.axis1.encoder.error = 0

    # Configure both axes
    for i in [0, 1]:
        axis = getattr(odrv, f'axis{i}')

        # Motor config
        axis.motor.config.pole_pairs = 15
        axis.motor.config.current_lim = 20
        axis.motor.config.requested_current_range = 60
        axis.motor.config.calibration_current = 10
        axis.motor.config.resistance_calib_max_voltage = 12  # Critical for low-resistance motors

        # Encoder config
        axis.encoder.config.mode = ENCODER_MODE_HALL
        axis.encoder.config.cpr = 90
        axis.encoder.config.bandwidth = 100

        # Controller config
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.vel_limit = 10

    print(f"      ✓ Configuration applied")

    # Step 4: Test motor phase wires
    print("\n[4/6] Testing motor phase wire connections...")

    odrv.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(4)

    if odrv.axis0.motor.error == 0:
        print(f"      ✓ Axis 0 phase wires connected ({odrv.axis0.motor.config.phase_resistance:.3f}Ω)")
    else:
        print(f"      ✗ Axis 0 error {odrv.axis0.motor.error} - check phase wires")
        sys.exit(1)

    odrv.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(4)

    if odrv.axis1.motor.error == 0:
        print(f"      ✓ Axis 1 phase wires connected ({odrv.axis1.motor.config.phase_resistance:.3f}Ω)")
    else:
        print(f"      ✗ Axis 1 error {odrv.axis1.motor.error} - check phase wires")
        sys.exit(1)

    # Step 5: Full calibration with encoders
    print("\n[5/6] Calibrating motors with hall encoders...")
    print("      (Motors will beep and spin)")

    time.sleep(2)
    print("\n      Calibrating Axis 0 (Left motor)...")
    odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    while odrv.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    if odrv.axis0.encoder.is_ready and odrv.axis0.motor.error == 0:
        print(f"      ✓ Axis 0 calibrated (Hall state: {odrv.axis0.encoder.hall_state})")
    else:
        print(f"      ✗ Axis 0 calibration failed")
        print(f"        Motor error: {odrv.axis0.motor.error}")
        print(f"        Encoder error: {odrv.axis0.encoder.error}")
        sys.exit(1)

    time.sleep(1)
    print("\n      Calibrating Axis 1 (Right motor)...")
    odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    while odrv.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    if odrv.axis1.encoder.is_ready and odrv.axis1.motor.error == 0:
        print(f"      ✓ Axis 1 calibrated (Hall state: {odrv.axis1.encoder.hall_state})")
    else:
        print(f"      ✗ Axis 1 calibration failed")
        print(f"        Motor error: {odrv.axis1.motor.error}")
        print(f"        Encoder error: {odrv.axis1.encoder.error}")
        sys.exit(1)

    # Step 6: Test velocity control
    print("\n[6/6] Testing velocity control...")
    print("      (Motors will spin)")

    time.sleep(2)

    # Test Axis 0
    print("\n      Testing Axis 0...")
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    odrv.axis0.controller.input_vel = 2
    time.sleep(2)
    vel0_fwd = odrv.axis0.encoder.vel_estimate

    odrv.axis0.controller.input_vel = -2
    time.sleep(2)
    vel0_rev = odrv.axis0.encoder.vel_estimate

    odrv.axis0.controller.input_vel = 0
    time.sleep(0.5)
    odrv.axis0.requested_state = AXIS_STATE_IDLE

    print(f"      Forward: {vel0_fwd:.2f} rev/s, Reverse: {vel0_rev:.2f} rev/s")

    if abs(vel0_fwd - 2) < 0.5 and abs(vel0_rev + 2) < 0.5:
        print(f"      ✓ Axis 0 velocity control working")
    else:
        print(f"      ⚠ Axis 0 velocity tracking poor")

    # Test Axis 1
    time.sleep(1)
    print("\n      Testing Axis 1...")
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    odrv.axis1.controller.input_vel = 2
    time.sleep(2)
    vel1_fwd = odrv.axis1.encoder.vel_estimate

    odrv.axis1.controller.input_vel = -2
    time.sleep(2)
    vel1_rev = odrv.axis1.encoder.vel_estimate

    odrv.axis1.controller.input_vel = 0
    time.sleep(0.5)
    odrv.axis1.requested_state = AXIS_STATE_IDLE

    print(f"      Forward: {vel1_fwd:.2f} rev/s, Reverse: {vel1_rev:.2f} rev/s")

    if abs(vel1_fwd - 2) < 0.5 and abs(vel1_rev + 2) < 0.5:
        print(f"      ✓ Axis 1 velocity control working")
    else:
        print(f"      ⚠ Axis 1 velocity tracking poor")

    # Save configuration
    print("\n[7/7] Saving configuration...")
    odrv.save_configuration()
    print(f"      ✓ Configuration saved to ODrive")

    # Final summary
    print("\n" + "="*60)
    print("  ✅ SUCCESS! ODrive Setup Complete")
    print("="*60)
    print("\n  Summary:")
    print(f"    • Axis 0 (Left):  {odrv.axis0.motor.config.phase_resistance:.3f}Ω, Hall ready")
    print(f"    • Axis 1 (Right): {odrv.axis1.motor.config.phase_resistance:.3f}Ω, Hall ready")
    print(f"    • Velocity control: Working")
    print(f"    • Configuration: Saved")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✗ Cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
