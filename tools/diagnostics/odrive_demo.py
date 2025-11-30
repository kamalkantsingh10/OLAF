#!/usr/bin/env python3
"""
ODrive Motor Demo - Show off your working motors!

This script demonstrates various motor movements:
- Forward/reverse
- Both wheels together
- Differential turning
- Speed ramping

Usage: python tools/diagnostics/odrive_demo.py
"""

import odrive
from odrive.enums import *
import time
import sys

def connect_odrive():
    """Connect to ODrive"""
    print("\n" + "="*60)
    print("  ODrive Motor Demo")
    print("="*60)
    print("\nConnecting to ODrive...")

    try:
        odrv = odrive.find_any(timeout=10)
        print(f"✓ Connected - Voltage: {odrv.vbus_voltage:.1f}V\n")
        return odrv
    except:
        print("✗ ODrive not found!")
        sys.exit(1)

def enable_motors(odrv):
    """Enable closed-loop control on both motors"""
    print("Enabling motors...")
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)
    print("✓ Motors enabled\n")

def disable_motors(odrv):
    """Disable motors"""
    print("\nStopping motors...")
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    time.sleep(0.5)
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    odrv.axis1.requested_state = AXIS_STATE_IDLE
    print("✓ Motors stopped\n")

def demo_forward_reverse(odrv):
    """Demo: Forward and reverse movement"""
    print("="*60)
    print("Demo 1: Forward and Reverse")
    print("="*60)

    print("\nMoving forward at 2 rev/s for 3 seconds...")
    odrv.axis0.controller.input_vel = 2
    odrv.axis1.controller.input_vel = 2
    time.sleep(3)

    print("Stopping...")
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    time.sleep(1)

    print("Moving reverse at -2 rev/s for 3 seconds...")
    odrv.axis0.controller.input_vel = -2
    odrv.axis1.controller.input_vel = -2
    time.sleep(3)

    print("Stopping...")
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    time.sleep(1)
    print("✓ Forward/reverse complete\n")

def demo_turning(odrv):
    """Demo: Differential turning"""
    print("="*60)
    print("Demo 2: Turning (Differential Drive)")
    print("="*60)

    print("\nTurning right (left wheel faster)...")
    odrv.axis0.controller.input_vel = 3  # Left faster
    odrv.axis1.controller.input_vel = 1  # Right slower
    time.sleep(2)

    print("Stopping...")
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    time.sleep(1)

    print("Turning left (right wheel faster)...")
    odrv.axis0.controller.input_vel = 1  # Left slower
    odrv.axis1.controller.input_vel = 3  # Right faster
    time.sleep(2)

    print("Stopping...")
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    time.sleep(1)
    print("✓ Turning demo complete\n")

def demo_spin_in_place(odrv):
    """Demo: Spin in place (tank turn)"""
    print("="*60)
    print("Demo 3: Spin in Place")
    print("="*60)

    print("\nSpinning clockwise...")
    odrv.axis0.controller.input_vel = 2   # Left forward
    odrv.axis1.controller.input_vel = -2  # Right reverse
    time.sleep(2)

    print("Stopping...")
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    time.sleep(1)

    print("Spinning counter-clockwise...")
    odrv.axis0.controller.input_vel = -2  # Left reverse
    odrv.axis1.controller.input_vel = 2   # Right forward
    time.sleep(2)

    print("Stopping...")
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 0
    time.sleep(1)
    print("✓ Spin demo complete\n")

def demo_speed_ramp(odrv):
    """Demo: Speed ramping"""
    print("="*60)
    print("Demo 4: Speed Ramping")
    print("="*60)

    print("\nRamping up from 0 to 4 rev/s...")
    for vel in [0, 1, 2, 3, 4]:
        print(f"  Speed: {vel} rev/s")
        odrv.axis0.controller.input_vel = vel
        odrv.axis1.controller.input_vel = vel
        time.sleep(1)

    print("Ramping down from 4 to 0 rev/s...")
    for vel in [3, 2, 1, 0]:
        print(f"  Speed: {vel} rev/s")
        odrv.axis0.controller.input_vel = vel
        odrv.axis1.controller.input_vel = vel
        time.sleep(1)

    print("✓ Speed ramp complete\n")

def demo_individual_motors(odrv):
    """Demo: Test each motor individually"""
    print("="*60)
    print("Demo 5: Individual Motor Test")
    print("="*60)

    print("\nTesting LEFT motor only...")
    odrv.axis0.controller.input_vel = 2
    odrv.axis1.controller.input_vel = 0
    time.sleep(2)

    print("Stopping...")
    odrv.axis0.controller.input_vel = 0
    time.sleep(1)

    print("Testing RIGHT motor only...")
    odrv.axis0.controller.input_vel = 0
    odrv.axis1.controller.input_vel = 2
    time.sleep(2)

    print("Stopping...")
    odrv.axis1.controller.input_vel = 0
    time.sleep(1)
    print("✓ Individual motor test complete\n")

def main():
    """Main demo sequence"""
    odrv = connect_odrive()

    # Check if motors are calibrated
    if not odrv.axis0.encoder.is_ready or not odrv.axis1.encoder.is_ready:
        print("⚠ Motors not calibrated!")
        print("Please run calibration first:")
        print("  python tools/diagnostics/odrive_test_and_calibrate.py")
        sys.exit(1)

    print("Motors are calibrated and ready!")
    print("\nThis demo will run 5 tests:")
    print("  1. Forward and reverse")
    print("  2. Turning (differential drive)")
    print("  3. Spin in place (tank turn)")
    print("  4. Speed ramping")
    print("  5. Individual motor test")
    print("\nPress Ctrl+C to stop at any time.")
    print("\nStarting in 3 seconds...")
    time.sleep(3)

    try:
        enable_motors(odrv)

        demo_forward_reverse(odrv)
        time.sleep(1)

        demo_turning(odrv)
        time.sleep(1)

        demo_spin_in_place(odrv)
        time.sleep(1)

        demo_speed_ramp(odrv)
        time.sleep(1)

        demo_individual_motors(odrv)

        disable_motors(odrv)

        print("="*60)
        print("  ✅ Demo Complete!")
        print("="*60)
        print("\nAll motor demonstrations finished successfully.")
        print("Your ODrive is ready for Story 6.4 (IMU integration)!")
        print("="*60 + "\n")

    except KeyboardInterrupt:
        print("\n\nDemo stopped by user")
        disable_motors(odrv)
        sys.exit(0)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
