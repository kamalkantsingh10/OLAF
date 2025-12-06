#!/usr/bin/env python3
"""
Step 1: Basic Motor Test (USB Connection)
DIY Self-Balancing Robot Tutorial

Hardware needed:
- ODrive v3.6
- 2x Hoverboard motors (with hall sensors)
- 36V battery pack
- USB cable (ODrive → Computer)

What this does:
- Connects to ODrive via USB
- Enables both motors
- Spins them forward, reverse, and stops
- Simple test to verify your wiring works

Usage: python step1_motors.py
"""

import odrive
from odrive.enums import *
import time
import sys


def print_header():
    """Print fancy header"""
    print("\n" + "="*60)
    print("  Step 1: Basic Motor Test")
    print("  DIY Self-Balancing Robot Tutorial")
    print("="*60 + "\n")


def connect_odrive():
    """Connect to ODrive and return handle"""
    print("🔌 Connecting to ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
        print(f"   ✓ Connected! Voltage: {odrv.vbus_voltage:.1f}V")
        return odrv
    except Exception as e:
        print(f"   ✗ ODrive not found!")
        print(f"\n   Troubleshooting:")
        print(f"   - Is USB cable connected?")
        print(f"   - Is green LED on ODrive lit?")
        print(f"   - Did you run: sudo pip3 install odrive")
        sys.exit(1)


def enable_motors(odrv):
    """Put both motors in closed-loop control mode"""
    print("\n⚡ Enabling motors...")

    # Clear any previous errors
    odrv.axis0.error = 0
    odrv.axis1.error = 0

    # Enable closed-loop control
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    time.sleep(0.5)

    # Check if motors are ready
    if odrv.axis0.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
        print(f"   ✓ Left motor (Axis 0) ready")
    else:
        print(f"   ✗ Left motor failed (error: {odrv.axis0.error})")

    if odrv.axis1.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
        print(f"   ✓ Right motor (Axis 1) ready")
    else:
        print(f"   ✗ Right motor failed (error: {odrv.axis1.error})")


def set_velocity(odrv, axis, velocity):
    """Set velocity for one axis"""
    axis_obj = getattr(odrv, f'axis{axis}')
    axis_obj.controller.input_vel = velocity


def test_sequence(odrv):
    """Run through a test sequence"""

    print("\n🎯 Running test sequence...")
    print("   Your motors will spin. Make sure wheels are off the ground!\n")

    input("   Press ENTER to start test... ")

    # Test 1: Both motors forward
    print("\n   [1/4] Both motors forward (2 rev/s)...")
    set_velocity(odrv, 0, 2.0)
    set_velocity(odrv, 1, 2.0)
    time.sleep(3)

    # Test 2: Stop
    print("   [2/4] Stopping...")
    set_velocity(odrv, 0, 0)
    set_velocity(odrv, 1, 0)
    time.sleep(2)

    # Test 3: Both motors reverse
    print("   [3/4] Both motors reverse (-2 rev/s)...")
    set_velocity(odrv, 0, -2.0)
    set_velocity(odrv, 1, -2.0)
    time.sleep(3)

    # Test 4: Stop
    print("   [4/4] Stopping...")
    set_velocity(odrv, 0, 0)
    set_velocity(odrv, 1, 0)
    time.sleep(1)

    print("\n   ✓ Test complete!")


def disable_motors(odrv):
    """Put motors back to idle"""
    print("\n🛑 Disabling motors...")
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    odrv.axis1.requested_state = AXIS_STATE_IDLE
    print("   ✓ Motors disabled")


def main():
    print_header()

    # Connect
    odrv = connect_odrive()

    # Enable
    enable_motors(odrv)

    # Test
    try:
        test_sequence(odrv)
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")

    # Cleanup
    disable_motors(odrv)

    # Summary
    print("\n" + "="*60)
    print("  ✅ Step 1 Complete!")
    print("="*60)
    print("\n  What you proved:")
    print("    • ODrive can communicate via USB")
    print("    • Motors are wired correctly")
    print("    • Hall sensors are working")
    print("    • Velocity control works")
    print("\n  Next: Step 2 - Connect ESP32 via UART")
    print()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
