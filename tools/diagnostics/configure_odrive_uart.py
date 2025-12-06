#!/usr/bin/env python3
"""
ODrive UART Configuration Script
Story 6.3: ODrive Motor Controller Setup

Configures the ODrive's UART interface (GPIO1/GPIO2) for ESP32 communication.
This must be run ONCE before connecting the ESP32.

The UART configuration enables:
- 115200 baud rate on GPIO pins
- ASCII protocol for simple text commands
- Compatibility with ESP32 HardwareSerial

Usage: python tools/diagnostics/configure_odrive_uart.py

After running this script, the ODrive will be ready to receive commands from
the ESP32 via UART on GPIO1 (TX) and GPIO2 (RX).
"""

import odrive
from odrive.enums import *
import time
import sys


def main():
    print("\n" + "="*60)
    print("  ODrive UART Configuration for ESP32")
    print("="*60)

    # Step 1: Connect to ODrive
    print("\n[1/3] Connecting to ODrive via USB...")
    try:
        odrv = odrive.find_any(timeout=10)
        print(f"      ✓ Connected!")
        print(f"      Voltage: {odrv.vbus_voltage:.1f}V")
        print(f"      Firmware: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
    except Exception as e:
        print(f"      ✗ ODrive not found!")
        print(f"\n      Check: USB connected, green LED on")
        print(f"      Error: {e}")
        sys.exit(1)

    # Step 2: Check if already configured
    print("\n[2/3] Checking current UART configuration...")
    current_uart_state = odrv.config.enable_uart_a
    current_baudrate = odrv.config.uart_a_baudrate

    print(f"      Current state:")
    print(f"        UART enabled: {current_uart_state}")
    print(f"        Baud rate: {current_baudrate}")

    if current_uart_state and current_baudrate == 115200:
        print(f"\n      ℹ ODrive UART already configured correctly!")
        print(f"      No changes needed.")

        response = input("\n      Reconfigure anyway? (y/N): ").strip().lower()
        if response != 'y':
            print("\n      Skipping configuration. Exiting.")
            sys.exit(0)

    # Step 3: Configure UART
    print("\n[3/3] Configuring UART on GPIO1/GPIO2...")

    try:
        # Enable UART on stream A (GPIO pins)
        odrv.config.enable_uart_a = True

        # Set baud rate to 115200 (standard for ESP32)
        odrv.config.uart_a_baudrate = 115200

        # Optional: Keep ASCII protocol on USB for debugging
        # Set to False if you only want UART control
        odrv.config.enable_ascii_protocol_on_usb = True

        print(f"      ✓ UART configuration applied:")
        print(f"        • GPIO1 (TX) enabled at 115200 baud")
        print(f"        • GPIO2 (RX) enabled at 115200 baud")
        print(f"        • ASCII protocol ready")
        print(f"        • USB debugging: enabled")

    except Exception as e:
        print(f"      ✗ Configuration failed!")
        print(f"      Error: {e}")
        sys.exit(1)

    # Step 4: Save configuration
    print("\n[4/4] Saving configuration to ODrive...")
    try:
        odrv.save_configuration()
        print(f"      ✓ Configuration saved permanently")

    except Exception as e:
        print(f"      ✗ Save failed!")
        print(f"      Error: {e}")
        sys.exit(1)

    # Step 5: Reboot notice
    print("\n[5/5] Reboot required...")
    print(f"      ODrive needs to reboot for UART to activate")

    response = input("\n      Reboot ODrive now? (Y/n): ").strip().lower()
    if response != 'n':
        try:
            print(f"\n      Rebooting ODrive...")
            odrv.reboot()
            print(f"      ✓ Reboot command sent")
            print(f"\n      Wait 5 seconds for ODrive to restart...")
            time.sleep(5)

        except Exception as e:
            print(f"      ⚠ Reboot command may have failed")
            print(f"      Power cycle the ODrive manually if needed")

    # Final summary
    print("\n" + "="*60)
    print("  ✅ SUCCESS! ODrive UART Configured")
    print("="*60)
    print("\n  Next Steps:")
    print("    1. Wire ESP32 to ODrive:")
    print("       • ODrive GPIO1 → ESP32 GPIO16 (RX2)")
    print("       • ODrive GPIO2 → ESP32 GPIO17 (TX2)")
    print("       • ODrive GND → ESP32 GND")
    print("\n    2. Upload ESP32 firmware with UART enabled")
    print("\n    3. Test with: odriveSerial.print(\"w axis0.requested_state 8\\n\");")
    print("\n  Configuration saved! This only needs to run once.")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✗ Cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
