#!/usr/bin/env python3
"""Calibration tool for Feetech SCS0009 ear servos via Waveshare adapter.

Usage:
    python ears_calibration.py assign-all   # Interactive: assign IDs 4-7 one by one
    python ears_calibration.py set-id <current_id> <new_id>
"""

import argparse
import sys

from scservo_sdk import PortHandler, COMM_SUCCESS
from scservo_sdk.scscl import scscl, scs_id as ADDR_ID

# -- Hardware constants --
DEVICE_PORT = "/dev/waveshare_ears"
BAUDRATE = 1_000_000
FACTORY_DEFAULT_ID = 1

# -- Ear servo ID assignments --
SERVO_ASSIGNMENTS = [
    (4, "Left ear",  "Base rotation"),
    (5, "Left ear",  "Ear angle"),
    (6, "Right ear", "Base rotation"),
    (7, "Right ear", "Ear angle"),
]


def open_port(device: str = DEVICE_PORT, baudrate: int = BAUDRATE) -> tuple[PortHandler, scscl]:
    """Open serial port and return (port_handler, packet_handler)."""
    port = PortHandler(device)
    if not port.openPort():
        print(f"[ERROR] Failed to open port: {device}")
        sys.exit(1)
    if not port.setBaudRate(baudrate):
        print(f"[ERROR] Failed to set baud rate: {baudrate}")
        sys.exit(1)
    packet = scscl(port)
    print(f"[INFO] Port opened: {device} @ {baudrate} bps")
    return port, packet


def set_servo_id(packet: scscl, current_id: int, new_id: int) -> bool:
    """Change a servo's ID.

    WARNING: Connect only ONE servo at a time when using this function.
    Factory default ID is 1 — multiple servos on the bus with the same ID
    will cause bus conflicts.

    Args:
        packet: scscl packet handler (already connected).
        current_id: The servo's current ID (1-253).
        new_id: The desired new ID (1-253).

    Returns:
        True if ID was changed successfully.
    """
    # 1. Ping servo at current_id to verify it's on the bus
    _, result, _ = packet.ping(current_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] No servo found at ID {current_id}: {packet.getTxRxResult(result)}")
        return False
    print(f"[INFO] Servo found at ID {current_id}")

    # 2. Unlock EEPROM
    result, error = packet.unLockEprom(current_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Failed to unlock EEPROM: {packet.getTxRxResult(result)}")
        return False
    if error != 0:
        print(f"[ERROR] Servo error on unlock: {packet.getRxPacketError(error)}")
        return False

    # 3. Write new ID
    result, error = packet.write1ByteTxRx(current_id, ADDR_ID, new_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Failed to write new ID: {packet.getTxRxResult(result)}")
        return False
    if error != 0:
        print(f"[ERROR] Servo error on ID write: {packet.getRxPacketError(error)}")
        return False

    # 4. Lock EEPROM (address with new_id now)
    packet.LockEprom(new_id)

    # 5. Verify by pinging new ID
    _, result, _ = packet.ping(new_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Verification failed — no response at new ID {new_id}")
        return False

    print(f"[OK] Servo ID changed: {current_id} -> {new_id}")
    return True


def assign_all(packet: scscl) -> bool:
    """Assign IDs 4-7 to ear servos, adding one at a time to the daisy chain.

    Each new servo arrives with factory default ID 1. After reassignment
    it no longer conflicts, so just keep adding the next servo to the chain.
    """
    print("=== EAR SERVO ID ASSIGNMENT ===\n")
    print("Assignments:")
    for target_id, location, function in SERVO_ASSIGNMENTS:
        print(f"  ID {target_id}: {location} - {function}")
    print()

    for i, (target_id, location, function) in enumerate(SERVO_ASSIGNMENTS, 1):
        input(f"[{i}/4] Add servo for {location} / {function} (will become ID {target_id}), "
              f"then press Enter...")

        ok = set_servo_id(packet, FACTORY_DEFAULT_ID, target_id)
        if not ok:
            print(f"[ERROR] Failed to assign ID {target_id}. Fix and retry.")
            return False

        print()

    print("=== ALL 4 SERVOS ASSIGNED ===")
    print("All servos are on the chain with IDs 4, 5, 6, 7.")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="SCS0009 ear servo calibration tool")
    subparsers = parser.add_subparsers(dest="command")

    # -- ping command --
    sp_ping = subparsers.add_parser(
        "ping", help="Ping a servo ID to check if it responds"
    )
    sp_ping.add_argument("servo_id", type=int, help="Servo ID to ping (1-253)")

    # -- assign-all command --
    subparsers.add_parser(
        "assign-all", help="Assign IDs 4-7 to ear servos interactively"
    )

    # -- set-id command --
    sp_id = subparsers.add_parser(
        "set-id", help="Change a single servo's ID"
    )
    sp_id.add_argument("current_id", type=int, help="Current servo ID (default factory: 1)")
    sp_id.add_argument("new_id", type=int, help="Desired new ID (1-253)")

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(0)

    port, packet = open_port()

    try:
        if args.command == "ping":
            _, result, _ = packet.ping(args.servo_id)
            if result == COMM_SUCCESS:
                print(f"[OK] Servo ID {args.servo_id} responded")
            else:
                print(f"[FAIL] No response from ID {args.servo_id}")
            sys.exit(0 if result == COMM_SUCCESS else 1)
        elif args.command == "assign-all":
            ok = assign_all(packet)
            sys.exit(0 if ok else 1)
        elif args.command == "set-id":
            ok = set_servo_id(packet, args.current_id, args.new_id)
            sys.exit(0 if ok else 1)
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
