#!/usr/bin/env python3
"""Calibration tool for Feetech SCS0009 ear servos via Waveshare adapter.

Usage:
    python ears_calibration.py assign-all   # Interactive: assign IDs 4-7 one by one
    python ears_calibration.py set-id <current_id> <new_id>
"""

import argparse
import os
import sys

import yaml
from scservo_sdk import PortHandler, COMM_SUCCESS
from scservo_sdk.scscl import scscl, scs_id as ADDR_ID, SCSCL_PRESENT_POSITION_L

# -- SCS0009 EEPROM registers (not in scscl.py) --
ADDR_OFS_L = 31   # Offset low byte (signed 16-bit)
ADDR_OFS_H = 32   # Offset high byte
CENTER_POSITION = 512  # Mid-range of 0-1023

# -- Load config from central servo-ids.yaml --
CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..", "..", "config", "servo-ids.yaml"
)
FACTORY_DEFAULT_ID = 1


def load_config() -> dict:
    """Load ears config from config/servo-ids.yaml."""
    path = os.path.normpath(CONFIG_PATH)
    with open(path) as f:
        config = yaml.safe_load(f)
    return config["ears"]


def get_servo_assignments(ears_config: dict) -> list[tuple[int, str]]:
    """Build assignment list from config."""
    assignments = []
    for name, info in ears_config["servos"].items():
        assignments.append((info["id"], info["function"]))
    return assignments


def open_port(ears_config: dict) -> tuple[PortHandler, scscl]:
    """Open serial port from config and return (port_handler, packet_handler)."""
    device = ears_config["port"]
    baudrate = ears_config["baudrate"]
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


def assign_all(packet: scscl, ears_config: dict) -> bool:
    """Assign ear servo IDs from config, adding one at a time to the daisy chain.

    Each new servo arrives with factory default ID 1. After reassignment
    it no longer conflicts, so just keep adding the next servo to the chain.
    """
    assignments = get_servo_assignments(ears_config)

    print("=== EAR SERVO ID ASSIGNMENT ===\n")
    print("Assignments (from config/servo-ids.yaml):")
    for target_id, function in assignments:
        print(f"  ID {target_id}: {function}")
    print()

    total = len(assignments)
    for i, (target_id, function) in enumerate(assignments, 1):
        input(f"[{i}/{total}] Add servo for {function} (will become ID {target_id}), "
              f"then press Enter...")

        ok = set_servo_id(packet, FACTORY_DEFAULT_ID, target_id)
        if not ok:
            print(f"[ERROR] Failed to assign ID {target_id}. Fix and retry.")
            return False

        print()

    ids = [str(a[0]) for a in assignments]
    print(f"=== ALL {total} SERVOS ASSIGNED ===")
    print(f"All servos on the chain with IDs {', '.join(ids)}.")
    return True


def set_center(packet: scscl, servo_id: int) -> bool:
    """Set the servo's current physical position as center (512).

    Writes an offset to EEPROM so that the current position reads as 512.
    All future movements will be relative to this zero point.
    Commanding 'go to 512' will return the servo to this physical position.

    Args:
        packet: scscl packet handler.
        servo_id: Servo ID to calibrate.

    Returns:
        True if offset was written successfully.
    """
    # 1. Read current raw position
    current_pos, result, error = packet.ReadPos(servo_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Failed to read position from ID {servo_id}: "
              f"{packet.getTxRxResult(result)}")
        return False
    print(f"[INFO] Servo {servo_id}: current position = {current_pos}")

    # 2. Calculate offset so current position becomes 512
    offset = CENTER_POSITION - current_pos
    print(f"[INFO] Servo {servo_id}: writing offset = {offset}")

    # 3. Unlock EEPROM
    result, error = packet.unLockEprom(servo_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Failed to unlock EEPROM: {packet.getTxRxResult(result)}")
        return False

    # 4. Write offset as signed 16-bit to registers 31-32
    offset_unsigned = offset & 0xFFFF
    result, error = packet.write2ByteTxRx(servo_id, ADDR_OFS_L, offset_unsigned)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Failed to write offset: {packet.getTxRxResult(result)}")
        return False
    if error != 0:
        print(f"[ERROR] Servo error on offset write: {packet.getRxPacketError(error)}")
        return False

    # 5. Lock EEPROM
    packet.LockEprom(servo_id)

    # 6. Verify — read position again, should be ~512 now
    new_pos, result, _ = packet.ReadPos(servo_id)
    if result == COMM_SUCCESS:
        print(f"[OK] Servo {servo_id}: position now reads {new_pos} (offset {offset} saved)")
    else:
        print(f"[WARN] Servo {servo_id}: offset written but couldn't verify")

    return True


def set_center_all(packet: scscl, ears_config: dict) -> bool:
    """Set current position as center for all ear servos from config."""
    assignments = get_servo_assignments(ears_config)
    print("=== SET CENTER POSITION ===")
    print("Current physical position of each servo will become 0 degrees (512).\n")

    all_ok = True
    for servo_id, function in assignments:
        print(f"--- {function} (ID {servo_id}) ---")
        if not set_center(packet, servo_id):
            all_ok = False
        print()

    if all_ok:
        print("[OK] All servos calibrated. Current positions are now center.")
    else:
        print("[WARN] Some servos failed. Check errors above.")
    return all_ok


def main() -> None:
    parser = argparse.ArgumentParser(description="SCS0009 ear servo calibration tool")
    subparsers = parser.add_subparsers(dest="command")

    # -- ping command --
    sp_ping = subparsers.add_parser(
        "ping", help="Ping a servo ID to check if it responds"
    )
    sp_ping.add_argument("servo_id", type=int, help="Servo ID to ping (1-253)")

    # -- set-center command --
    subparsers.add_parser(
        "set-center", help="Set current position as center (0 deg) for all ear servos"
    )

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

    ears_config = load_config()
    port, packet = open_port(ears_config)

    try:
        if args.command == "ping":
            _, result, _ = packet.ping(args.servo_id)
            if result == COMM_SUCCESS:
                print(f"[OK] Servo ID {args.servo_id} responded")
            else:
                print(f"[FAIL] No response from ID {args.servo_id}")
            sys.exit(0 if result == COMM_SUCCESS else 1)
        elif args.command == "set-center":
            ok = set_center_all(packet, ears_config)
            sys.exit(0 if ok else 1)
        elif args.command == "assign-all":
            ok = assign_all(packet, ears_config)
            sys.exit(0 if ok else 1)
        elif args.command == "set-id":
            ok = set_servo_id(packet, args.current_id, args.new_id)
            sys.exit(0 if ok else 1)
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
