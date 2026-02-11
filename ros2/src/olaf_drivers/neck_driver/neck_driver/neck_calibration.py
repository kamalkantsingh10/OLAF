#!/usr/bin/env python3
"""Calibration tool for Feetech STS3215 neck servos via Waveshare adapter.

Usage:
    python neck_calibration.py ping <servo_id>       # Ping a servo
    python neck_calibration.py assign-all             # Assign IDs 1-3 interactively
    python neck_calibration.py set-id <cur> <new>     # Change a single servo ID
    python neck_calibration.py set-center             # Save current positions as center
    python neck_calibration.py go-center              # Move all to saved center positions
"""

import argparse
import os
import sys

import yaml
from scservo_sdk import PortHandler, COMM_SUCCESS
from scservo_sdk.sms_sts import sms_sts, SMS_STS_ID

# -- Config path resolution --
_CANDIDATES = [
    os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "..",
        "config", "servo-ids.yaml",
    ),
    os.path.expanduser("~/olaf/config/servo-ids.yaml"),
]
CONFIG_PATH = next(
    (os.path.normpath(p) for p in _CANDIDATES if os.path.isfile(p)),
    os.path.normpath(_CANDIDATES[0]),
)
FACTORY_DEFAULT_ID = 1


def load_config() -> dict:
    """Load neck config from config/servo-ids.yaml."""
    path = os.path.normpath(CONFIG_PATH)
    with open(path) as f:
        config = yaml.safe_load(f)
    return config["neck"]


def get_servo_assignments(neck_config: dict) -> list[tuple[int, str]]:
    """Build assignment list from config."""
    assignments = []
    for _name, info in neck_config["servos"].items():
        assignments.append((info["id"], info["function"]))
    return assignments


def open_port(neck_config: dict) -> tuple[PortHandler, sms_sts]:
    """Open serial port and return (port_handler, packet_handler)."""
    device = neck_config["port"]
    baudrate = neck_config["baudrate"]
    port = PortHandler(device)
    if not port.openPort():
        print(f"[ERROR] Failed to open port: {device}")
        sys.exit(1)
    if not port.setBaudRate(baudrate):
        print(f"[ERROR] Failed to set baud rate: {baudrate}")
        sys.exit(1)
    packet = sms_sts(port)
    print(f"[INFO] Port opened: {device} @ {baudrate} bps")
    return port, packet


def set_servo_id(packet: sms_sts, current_id: int, new_id: int) -> bool:
    """Change a servo's ID.

    WARNING: Connect only ONE servo at a time when using this function.
    Factory default ID is 1 -- multiple servos on the bus with the same ID
    will cause bus conflicts.

    Args:
        packet: sms_sts packet handler (already connected).
        current_id: The servo's current ID (1-253).
        new_id: The desired new ID (1-253).

    Returns:
        True if ID was changed successfully.
    """
    # 1. Ping servo at current_id
    _, result, _ = packet.ping(current_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] No servo found at ID {current_id}: "
              f"{packet.getTxRxResult(result)}")
        return False
    print(f"[INFO] Servo found at ID {current_id}")

    # 2. Unlock EEPROM
    result, error = packet.unLockEprom(current_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Failed to unlock EEPROM: "
              f"{packet.getTxRxResult(result)}")
        return False
    if error != 0:
        print(f"[ERROR] Servo error on unlock: "
              f"{packet.getRxPacketError(error)}")
        return False

    # 3. Write new ID
    result, error = packet.write1ByteTxRx(current_id, SMS_STS_ID, new_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Failed to write new ID: "
              f"{packet.getTxRxResult(result)}")
        return False
    if error != 0:
        print(f"[ERROR] Servo error on ID write: "
              f"{packet.getRxPacketError(error)}")
        return False

    # 4. Lock EEPROM (address with new_id now)
    packet.LockEprom(new_id)

    # 5. Verify by pinging new ID
    _, result, _ = packet.ping(new_id)
    if result != COMM_SUCCESS:
        print(f"[ERROR] Verification failed -- no response at new ID {new_id}")
        return False

    print(f"[OK] Servo ID changed: {current_id} -> {new_id}")
    return True


def assign_all(packet: sms_sts, neck_config: dict) -> bool:
    """Assign neck servo IDs from config, adding one at a time.

    Each new servo arrives with factory default ID 1. After reassignment
    it no longer conflicts, so keep adding the next servo to the chain.
    """
    assignments = get_servo_assignments(neck_config)

    print("=== NECK SERVO ID ASSIGNMENT ===\n")
    print("Assignments (from config/servo-ids.yaml):")
    for target_id, function in assignments:
        print(f"  ID {target_id}: {function}")
    print()

    total = len(assignments)
    for i, (target_id, function) in enumerate(assignments, 1):
        input(
            f"[{i}/{total}] Connect servo for '{function}' "
            f"(will become ID {target_id}), then press Enter..."
        )

        ok = set_servo_id(packet, FACTORY_DEFAULT_ID, target_id)
        if not ok:
            print(f"[ERROR] Failed to assign ID {target_id}. Fix and retry.")
            return False

        print()

    ids = [str(a[0]) for a in assignments]
    print(f"=== ALL {total} SERVOS ASSIGNED ===")
    print(f"All servos on the chain with IDs {', '.join(ids)}.")
    return True


def set_center_all(packet: sms_sts, neck_config: dict) -> bool:
    """Read current positions and save as center_position in config.

    Position neck physically at desired center before running this.
    The driver uses these values as the zero reference for degree-based movement.
    """
    print("=== SET CENTER POSITION ===")
    print("Reading current positions and saving to config/servo-ids.yaml\n")

    config_path = os.path.normpath(CONFIG_PATH)
    with open(config_path) as f:
        full_config = yaml.safe_load(f)

    all_ok = True
    for name, info in neck_config["servos"].items():
        servo_id = info["id"]
        function = info["function"]

        pos, result, _ = packet.ReadPos(servo_id)
        if result != COMM_SUCCESS:
            print(f"[ERROR] Failed to read ID {servo_id}: "
                  f"{packet.getTxRxResult(result)}")
            all_ok = False
            continue

        full_config["neck"]["servos"][name]["center_position"] = pos
        print(f"[OK] ID {servo_id} ({function}): center_position = {pos}")

    if all_ok:
        with open(config_path, "w") as f:
            yaml.dump(full_config, f, default_flow_style=False,
                      sort_keys=False)
        print(f"\n[OK] Saved to {config_path}")
    else:
        print("\n[WARN] Some servos failed. Config not saved.")

    return all_ok


def go_center(packet: sms_sts, neck_config: dict) -> bool:
    """Move all neck servos to their saved center positions."""
    print("=== GO TO CENTER ===\n")

    DEFAULT_SPEED = 500   # Steps/sec — moderate speed for safety
    DEFAULT_ACC = 50      # Smooth acceleration ramp

    all_ok = True
    for _name, info in neck_config["servos"].items():
        servo_id = info["id"]
        function = info["function"]
        center = info.get("center_position")

        if center is None:
            print(f"[ERROR] No center_position for ID {servo_id}. "
                  "Run set-center first.")
            all_ok = False
            continue

        # WritePosEx(id, position, speed, acceleration)
        result, error = packet.WritePosEx(
            servo_id, center, DEFAULT_SPEED, DEFAULT_ACC
        )
        if result != COMM_SUCCESS:
            print(f"[ERROR] Failed to move ID {servo_id}: "
                  f"{packet.getTxRxResult(result)}")
            all_ok = False
        else:
            print(f"[OK] ID {servo_id} ({function}) -> position {center}")

    return all_ok


def main() -> None:
    parser = argparse.ArgumentParser(
        description="STS3215 neck servo calibration tool"
    )
    subparsers = parser.add_subparsers(dest="command")

    # -- ping --
    sp_ping = subparsers.add_parser(
        "ping", help="Ping a servo ID to check if it responds"
    )
    sp_ping.add_argument(
        "servo_id", type=int, help="Servo ID to ping (1-253)"
    )

    # -- set-center --
    subparsers.add_parser(
        "set-center",
        help="Set current position as center (0 deg) for all neck servos",
    )

    # -- go-center --
    subparsers.add_parser(
        "go-center", help="Move all neck servos to saved center positions"
    )

    # -- assign-all --
    subparsers.add_parser(
        "assign-all", help="Assign IDs 1-3 to neck servos interactively"
    )

    # -- set-id --
    sp_id = subparsers.add_parser(
        "set-id", help="Change a single servo's ID"
    )
    sp_id.add_argument(
        "current_id", type=int, help="Current servo ID (default factory: 1)"
    )
    sp_id.add_argument(
        "new_id", type=int, help="Desired new ID (1-253)"
    )

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(0)

    neck_config = load_config()
    port, packet = open_port(neck_config)

    try:
        if args.command == "ping":
            _, result, _ = packet.ping(args.servo_id)
            if result == COMM_SUCCESS:
                print(f"[OK] Servo ID {args.servo_id} responded")
            else:
                print(f"[FAIL] No response from ID {args.servo_id}")
            sys.exit(0 if result == COMM_SUCCESS else 1)
        elif args.command == "set-center":
            ok = set_center_all(packet, neck_config)
            sys.exit(0 if ok else 1)
        elif args.command == "go-center":
            ok = go_center(packet, neck_config)
            sys.exit(0 if ok else 1)
        elif args.command == "assign-all":
            ok = assign_all(packet, neck_config)
            sys.exit(0 if ok else 1)
        elif args.command == "set-id":
            ok = set_servo_id(packet, args.current_id, args.new_id)
            sys.exit(0 if ok else 1)
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
