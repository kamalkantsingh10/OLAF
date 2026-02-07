"""Ears servo driver for Feetech SCS0009 servos via Waveshare adapter."""

from scservo_sdk import PortHandler, PacketHandler


class EarsServoDriver:
    """Controls 4× SCS0009 ear servos (2-DOF per ear: pan + tilt).

    Servo IDs:
        1: Left ear pan
        2: Left ear tilt
        3: Right ear pan
        4: Right ear tilt

    At 0° both ears point straight up, parallel to eyeline.
    """

    # Servo IDs (to be confirmed by testing)
    LEFT_PAN = 4
    LEFT_TILT = 5
    RIGHT_PAN = 6
    RIGHT_TILT = 7

    # SCS0009 register addresses
    ADDR_GOAL_POSITION = 42  # 0x2A
    ADDR_PRESENT_POSITION = 56  # 0x38

    # Position constants
    CENTER_POSITION = 512  # 0° - straight up
    COUNTS_PER_DEGREE = 3.41  # ~1024 counts / 300° range

    def __init__(self, port: str = "/dev/waveshare_ears", baudrate: int = 1_000_000):
        """Initialize connection to Waveshare adapter.

        Args:
            port: Serial port path
            baudrate: Baud rate (default 1Mbps for SCS series)
        """
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(0)  # Protocol version 0 for SCS

        if not self.port_handler.openPort():
            raise ConnectionError(f"Failed to open port {port}")

        if not self.port_handler.setBaudRate(baudrate):
            raise ConnectionError(f"Failed to set baudrate {baudrate}")

        print(f"Connected to ears on {port} @ {baudrate} baud")

    def close(self):
        """Close serial connection."""
        self.port_handler.closePort()

    def _degrees_to_position(self, degrees: float) -> int:
        """Convert degrees to servo position.

        Args:
            degrees: Angle from center (-150 to +150)

        Returns:
            Servo position (0-1023)
        """
        position = int(self.CENTER_POSITION + degrees * self.COUNTS_PER_DEGREE)
        return max(0, min(1023, position))

    def _position_to_degrees(self, position: int) -> float:
        """Convert servo position to degrees."""
        return (position - self.CENTER_POSITION) / self.COUNTS_PER_DEGREE

    def move_servo(self, servo_id: int, degrees: float) -> bool:
        """Move a single servo to specified angle.

        Args:
            servo_id: Servo ID (1-4)
            degrees: Target angle (0 = straight up)

        Returns:
            True if successful
        """
        position = self._degrees_to_position(degrees)
        result, _ = self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, self.ADDR_GOAL_POSITION, position
        )
        if result != 0:
            print(f"Servo {servo_id} move failed: {self.packet_handler.getTxRxResult(result)}")
            return False
        return True

    def move_left_ear(self, pan: float, tilt: float) -> bool:
        """Move left ear.

        Args:
            pan: Pan angle in degrees (0 = forward)
            tilt: Tilt angle in degrees (0 = upright)
        """
        success = self.move_servo(self.LEFT_PAN, pan)
        success &= self.move_servo(self.LEFT_TILT, tilt)
        return success

    def move_right_ear(self, pan: float, tilt: float) -> bool:
        """Move right ear.

        Args:
            pan: Pan angle in degrees (0 = forward)
            tilt: Tilt angle in degrees (0 = upright)
        """
        success = self.move_servo(self.RIGHT_PAN, pan)
        success &= self.move_servo(self.RIGHT_TILT, tilt)
        return success

    def move_both_ears(self, left_pan: float, left_tilt: float,
                       right_pan: float, right_tilt: float) -> bool:
        """Move both ears simultaneously.

        Args:
            left_pan: Left ear pan angle
            left_tilt: Left ear tilt angle
            right_pan: Right ear pan angle
            right_tilt: Right ear tilt angle
        """
        success = self.move_left_ear(left_pan, left_tilt)
        success &= self.move_right_ear(right_pan, right_tilt)
        return success

    def center_all(self) -> bool:
        """Move all ears to center position (512)."""
        success = True
        for servo_id in [self.LEFT_PAN, self.LEFT_TILT, self.RIGHT_PAN, self.RIGHT_TILT]:
            result, _ = self.packet_handler.write2ByteTxRx(
                self.port_handler, servo_id, self.ADDR_GOAL_POSITION, 512
            )
            if result != 0:
                success = False
        return success

    def go_to_position(self, servo_id: int, position: int) -> bool:
        """Move servo to raw position (0-1023)."""
        result, _ = self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, self.ADDR_GOAL_POSITION, position
        )
        return result == 0

    def read_position(self, servo_id: int) -> float | None:
        """Read current position of a servo in degrees."""
        position, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, self.ADDR_PRESENT_POSITION
        )
        if result != 0:
            print(f"Read servo {servo_id} failed: {self.packet_handler.getTxRxResult(result)}")
            return None
        return self._position_to_degrees(position)

    def ping(self, servo_id: int) -> bool:
        """Ping a servo to check if it's connected."""
        _, result, _ = self.packet_handler.ping(self.port_handler, servo_id)
        return result == 0

    def set_as_center(self, servo_id: int) -> bool:
        """Set the current physical position as the center (0 degrees).

        This writes an offset to the servo so current position = 512 (center).

        Args:
            servo_id: Servo ID

        Returns:
            True if successful
        """
        ADDR_OFFSET = 31  # Offset register for SCS series

        # Read current position
        current_pos, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, servo_id, self.ADDR_PRESENT_POSITION
        )
        if result != 0:
            print(f"Failed to read position: {self.packet_handler.getTxRxResult(result)}")
            return False

        # Calculate offset needed to make current position = 512
        offset = self.CENTER_POSITION - current_pos

        # Unlock EEPROM
        self._unlock_eeprom(servo_id)

        # Offset is stored as signed 16-bit
        result, _ = self.packet_handler.write2ByteTxRx(
            self.port_handler, servo_id, ADDR_OFFSET, offset & 0xFFFF
        )
        if result != 0:
            print(f"Failed to set offset: {self.packet_handler.getTxRxResult(result)}")
            return False

        # Lock EEPROM to save
        self._lock_eeprom(servo_id)

        print(f"Servo {servo_id}: position {current_pos} -> set as center (offset {offset}, saved)")
        return True

    def set_all_as_center(self) -> bool:
        """Set current position of all 4 ear servos as center."""
        success = True
        for servo_id in [self.LEFT_PAN, self.LEFT_TILT, self.RIGHT_PAN, self.RIGHT_TILT]:
            if not self.set_as_center(servo_id):
                success = False
        return success

    def _unlock_eeprom(self, servo_id: int) -> bool:
        """Unlock EEPROM for writing."""
        ADDR_LOCK = 55  # Lock register (0 = unlocked, 1 = locked)
        result, _ = self.packet_handler.write1ByteTxRx(
            self.port_handler, servo_id, ADDR_LOCK, 0
        )
        return result == 0

    def _lock_eeprom(self, servo_id: int) -> bool:
        """Lock EEPROM after writing."""
        ADDR_LOCK = 55
        result, _ = self.packet_handler.write1ByteTxRx(
            self.port_handler, servo_id, ADDR_LOCK, 1
        )
        return result == 0

    def set_servo_id(self, current_id: int, new_id: int) -> bool:
        """Change a servo's ID. Only connect ONE servo at a time!

        Args:
            current_id: Current servo ID
            new_id: New servo ID (1-253)

        Returns:
            True if successful
        """
        ADDR_ID = 5  # ID register address for SCS series

        # Unlock EEPROM first
        if not self._unlock_eeprom(current_id):
            print("Warning: Failed to unlock EEPROM")

        # Write new ID
        result, _ = self.packet_handler.write1ByteTxRx(
            self.port_handler, current_id, ADDR_ID, new_id
        )
        if result != 0:
            print(f"Failed to change ID: {self.packet_handler.getTxRxResult(result)}")
            return False

        # Lock EEPROM to save
        if not self._lock_eeprom(new_id):
            print("Warning: Failed to lock EEPROM")

        print(f"Changed servo ID from {current_id} to {new_id} (saved to EEPROM)")
        return True

    def scan(self, start_id: int = 1, end_id: int = 20) -> list[int]:
        """Scan for connected servos in ID range.

        Args:
            start_id: First ID to check
            end_id: Last ID to check

        Returns:
            List of responding servo IDs
        """
        found = []
        print(f"Scanning servo IDs {start_id}-{end_id}...")
        for sid in range(start_id, end_id + 1):
            if self.ping(sid):
                found.append(sid)
                print(f"  Found servo ID {sid}")
        print(f"Scan complete. Found {len(found)} servos: {found}")
        return found


if __name__ == "__main__":
    import sys

    driver = EarsServoDriver()

    if len(sys.argv) > 1 and sys.argv[1] == "assign":
        # ID assignment mode - connect ONE servo at a time!
        print("=== SERVO ID ASSIGNMENT MODE ===")
        print("Connect only ONE servo at a time!\n")

        for new_id in [4, 5, 6, 7]:
            input(f"Connect next servo, position at CENTER, press Enter (will become ID {new_id})...")

            # Always write to ID 1 (factory default)
            print(f"Setting ID 1 -> {new_id}...")
            driver.set_servo_id(1, new_id)

            # Set current position as center
            driver.set_as_center(new_id)
            print(f"Servo {new_id} done. Add next servo to the chain.\n")

        print("Done! Now reconnect all 4 servos and run without 'assign'.")

    else:
        # Normal test mode
        found_servos = driver.scan(start_id=1, end_id=15)

        if not found_servos:
            print("No servos found! Check wiring and power.")
            driver.close()
            exit(1)

        if len(found_servos) < 4:
            print(f"\nWARNING: Expected 4 servos, found {len(found_servos)}")
            print("Run with 'assign' argument to set IDs: python ears_servo_driver.py assign\n")

        print("\nTesting each found servo individually...")
        print("Using raw positions: 512=center, +50 counts = small move\n")

        for sid in found_servos:
            input(f"Press Enter to move servo {sid} to position 562 (+50 from center)...")
            driver.go_to_position(sid, 562)
            input(f"Press Enter to return servo {sid} to center (512)...")
            driver.go_to_position(sid, 512)

        print("\nCentering all servos...")
        driver.center_all()
        print("Done. Update the ID mapping based on what you observed.")

    driver.close()
