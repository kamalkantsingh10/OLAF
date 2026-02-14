"""Neck servo driver for Feetech STS3215 servos via Waveshare adapter.

Reads limits, center positions, and speed from config/servo-ids.yaml.
All angles are in degrees relative to center (0 = head level/forward).

Parallel linkage mechanics:
    - Pan servo: direct rotation (left/right)
    - Tilt: left_linkage and right_linkage move in opposite directions
    - Roll: left_linkage and right_linkage move in same direction
"""

import os

import yaml
from scservo_sdk import PortHandler, COMM_SUCCESS
from scservo_sdk.sms_sts import (
    sms_sts,
    SMS_STS_PRESENT_LOAD_L,
    SMS_STS_PRESENT_VOLTAGE,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_TORQUE_ENABLE,
)

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


class NeckServoDriver:
    """Controls 3x STS3215 neck servos with parallel linkage tilt/roll.

    Pan angle: positive = OLAF looks left, negative = right.
    Tilt angle: positive = chin up, negative = chin down.
    Roll angle: positive = head tilts to OLAF's right, negative = left.
    """

    def __init__(self, config_path: str = CONFIG_PATH):
        config_path = os.path.normpath(config_path)
        with open(config_path) as f:
            full_config = yaml.safe_load(f)
        self._config = full_config["neck"]

        self._counts_per_deg = self._config["counts_per_degree"]
        self._base_speed = self._config["base_speed"]
        self._base_acc = self._config["base_acc"]
        self._servos = self._config["servos"]

        self._port = PortHandler(self._config["port"])
        if not self._port.openPort():
            raise ConnectionError(f"Failed to open port {self._config['port']}")
        if not self._port.setBaudRate(self._config["baudrate"]):
            raise ConnectionError(f"Failed to set baud rate {self._config['baudrate']}")
        self._packet = sms_sts(self._port)

    def close(self):
        """Close serial connection."""
        self._port.closePort()

    def _get_servo(self, name: str) -> dict:
        """Get servo config by name."""
        if name not in self._servos:
            raise ValueError(f"Unknown servo: {name}. Valid: {list(self._servos.keys())}")
        return self._servos[name]

    def _degrees_to_position(self, servo: dict, degrees: float) -> int:
        """Convert degrees to raw servo position, respecting direction."""
        center = servo["center_position"]
        direction = servo["direction"]
        position = center + int(degrees * direction * self._counts_per_deg)
        return max(0, min(4095, position))

    def _clamp_angle(self, servo: dict, degrees: float) -> float:
        """Clamp angle to configured min/max limits."""
        min_a = servo["min_angle"]
        max_a = servo["max_angle"]
        return max(min_a, min(max_a, degrees))

    def _clear_overload(self, servo_id: int) -> None:
        """Toggle torque to clear latched overload protection."""
        self._packet.write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, 0)
        self._packet.write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, 1)

    def _write_pos(self, servo_id: int, position: int, speed: int, acc: int) -> bool:
        """Write position to a servo with overload auto-recovery.

        If the servo has a latched overload error, toggles torque to clear
        it and retries the command.
        """
        result, error = self._packet.WritePosEx(servo_id, position, speed, acc)
        if result != COMM_SUCCESS:
            return False
        if error & 0x20:  # Overload bit
            self._clear_overload(servo_id)
            self._packet.WritePosEx(servo_id, position, speed, acc)
        return True

    def _move_servo(self, name: str, degrees: float, speed: int | None = None,
                    acc: int | None = None) -> bool:
        """Move a single servo to the given angle.

        Args:
            name: Servo name from config.
            degrees: Target angle (0 = center). Clamped to limits.
            speed: Steps/sec (None = base_speed from config).
            acc: Acceleration (None = base_acc from config).

        Returns:
            True if command was sent successfully.
        """
        servo = self._get_servo(name)
        degrees = self._clamp_angle(servo, degrees)
        position = self._degrees_to_position(servo, degrees)
        speed = speed or self._base_speed
        acc = acc or self._base_acc
        return self._write_pos(servo["id"], position, speed, acc)

    def move_pan(self, degrees: float, speed: int | None = None) -> bool:
        """Pan head left/right.

        Args:
            degrees: Target angle. Positive = OLAF looks left, negative = right.
            speed: Steps/sec (None = default).
        """
        return self._move_servo("pan", degrees, speed)

    def move_tilt(self, degrees: float, speed: int | None = None) -> bool:
        """Tilt head up/down via parallel linkage.

        Both linkage servos move in opposite physical directions.

        Args:
            degrees: Target angle. Positive = chin up, negative = chin down.
            speed: Steps/sec (None = default).
        """
        return self.move_pose(tilt=degrees, roll=0.0, speed=speed)

    def move_roll(self, degrees: float, speed: int | None = None) -> bool:
        """Roll head left/right via parallel linkage.

        Both linkage servos move in the same physical direction to produce roll.
        Left gets +degrees, right gets -degrees (direction config inverts).

        Args:
            degrees: Target angle. Positive = OLAF tilts head to his right,
                     negative = tilts to his left.
            speed: Steps/sec (None = default).
        """
        return self.move_pose(tilt=0.0, roll=degrees, speed=speed)

    def move_pose(self, pan: float = 0.0, tilt: float = 0.0, roll: float = 0.0,
                  speed: int | None = None) -> bool:
        """Set full neck pose (pan + tilt + roll combined).

        Tilt and roll are combined on the linkage servos:
            left_linkage  = tilt + roll
            right_linkage = tilt - roll  (tilt uses opposite dirs, roll uses same)

        Args:
            pan: Degrees. Positive = OLAF looks left.
            tilt: Degrees. Positive = chin up.
            roll: Degrees. Positive = OLAF tilts head to his right.
            speed: Steps/sec (None = default).
        """
        left = self._get_servo("left_linkage")
        right = self._get_servo("right_linkage")

        # Combine tilt + roll for each linkage
        # Left linkage: direction=1, +counts = chin up (tilt+) and roll+
        left_deg = tilt + roll
        # Right linkage: direction=-1, +counts = chin down
        # For tilt up: right needs negative deg → position decreases → chin up
        # For roll: same direction as left → also +roll
        # But right direction=-1, so tilt=-tilt_deg and roll=+roll_deg in raw terms
        # Combined right_deg for the _degrees_to_position with direction=-1:
        #   tilt component: tilt (same sign as left, direction handles inversion)
        #   roll component: -roll (opposite sign, since right dir is inverted)
        right_deg = tilt - roll

        left_deg = self._clamp_angle(left, left_deg)
        right_deg = self._clamp_angle(right, right_deg)

        pos_l = self._degrees_to_position(left, left_deg)
        pos_r = self._degrees_to_position(right, right_deg)

        speed = speed or self._base_speed
        acc = self._base_acc

        ok1 = self._move_servo("pan", pan, speed)
        ok2 = self._write_pos(left["id"], pos_l, speed, acc)
        ok3 = self._write_pos(right["id"], pos_r, speed, acc)
        return ok1 and ok2 and ok3

    def center_all(self, speed: int | None = None) -> bool:
        """Move all servos to center (0 degrees)."""
        return self.move_pose(0.0, 0.0, 0.0, speed)

    def read_position(self, servo_name: str) -> float | None:
        """Read current position of a servo in degrees from center."""
        servo = self._get_servo(servo_name)
        pos, result, _ = self._packet.ReadPos(servo["id"])
        if result != COMM_SUCCESS:
            return None
        center = servo["center_position"]
        direction = servo["direction"]
        return (pos - center) / (direction * self._counts_per_deg)

    def read_load(self, servo_name: str) -> int | None:
        """Read current load from a servo (0-1023 magnitude, bit 10 = direction)."""
        servo = self._get_servo(servo_name)
        load, result, _ = self._packet.read2ByteTxRx(servo["id"], SMS_STS_PRESENT_LOAD_L)
        if result != COMM_SUCCESS:
            return None
        return load

    def read_voltage(self, servo_name: str) -> float | None:
        """Read supply voltage in volts."""
        servo = self._get_servo(servo_name)
        voltage, result, _ = self._packet.read1ByteTxRx(servo["id"], SMS_STS_PRESENT_VOLTAGE)
        if result != COMM_SUCCESS:
            return None
        return voltage / 10.0

    def read_temperature(self, servo_name: str) -> int | None:
        """Read temperature in degrees Celsius."""
        servo = self._get_servo(servo_name)
        temp, result, _ = self._packet.read1ByteTxRx(servo["id"], SMS_STS_PRESENT_TEMPERATURE)
        if result != COMM_SUCCESS:
            return None
        return temp

    def read_all_feedback(self, servo_name: str) -> dict | None:
        """Read all feedback: position, load, voltage, temperature."""
        return {
            "position_deg": self.read_position(servo_name),
            "load": self.read_load(servo_name),
            "voltage": self.read_voltage(servo_name),
            "temperature": self.read_temperature(servo_name),
        }
