"""Ears servo driver for Feetech SCS0009 servos via Waveshare adapter.

Reads limits, center positions, and speed from config/servo-ids.yaml.
All angles are in degrees relative to center (0 = straight up).
"""

import os

import yaml
from scservo_sdk import PortHandler, COMM_SUCCESS
from scservo_sdk.scscl import (
    scscl,
    SCSCL_PRESENT_LOAD_L,
    SCSCL_PRESENT_VOLTAGE,
    SCSCL_PRESENT_TEMPERATURE,
)

# -- Config path resolution --
# Try relative to source tree first, then common deploy locations
_CANDIDATES = [
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "..", "config", "servo-ids.yaml"),
    os.path.expanduser("~/olaf/config/servo-ids.yaml"),
]
CONFIG_PATH = next(
    (os.path.normpath(p) for p in _CANDIDATES if os.path.isfile(p)),
    os.path.normpath(_CANDIDATES[0]),  # Fallback for error message
)


class EarsServoDriver:
    """Controls 4x SCS0009 ear servos with angle limits and speed control.

    Angles are in degrees from center (0). Positive = outward for pan,
    forward for tilt. Values beyond limits are clamped automatically.
    """

    def __init__(self, config_path: str = CONFIG_PATH):
        config_path = os.path.normpath(config_path)
        with open(config_path) as f:
            full_config = yaml.safe_load(f)
        self._config = full_config["ears"]

        self._counts_per_deg = self._config["counts_per_degree"]
        self._base_speed = self._config["base_speed"]
        self._servos = self._config["servos"]

        # Open port
        device = self._config["port"]
        baudrate = self._config["baudrate"]
        self._port = PortHandler(device)
        if not self._port.openPort():
            raise ConnectionError(f"Failed to open port {device}")
        if not self._port.setBaudRate(baudrate):
            raise ConnectionError(f"Failed to set baud rate {baudrate}")
        self._packet = scscl(self._port)

    def close(self):
        """Close serial connection."""
        self._port.closePort()

    def _get_servo(self, name: str) -> dict:
        """Get servo config by name (left_pan, left_tilt, right_pan, right_tilt)."""
        if name not in self._servos:
            raise ValueError(f"Unknown servo: {name}. Valid: {list(self._servos.keys())}")
        return self._servos[name]

    def _degrees_to_position(self, servo: dict, degrees: float) -> int:
        """Convert degrees to raw servo position, respecting direction.

        Args:
            servo: Servo config dict with center_position, direction, limits.
            degrees: Angle in degrees from center.

        Returns:
            Raw position (0-1023), clamped to physical limits.
        """
        center = servo["center_position"]
        direction = servo["direction"]
        position = center + int(degrees * direction * self._counts_per_deg)
        return max(0, min(1023, position))

    def _clamp_angle(self, servo: dict, degrees: float) -> float:
        """Clamp angle to configured min/max limits."""
        min_a = servo["min_angle"]
        max_a = servo["max_angle"]
        clamped = max(min_a, min(max_a, degrees))
        if clamped != degrees:
            print(f"[WARN] Angle {degrees} clamped to [{min_a}, {max_a}] -> {clamped}")
        return clamped

    def move(self, servo_name: str, degrees: float, speed_pct: float = 0.0) -> bool:
        """Move a servo to the given angle in degrees from center.

        Args:
            servo_name: One of left_pan, left_tilt, right_pan, right_tilt.
            degrees: Target angle (0 = center). Clamped to configured limits.
            speed_pct: Speed adjustment as fraction (-0.3 to +0.3).
                       0.0 = base speed, +0.3 = 30% faster, -0.3 = 30% slower.

        Returns:
            True if command was sent successfully.
        """
        servo = self._get_servo(servo_name)
        degrees = self._clamp_angle(servo, degrees)
        position = self._degrees_to_position(servo, degrees)

        # Adjust speed: base_speed ± 30%
        speed_pct = max(-0.3, min(0.3, speed_pct))
        speed = int(self._base_speed * (1.0 + speed_pct))

        servo_id = servo["id"]
        result, error = self._packet.WritePos(servo_id, position, 0, speed)
        if result != COMM_SUCCESS:
            print(f"[ERROR] Move failed for {servo_name} (ID {servo_id}): "
                  f"{self._packet.getTxRxResult(result)}")
            return False
        return True

    def move_left_pan(self, degrees: float, speed_pct: float = 0.0) -> bool:
        """Move left ear pan. 0=center, +90=max outward."""
        return self.move("left_pan", degrees, speed_pct)

    def move_left_tilt(self, degrees: float, speed_pct: float = 0.0) -> bool:
        """Move left ear tilt. 0=center, -90=back, +110=forward."""
        return self.move("left_tilt", degrees, speed_pct)

    def move_right_pan(self, degrees: float, speed_pct: float = 0.0) -> bool:
        """Move right ear pan. 0=center, +90=max outward."""
        return self.move("right_pan", degrees, speed_pct)

    def move_right_tilt(self, degrees: float, speed_pct: float = 0.0) -> bool:
        """Move right ear tilt. 0=center, -90=back, +110=forward."""
        return self.move("right_tilt", degrees, speed_pct)

    def center_all(self) -> bool:
        """Move all servos to center (0 degrees)."""
        ok = True
        for name in self._servos:
            if not self.move(name, 0.0):
                ok = False
        return ok

    def read_position(self, servo_name: str) -> float | None:
        """Read current position of a servo in degrees from center.

        Returns:
            Angle in degrees, or None on read failure.
        """
        servo = self._get_servo(servo_name)
        servo_id = servo["id"]
        pos, result, _ = self._packet.ReadPos(servo_id)
        if result != COMM_SUCCESS:
            return None
        center = servo["center_position"]
        direction = servo["direction"]
        return (pos - center) / (direction * self._counts_per_deg)

    def read_load(self, servo_name: str) -> int | None:
        """Read current load/torque from a servo.

        Returns:
            Load value (0-1023, 10-bit), or None on read failure.
            Bit 10 indicates direction (0=CW, 1=CCW), bits 0-9 are magnitude.
        """
        servo = self._get_servo(servo_name)
        load, result, _ = self._packet.read2ByteTxRx(servo["id"], SCSCL_PRESENT_LOAD_L)
        if result != COMM_SUCCESS:
            return None
        return load

    def read_voltage(self, servo_name: str) -> float | None:
        """Read current supply voltage from a servo.

        Returns:
            Voltage in volts (raw value / 10), or None on read failure.
        """
        servo = self._get_servo(servo_name)
        voltage, result, _ = self._packet.read1ByteTxRx(servo["id"], SCSCL_PRESENT_VOLTAGE)
        if result != COMM_SUCCESS:
            return None
        return voltage / 10.0

    def read_temperature(self, servo_name: str) -> int | None:
        """Read current temperature from a servo.

        Returns:
            Temperature in degrees Celsius, or None on read failure.
        """
        servo = self._get_servo(servo_name)
        temp, result, _ = self._packet.read1ByteTxRx(servo["id"], SCSCL_PRESENT_TEMPERATURE)
        if result != COMM_SUCCESS:
            return None
        return temp

    def read_all_feedback(self, servo_name: str) -> dict | None:
        """Read all feedback from a servo: position, load, voltage, temperature.

        Returns:
            Dict with keys: position_deg, load, voltage, temperature. None on failure.
        """
        return {
            "position_deg": self.read_position(servo_name),
            "load": self.read_load(servo_name),
            "voltage": self.read_voltage(servo_name),
            "temperature": self.read_temperature(servo_name),
        }
