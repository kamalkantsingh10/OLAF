"""I2C client for OLAF Head ESP32 module.

Wraps smbus2 to send register-based commands to the Head ESP32
at I2C address 0x08. Handles expression, blink, look direction,
and system status commands.

Register map (from Story 1.3):
    0x00: STATUS (R)           - Module status
    0x10: EXPRESSION_TYPE (W)  - 0-6 expression enum
    0x11: EXPRESSION_INTENSITY (W) - 1-5
    0x12: BLINK_TRIGGER (W)    - Write any value to blink
    0x20: LOOK_X (W)           - Signed: -100 to +100
    0x21: LOOK_Y (W)           - Signed: -100 to +100
    0x30: SYSTEM_STATUS (W)    - 0-5 status enum
"""

import logging
from smbus2 import SMBus

# I2C slave address (Head ESP32)
I2C_HEAD_ADDRESS = 0x08
I2C_BUS_NUMBER = 1

# Register addresses
REG_STATUS = 0x00
REG_EXPRESSION_TYPE = 0x10
REG_EXPRESSION_INTENSITY = 0x11
REG_BLINK_TRIGGER = 0x12
REG_LOOK_X = 0x20
REG_LOOK_Y = 0x21
REG_SYSTEM_STATUS = 0x30

# Expression type mapping (string -> register value)
EXPRESSION_MAP: dict[str, int] = {
    "neutral": 0,
    "happy": 1,
    "sad": 2,
    "surprised": 3,
    "angry": 4,
    "sleepy": 5,
    "wink": 6,
}

# System status mapping (string -> register value)
STATUS_MAP: dict[str, int] = {
    "idle": 0,
    "woke_up": 1,
    "listening": 2,
    "processing": 3,
    "speaking": 4,
    "going_idle": 5,
}

logger = logging.getLogger(__name__)


class HeadI2CClient:
    """I2C communication client for Head ESP32 module."""

    def __init__(self, bus_number: int = I2C_BUS_NUMBER,
                 address: int = I2C_HEAD_ADDRESS) -> None:
        self._address = address
        self._bus_number = bus_number
        self._bus: SMBus | None = None

    def open(self) -> None:
        """Open the I2C bus."""
        self._bus = SMBus(self._bus_number)
        logger.info("I2C bus %d opened for head module at 0x%02X",
                     self._bus_number, self._address)

    def close(self) -> None:
        """Close the I2C bus."""
        if self._bus:
            self._bus.close()
            self._bus = None
            logger.info("I2C bus closed")

    def _write_byte(self, register: int, value: int) -> bool:
        """Write a single byte to a register.

        Args:
            register: Register address (0x00-0x30).
            value: Byte value to write (0-255).

        Returns:
            True if write succeeded, False on error.
        """
        if not self._bus:
            logger.error("I2C bus not open")
            return False
        try:
            self._bus.write_byte_data(self._address, register, value & 0xFF)
            return True
        except OSError as e:
            logger.error("I2C write failed: reg=0x%02X val=0x%02X err=%s",
                         register, value, e)
            return False

    def set_expression(self, expression: str, intensity: int = 3) -> bool:
        """Set eye expression by name.

        Args:
            expression: Expression name (neutral, happy, sad, surprised,
                        angry, sleepy, wink).
            intensity: 1-5 (1=subtle, 5=extreme).

        Returns:
            True if both writes succeeded.
        """
        expr_val = EXPRESSION_MAP.get(expression.lower())
        if expr_val is None:
            logger.warning("Unknown expression: '%s'. Valid: %s",
                           expression, list(EXPRESSION_MAP.keys()))
            return False

        intensity = max(1, min(5, intensity))
        ok1 = self._write_byte(REG_EXPRESSION_TYPE, expr_val)
        ok2 = self._write_byte(REG_EXPRESSION_INTENSITY, intensity)
        logger.debug("Expression: %s (val=%d) intensity=%d", expression, expr_val, intensity)
        return ok1 and ok2

    def trigger_blink(self) -> bool:
        """Trigger a blink animation.

        Returns:
            True if write succeeded.
        """
        logger.debug("Blink triggered")
        return self._write_byte(REG_BLINK_TRIGGER, 0x01)

    def set_look_direction(self, x: int, y: int) -> bool:
        """Set eye look direction.

        Args:
            x: Horizontal direction, -100 (left) to +100 (right).
            y: Vertical direction, -100 (down) to +100 (up).

        Returns:
            True if both writes succeeded.
        """
        x = max(-100, min(100, x))
        y = max(-100, min(100, y))
        # Convert signed to unsigned byte for I2C
        x_byte = x & 0xFF
        y_byte = y & 0xFF
        ok1 = self._write_byte(REG_LOOK_X, x_byte)
        ok2 = self._write_byte(REG_LOOK_Y, y_byte)
        logger.debug("Look: x=%d y=%d", x, y)
        return ok1 and ok2

    def set_system_status(self, status: str) -> bool:
        """Set system status (controls wake/sleep and LED animations).

        Args:
            status: Status name (idle, woke_up, listening, processing,
                    speaking, going_idle).

        Returns:
            True if write succeeded.
        """
        status_val = STATUS_MAP.get(status.lower())
        if status_val is None:
            logger.warning("Unknown status: '%s'. Valid: %s",
                           status, list(STATUS_MAP.keys()))
            return False

        logger.debug("System status: %s (val=%d)", status, status_val)
        return self._write_byte(REG_SYSTEM_STATUS, status_val)

    def read_status(self) -> int | None:
        """Read module status register.

        Returns:
            Status byte or None on error.
        """
        if not self._bus:
            logger.error("I2C bus not open")
            return None
        try:
            return self._bus.read_byte_data(self._address, REG_STATUS)
        except OSError as e:
            logger.error("I2C read failed: reg=0x%02X err=%s", REG_STATUS, e)
            return None
