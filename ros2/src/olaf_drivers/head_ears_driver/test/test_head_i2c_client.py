"""Unit tests for HeadI2CClient.

Tests expression mapping, parameter validation, and I2C write calls
using a mocked SMBus.
"""

from unittest.mock import MagicMock, patch
import pytest

from head_ears_driver.head_i2c_client import (
    HeadI2CClient,
    EXPRESSION_MAP,
    STATUS_MAP,
    REG_EXPRESSION_TYPE,
    REG_EXPRESSION_INTENSITY,
    REG_BLINK_TRIGGER,
    REG_LOOK_X,
    REG_LOOK_Y,
    REG_SYSTEM_STATUS,
)


@pytest.fixture
def client():
    """Create a HeadI2CClient with a mocked SMBus."""
    with patch("head_ears_driver.head_i2c_client.SMBus") as mock_smbus_cls:
        mock_bus = MagicMock()
        mock_smbus_cls.return_value = mock_bus
        c = HeadI2CClient()
        c.open()
        yield c, mock_bus


class TestExpressionMapping:
    """Tests for set_expression()."""

    @pytest.mark.parametrize("name,expected_val", list(EXPRESSION_MAP.items()))
    def test_valid_expressions(self, client, name, expected_val):
        c, bus = client
        assert c.set_expression(name) is True
        bus.write_byte_data.assert_any_call(0x08, REG_EXPRESSION_TYPE, expected_val)

    def test_invalid_expression_returns_false(self, client):
        c, bus = client
        assert c.set_expression("unknown_emotion") is False
        bus.write_byte_data.assert_not_called()

    def test_case_insensitive(self, client):
        c, bus = client
        assert c.set_expression("HAPPY") is True
        bus.write_byte_data.assert_any_call(0x08, REG_EXPRESSION_TYPE, 1)

    def test_intensity_clamped_to_range(self, client):
        c, bus = client
        c.set_expression("neutral", intensity=10)
        bus.write_byte_data.assert_any_call(0x08, REG_EXPRESSION_INTENSITY, 5)

        bus.reset_mock()
        c.set_expression("neutral", intensity=0)
        bus.write_byte_data.assert_any_call(0x08, REG_EXPRESSION_INTENSITY, 1)


class TestBlink:
    """Tests for trigger_blink()."""

    def test_blink_writes_to_register(self, client):
        c, bus = client
        assert c.trigger_blink() is True
        bus.write_byte_data.assert_called_once_with(0x08, REG_BLINK_TRIGGER, 0x01)


class TestLookDirection:
    """Tests for set_look_direction()."""

    def test_look_center(self, client):
        c, bus = client
        assert c.set_look_direction(0, 0) is True
        bus.write_byte_data.assert_any_call(0x08, REG_LOOK_X, 0)
        bus.write_byte_data.assert_any_call(0x08, REG_LOOK_Y, 0)

    def test_look_clamped(self, client):
        c, bus = client
        c.set_look_direction(200, -200)
        # 100 clamped, then & 0xFF
        bus.write_byte_data.assert_any_call(0x08, REG_LOOK_X, 100)
        # -100 as unsigned byte: (-100) & 0xFF = 156 = 0x9C
        bus.write_byte_data.assert_any_call(0x08, REG_LOOK_Y, 0x9C)

    def test_negative_values_as_unsigned(self, client):
        c, bus = client
        c.set_look_direction(-50, -25)
        bus.write_byte_data.assert_any_call(0x08, REG_LOOK_X, (-50) & 0xFF)
        bus.write_byte_data.assert_any_call(0x08, REG_LOOK_Y, (-25) & 0xFF)


class TestSystemStatus:
    """Tests for set_system_status()."""

    @pytest.mark.parametrize("name,expected_val", list(STATUS_MAP.items()))
    def test_valid_statuses(self, client, name, expected_val):
        c, bus = client
        assert c.set_system_status(name) is True
        bus.write_byte_data.assert_called_once_with(0x08, REG_SYSTEM_STATUS, expected_val)

    def test_invalid_status_returns_false(self, client):
        c, bus = client
        assert c.set_system_status("invalid") is False
        bus.write_byte_data.assert_not_called()


class TestErrorHandling:
    """Tests for I2C error scenarios."""

    def test_write_fails_on_os_error(self, client):
        c, bus = client
        bus.write_byte_data.side_effect = OSError("I2C timeout")
        assert c.set_expression("happy") is False

    def test_operations_fail_when_bus_not_open(self):
        c = HeadI2CClient()
        assert c.set_expression("happy") is False
        assert c.trigger_blink() is False
        assert c.set_look_direction(0, 0) is False
        assert c.set_system_status("idle") is False
        assert c.read_status() is None
