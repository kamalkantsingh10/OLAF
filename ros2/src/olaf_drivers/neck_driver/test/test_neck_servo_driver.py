"""Unit tests for NeckServoDriver.

Tests position calculations, clamping, tilt/roll math, and feedback
using a mocked scservo_sdk.
"""

from unittest.mock import MagicMock, patch, mock_open
import pytest
import yaml

from neck_driver.neck_servo_driver import NeckServoDriver

# Minimal config for testing
TEST_CONFIG = {
    "neck": {
        "port": "/dev/waveshare_neck",
        "baudrate": 1000000,
        "protocol": "sms_sts",
        "counts_per_degree": 11.38,
        "base_speed": 500,
        "base_acc": 50,
        "servos": {
            "pan": {
                "id": 11,
                "model": "STS3215",
                "function": "Neck base rotation",
                "center_position": 2436,
                "min_angle": -90,
                "max_angle": 90,
                "direction": -1,
            },
            "left_linkage": {
                "id": 12,
                "model": "STS3215",
                "function": "Left linkage",
                "center_position": 3426,
                "min_angle": -8,
                "max_angle": 8,
                "direction": 1,
            },
            "right_linkage": {
                "id": 13,
                "model": "STS3215",
                "function": "Right linkage",
                "center_position": 215,
                "min_angle": -8,
                "max_angle": 8,
                "direction": -1,
            },
        },
    }
}


@pytest.fixture
def driver():
    """Create a NeckServoDriver with mocked port and config."""
    config_yaml = yaml.dump(TEST_CONFIG)
    with patch("neck_driver.neck_servo_driver.PortHandler") as mock_port_cls, \
         patch("neck_driver.neck_servo_driver.sms_sts") as mock_sts_cls, \
         patch("builtins.open", mock_open(read_data=config_yaml)), \
         patch("os.path.isfile", return_value=True):
        mock_port = MagicMock()
        mock_port.openPort.return_value = True
        mock_port.setBaudRate.return_value = True
        mock_port_cls.return_value = mock_port

        mock_packet = MagicMock()
        mock_packet.WritePosEx.return_value = (0, 0)  # COMM_SUCCESS=0, no error
        mock_sts_cls.return_value = mock_packet

        d = NeckServoDriver()
        yield d, mock_packet


class TestDegreesToPosition:
    """Tests for position calculation."""

    def test_pan_center(self, driver):
        d, _ = driver
        servo = d._get_servo("pan")
        pos = d._degrees_to_position(servo, 0)
        assert pos == 2436

    def test_pan_left_30(self, driver):
        """Pan left 30 deg (direction=-1): center + 30*(-1)*11.38 = 2436 - 341 = 2095."""
        d, _ = driver
        servo = d._get_servo("pan")
        pos = d._degrees_to_position(servo, 30)
        assert pos == 2436 + int(30 * -1 * 11.38)  # 2095

    def test_pan_right_30(self, driver):
        d, _ = driver
        servo = d._get_servo("pan")
        pos = d._degrees_to_position(servo, -30)
        assert pos == 2436 + int(-30 * -1 * 11.38)  # 2777

    def test_left_linkage_up_5(self, driver):
        """Left linkage +5 deg (direction=1): 3426 + 5*1*11.38 = 3482."""
        d, _ = driver
        servo = d._get_servo("left_linkage")
        pos = d._degrees_to_position(servo, 5)
        assert pos == 3426 + int(5 * 1 * 11.38)  # 3482

    def test_right_linkage_tilt_up_5(self, driver):
        """Right linkage +5 deg (direction=-1): 215 + 5*(-1)*11.38 = 158."""
        d, _ = driver
        servo = d._get_servo("right_linkage")
        pos = d._degrees_to_position(servo, 5)
        assert pos == 215 + int(5 * -1 * 11.38)  # 158

    def test_position_clamped_to_0_4095(self, driver):
        d, _ = driver
        servo = d._get_servo("pan")
        pos = d._degrees_to_position(servo, 500)
        assert pos == 0  # direction=-1, goes below 0
        pos2 = d._degrees_to_position(servo, -500)
        assert pos2 == 4095  # direction=-1, goes above 4095


class TestAngleClamping:
    """Tests for angle limit clamping."""

    def test_pan_clamps_to_90(self, driver):
        d, _ = driver
        servo = d._get_servo("pan")
        assert d._clamp_angle(servo, 120) == 90

    def test_pan_clamps_to_neg90(self, driver):
        d, _ = driver
        servo = d._get_servo("pan")
        assert d._clamp_angle(servo, -120) == -90

    def test_linkage_clamps_to_8(self, driver):
        d, _ = driver
        servo = d._get_servo("left_linkage")
        assert d._clamp_angle(servo, 15) == 8

    def test_within_range_not_clamped(self, driver):
        d, _ = driver
        servo = d._get_servo("pan")
        assert d._clamp_angle(servo, 45) == 45


class TestMovePan:
    """Tests for pan movement."""

    def test_pan_left(self, driver):
        d, packet = driver
        d.move_pan(30)
        expected_pos = 2436 + int(30 * -1 * 11.38)
        packet.WritePosEx.assert_any_call(11, expected_pos, 500, 50)

    def test_pan_right(self, driver):
        d, packet = driver
        d.move_pan(-30)
        expected_pos = 2436 + int(-30 * -1 * 11.38)
        packet.WritePosEx.assert_any_call(11, expected_pos, 500, 50)


class TestMoveTilt:
    """Tests for tilt movement (parallel linkage)."""

    def test_tilt_up_5(self, driver):
        """Tilt up: left=+5deg, right=+5deg (directions handle physics)."""
        d, packet = driver
        d.move_tilt(5)
        left_pos = 3426 + int(5 * 1 * 11.38)   # 3482
        right_pos = 215 + int(5 * -1 * 11.38)   # 158
        calls = [c.args for c in packet.WritePosEx.call_args_list]
        assert (12, left_pos, 500, 50) in calls
        assert (13, right_pos, 500, 50) in calls

    def test_tilt_down_5(self, driver):
        d, packet = driver
        d.move_tilt(-5)
        left_pos = 3426 + int(-5 * 1 * 11.38)   # 3369
        right_pos = 215 + int(-5 * -1 * 11.38)   # 271
        calls = [c.args for c in packet.WritePosEx.call_args_list]
        assert (12, left_pos, 500, 50) in calls
        assert (13, right_pos, 500, 50) in calls

    def test_tilt_clamped_to_8(self, driver):
        """Tilt 20 deg should be clamped to 8 (linkage limit)."""
        d, packet = driver
        d.move_tilt(20)
        left_pos = 3426 + int(8 * 1 * 11.38)
        right_pos = 215 + int(8 * -1 * 11.38)
        calls = [c.args for c in packet.WritePosEx.call_args_list]
        assert (12, left_pos, 500, 50) in calls
        assert (13, right_pos, 500, 50) in calls


class TestMoveRoll:
    """Tests for roll movement (parallel linkage, same direction)."""

    def test_roll_right_3(self, driver):
        """Roll right: left=+3, right=-3 (right dir inverts to same physical dir)."""
        d, packet = driver
        d.move_roll(3)
        # move_pose(tilt=0, roll=3): left_deg=0+3=3, right_deg=0-3=-3
        left_pos = 3426 + int(3 * 1 * 11.38)    # 3460
        right_pos = 215 + int(-3 * -1 * 11.38)   # 249
        calls = [c.args for c in packet.WritePosEx.call_args_list]
        assert (12, left_pos, 500, 50) in calls
        assert (13, right_pos, 500, 50) in calls


class TestMovePose:
    """Tests for combined pan+tilt+roll."""

    def test_pose_all_zero(self, driver):
        d, packet = driver
        d.move_pose(0, 0, 0)
        calls = [c.args for c in packet.WritePosEx.call_args_list]
        assert (11, 2436, 500, 50) in calls
        assert (12, 3426, 500, 50) in calls
        assert (13, 215, 500, 50) in calls

    def test_pose_tilt_and_roll(self, driver):
        """Tilt up 3 + roll right 2: left=3+2=5, right=3-2=1."""
        d, packet = driver
        d.move_pose(pan=0, tilt=3, roll=2)
        left_pos = 3426 + int(5 * 1 * 11.38)
        right_pos = 215 + int(1 * -1 * 11.38)
        calls = [c.args for c in packet.WritePosEx.call_args_list]
        assert (12, left_pos, 500, 50) in calls
        assert (13, right_pos, 500, 50) in calls

    def test_pose_combined_clamped(self, driver):
        """Tilt 6 + roll 6 = left_deg=12 clamped to 8."""
        d, packet = driver
        d.move_pose(pan=0, tilt=6, roll=6)
        # left_deg = 6+6 = 12, clamped to 8
        # right_deg = 6-6 = 0
        left_pos = 3426 + int(8 * 1 * 11.38)
        right_pos = 215 + int(0 * -1 * 11.38)
        calls = [c.args for c in packet.WritePosEx.call_args_list]
        assert (12, left_pos, 500, 50) in calls
        assert (13, right_pos, 500, 50) in calls


class TestFeedback:
    """Tests for servo feedback reading."""

    def test_read_position(self, driver):
        d, packet = driver
        # Simulate pan at 30 deg left: pos = 2436 + 30*(-1)*11.38 = 2095
        packet.ReadPos.return_value = (2095, 0, 0)
        deg = d.read_position("pan")
        assert deg is not None
        assert abs(deg - 30) < 0.5  # (2095-2436) / (-1*11.38) ≈ 30

    def test_read_position_fail(self, driver):
        d, packet = driver
        packet.ReadPos.return_value = (0, 1, 0)  # COMM fail
        assert d.read_position("pan") is None

    def test_read_voltage(self, driver):
        d, packet = driver
        packet.read1ByteTxRx.return_value = (53, 0, 0)
        v = d.read_voltage("pan")
        assert v == 5.3

    def test_read_temperature(self, driver):
        d, packet = driver
        packet.read1ByteTxRx.return_value = (29, 0, 0)
        t = d.read_temperature("pan")
        assert t == 29

    def test_read_load(self, driver):
        d, packet = driver
        packet.read2ByteTxRx.return_value = (360, 0, 0)
        load = d.read_load("pan")
        assert load == 360

    def test_read_all_feedback(self, driver):
        d, packet = driver
        packet.ReadPos.return_value = (2436, 0, 0)
        packet.read2ByteTxRx.return_value = (100, 0, 0)
        packet.read1ByteTxRx.side_effect = [(53, 0, 0), (29, 0, 0)]
        fb = d.read_all_feedback("pan")
        assert fb is not None
        assert "position_deg" in fb
        assert "load" in fb
        assert "voltage" in fb
        assert "temperature" in fb


class TestErrorHandling:
    """Tests for error scenarios."""

    def test_invalid_servo_name(self, driver):
        d, _ = driver
        with pytest.raises(ValueError, match="Unknown servo"):
            d._get_servo("nonexistent")

    def test_move_returns_false_on_comm_fail(self, driver):
        d, packet = driver
        packet.WritePosEx.return_value = (1, 0)  # COMM fail
        assert d.move_pan(10) is False
