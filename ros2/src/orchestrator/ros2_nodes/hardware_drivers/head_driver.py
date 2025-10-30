#!/usr/bin/env python3
"""
ROS2 Head Driver Node - I2C Bridge for Head Module.

This node translates ROS2 topics into I2C register writes for the Head module,
enabling the orchestrator to control eye displays without knowing I2C details.

The Head module (ESP32 at I2C address 0x08) controls:
- 2× GC9A01 round TFT displays (eyes)
- Eye expression rendering at 60 FPS
- Synchronized blink animations
- Expression intensity levels (1-5)

Architecture:
- ROS2 Node runs on Raspberry Pi
- ESP32 runs as I2C slave with embedded animation engine
- This node acts as bridge: ROS2 topics → I2C register writes
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None  # Allow import for testing without hardware


# ==============================================================================
# I2C CONFIGURATION
# ==============================================================================

# I2C bus number (typically 1 on Raspberry Pi)
I2C_BUS = 1

# Head module I2C slave address
I2C_HEAD_ADDRESS = 0x08


# ==============================================================================
# I2C REGISTER MAP - HEAD MODULE
# ==============================================================================
# These registers are defined in the ESP32 firmware (Story 1.4)
# and provide the interface for controlling the head module.

# Standard registers (common across all modules)
REG_MODULE_ID = 0x00           # Read-only: Returns 0x08 for head module
REG_FIRMWARE_VERSION = 0x01    # Read-only: Firmware version number
REG_STATUS = 0x02              # Read-only: Module status byte (see STATUS_* flags)
REG_ERROR_CODE = 0x03          # Read-only: Last error code if STATUS_ERROR set
REG_COMMAND = 0x04             # Write: General command trigger register

# Head module specific registers
REG_EXPRESSION_TYPE = 0x10     # Write: Expression type (0-6, see emotion_map below)
REG_EXPRESSION_INTENSITY = 0x11  # Write: Intensity level (1-5, where 1=subtle, 5=extreme)
REG_BLINK_TRIGGER = 0x12       # Write: Any value triggers synchronized blink animation


# ==============================================================================
# STATUS BYTE BIT FLAGS
# ==============================================================================
# The status register (0x02) uses bit flags to indicate module state.

STATUS_READY = 0x01   # Module is ready to accept commands
STATUS_BUSY = 0x02    # Module is processing (e.g., animating transition)
STATUS_ERROR = 0x04   # Module encountered an error (check REG_ERROR_CODE)


# ==============================================================================
# RETRY AND TIMING CONFIGURATION
# ==============================================================================

# Number of I2C retry attempts before giving up
I2C_RETRY_ATTEMPTS = 3

# Delay between retry attempts (milliseconds)
I2C_RETRY_DELAY_MS = 100

# Status update frequency (Hz) - how often we poll module status
STATUS_UPDATE_HZ = 10.0


# ==============================================================================
# HEAD DRIVER NODE CLASS
# ==============================================================================

class HeadDriverNode(Node):
    """ROS2 node that bridges ROS2 topics to I2C register writes for Head module.

    This node subscribes to expression and blink topics, translates them to I2C
    register writes, and publishes module status at 10Hz.

    Responsibilities:
    1. Initialize I2C communication with Head module
    2. Perform health check on startup (verify module ID)
    3. Subscribe to expression and blink commands
    4. Translate ROS2 messages to I2C register writes
    5. Publish module status periodically
    6. Handle I2C errors with retry logic and graceful degradation

    Attributes:
        _i2c_bus: SMBus instance for I2C communication.
        _module_healthy: Flag indicating if module responded correctly during startup.
        _last_status: Cached status byte from last successful read (for error recovery).
        _status_publisher: Publisher for module status messages.
        _status_timer: Timer for periodic status updates (10Hz).
    """

    def __init__(self):
        """Initialize the Head Driver node.

        Sets up I2C communication, performs module health check, and creates
        ROS2 subscriptions and publications.
        """
        super().__init__('head_driver')

        # I2C communication state
        self._i2c_bus: Optional[SMBus] = None
        self._module_healthy: bool = False
        self._last_status: int = STATUS_ERROR  # Start pessimistic

        # Initialize I2C bus connection
        self._init_i2c()

        # Perform module health check (verify module ID)
        self._check_module_health()

        # Create ROS2 subscriptions (expression, blink commands)
        self._setup_subscriptions()

        # Create ROS2 publications (status updates)
        self._setup_publications()

        self.get_logger().info('Head driver node initialized')

    # ==========================================================================
    # INITIALIZATION METHODS
    # ==========================================================================

    def _init_i2c(self) -> None:
        """Initialize I2C bus connection.

        Opens the I2C bus (typically /dev/i2c-1 on Raspberry Pi) for communication
        with the Head module.

        Raises:
            RuntimeError: If smbus2 library is not available.
            Exception: If I2C bus cannot be opened (permissions, hardware issue).
        """
        # Check if smbus2 library is available
        if SMBus is None:
            self.get_logger().error('smbus2 library not available - install with: pip install smbus2')
            raise RuntimeError('smbus2 library required for I2C communication')

        try:
            # Open I2C bus (typically bus 1 on Raspberry Pi)
            self._i2c_bus = SMBus(I2C_BUS)
            self.get_logger().info(f'I2C bus {I2C_BUS} opened successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C bus {I2C_BUS}: {e}')
            self.get_logger().error('Ensure I2C is enabled and user is in i2c group')
            raise

    def _check_module_health(self) -> None:
        """Perform module health check on startup.

        Reads the module ID register (0x00) to verify the Head module is responding
        correctly. A healthy Head module should return 0x08 (its I2C address).

        This health check ensures:
        1. I2C communication is working
        2. The correct module is connected at address 0x08
        3. Module firmware is running and responsive

        Sets internal health status flag based on the result.
        """
        self.get_logger().info('Performing Head module health check...')

        # Read module ID register (should return 0x08 for Head module)
        module_id = self._read_register(REG_MODULE_ID)

        if module_id == I2C_HEAD_ADDRESS:
            # Health check passed - module responding correctly
            self._module_healthy = True
            self._last_status = STATUS_READY
            self.get_logger().info(
                f'Head module health check PASSED - Module ID: 0x{module_id:02X}'
            )
        else:
            # Health check failed - module not responding or wrong ID
            self._module_healthy = False
            self._last_status = STATUS_ERROR
            if module_id is None:
                self.get_logger().error('Head module health check FAILED - No response from 0x08')
                self.get_logger().error('Check: ESP32 powered on, I2C wiring, pull-up resistors')
            else:
                self.get_logger().error(
                    f'Head module health check FAILED - Expected 0x{I2C_HEAD_ADDRESS:02X}, '
                    f'got 0x{module_id:02X}'
                )
                self.get_logger().error('Wrong module at address 0x08 or firmware issue')

    # ==========================================================================
    # ROS2 TOPIC SETUP
    # ==========================================================================

    def _setup_subscriptions(self) -> None:
        """Create ROS2 topic subscriptions for incoming commands.

        Subscribes to:
        1. /olaf/head/expression - Expression change commands (emotion + intensity)
        2. /olaf/head/blink - Blink trigger commands (synchronized blink animation)

        These topics allow higher-level nodes (personality coordinator, AI agent)
        to control eye expressions without knowing I2C details.
        """
        # Subscribe to expression commands
        # Format: "emotion_name,intensity" (e.g., "happy,4" or just "happy")
        self.create_subscription(
            String,
            '/olaf/head/expression',
            self._expression_callback,
            10  # Queue size
        )

        # Subscribe to blink trigger commands
        # Format: Empty message (just triggers blink, no parameters)
        self.create_subscription(
            Empty,
            '/olaf/head/blink',
            self._blink_callback,
            10  # Queue size
        )

        self.get_logger().info('Subscriptions created: /olaf/head/expression, /olaf/head/blink')

    def _setup_publications(self) -> None:
        """Create ROS2 topic publications and timers for status updates.

        Publishes:
        1. /olaf/head/status - Module status string ("READY", "BUSY", "ERROR")

        Status is published at 10Hz to allow monitoring nodes to detect failures
        quickly while not overwhelming the I2C bus.
        """
        # Create status publisher
        self._status_publisher = self.create_publisher(
            String,
            '/olaf/head/status',
            10  # Queue size
        )

        # Create timer for periodic status updates (10Hz = 100ms period)
        status_period = 1.0 / STATUS_UPDATE_HZ
        self._status_timer = self.create_timer(status_period, self._status_callback)

        self.get_logger().info(f'Status publisher created: /olaf/head/status at {STATUS_UPDATE_HZ}Hz')

    # ==========================================================================
    # I2C COMMUNICATION METHODS (with retry logic)
    # ==========================================================================

    def _read_register(self, register: int) -> Optional[int]:
        """Read a single byte from an I2C register with retry logic.

        Attempts to read from the specified register up to I2C_RETRY_ATTEMPTS times
        with I2C_RETRY_DELAY_MS delay between attempts. This handles transient
        I2C bus errors (noise, timing issues, module busy).

        Args:
            register: The register address to read from (0x00-0xFF).

        Returns:
            The byte value read from the register (0x00-0xFF), or None if all
            retry attempts failed.

        Note:
            - Logs warnings for each failed attempt
            - Logs error only after all attempts exhausted
            - Returns None on failure (caller should handle gracefully)
        """
        if self._i2c_bus is None:
            self.get_logger().error('I2C bus not initialized')
            return None

        # Retry loop - attempt up to I2C_RETRY_ATTEMPTS times
        for attempt in range(I2C_RETRY_ATTEMPTS):
            try:
                # Attempt I2C read: read 1 byte from register at HEAD address
                value = self._i2c_bus.read_byte_data(I2C_HEAD_ADDRESS, register)
                return value  # Success!

            except OSError as e:
                # I2C operation failed (timeout, NACK, bus error, etc.)
                if attempt < I2C_RETRY_ATTEMPTS - 1:
                    # Not the last attempt - log warning and retry
                    self.get_logger().warning(
                        f'I2C read failed (attempt {attempt + 1}/{I2C_RETRY_ATTEMPTS}), '
                        f'register 0x{register:02X}: {e}'
                    )
                    time.sleep(I2C_RETRY_DELAY_MS / 1000.0)  # Wait before retry
                else:
                    # Last attempt failed - log error and give up
                    self.get_logger().error(
                        f'I2C read failed after {I2C_RETRY_ATTEMPTS} attempts, '
                        f'register 0x{register:02X}: {e}'
                    )

        return None  # All retry attempts exhausted

    def _write_register(self, register: int, value: int) -> bool:
        """Write a single byte to an I2C register with retry logic.

        Attempts to write to the specified register up to I2C_RETRY_ATTEMPTS times
        with I2C_RETRY_DELAY_MS delay between attempts. This handles transient
        I2C bus errors.

        Args:
            register: The register address to write to (0x00-0xFF).
            value: The byte value to write (0x00-0xFF).

        Returns:
            True if write succeeded, False if all retry attempts failed.

        Note:
            - Logs warnings for each failed attempt
            - Logs error only after all attempts exhausted
            - Returns False on failure (caller should check and handle)
        """
        if self._i2c_bus is None:
            self.get_logger().error('I2C bus not initialized')
            return False

        # Retry loop - attempt up to I2C_RETRY_ATTEMPTS times
        for attempt in range(I2C_RETRY_ATTEMPTS):
            try:
                # Attempt I2C write: write 1 byte to register at HEAD address
                self._i2c_bus.write_byte_data(I2C_HEAD_ADDRESS, register, value)
                return True  # Success!

            except OSError as e:
                # I2C operation failed (timeout, NACK, bus error, etc.)
                if attempt < I2C_RETRY_ATTEMPTS - 1:
                    # Not the last attempt - log warning and retry
                    self.get_logger().warning(
                        f'I2C write failed (attempt {attempt + 1}/{I2C_RETRY_ATTEMPTS}), '
                        f'register 0x{register:02X}, value 0x{value:02X}: {e}'
                    )
                    time.sleep(I2C_RETRY_DELAY_MS / 1000.0)  # Wait before retry
                else:
                    # Last attempt failed - log error and give up
                    self.get_logger().error(
                        f'I2C write failed after {I2C_RETRY_ATTEMPTS} attempts, '
                        f'register 0x{register:02X}, value 0x{value:02X}: {e}'
                    )

        return False  # All retry attempts exhausted

    # ==========================================================================
    # ROS2 CALLBACK METHODS
    # ==========================================================================

    def _expression_callback(self, msg: String) -> None:
        """Handle expression topic messages.

        Translates high-level expression commands into I2C register writes.

        Message format: "emotion_name,intensity" or just "emotion_name"
        - emotion_name: One of: neutral, happy, curious, thinking, confused, sad, excited
        - intensity: Optional, 1-5 (1=subtle, 5=extreme), defaults to 3

        Examples:
        - "happy,5" → Happy expression at maximum intensity
        - "curious" → Curious expression at default intensity (3)
        - "thinking,2" → Thinking expression at subtle intensity

        Args:
            msg: String message containing expression data.

        I2C Operations:
        - Writes emotion type to register 0x10 (REG_EXPRESSION_TYPE)
        - Writes intensity to register 0x11 (REG_EXPRESSION_INTENSITY)
        - ESP32 firmware then renders expression at 60 FPS
        """
        try:
            # Parse message: split by comma to get emotion and optional intensity
            parts = msg.data.split(',')
            emotion = parts[0].strip().lower()

            # Map emotion names to register values (0-6)
            # These values match the expression types in ESP32 firmware
            emotion_map = {
                'neutral': 0,   # Default neutral expression
                'happy': 1,     # Wide eyes, slight upward curve
                'curious': 2,   # Wide eyes with focus
                'thinking': 3,  # Eyes looking up/side
                'confused': 4,  # Asymmetric eyes
                'sad': 5,       # Droopy eyes, downward curve
                'excited': 6    # Wide eyes with sparkle/motion
            }

            # Validate emotion name
            if emotion not in emotion_map:
                self.get_logger().warning(
                    f'Unknown emotion: "{emotion}" - Valid: {list(emotion_map.keys())}'
                )
                return

            emotion_value = emotion_map[emotion]

            # Parse intensity (default to 3 if not provided)
            intensity = 3  # Default: medium intensity
            if len(parts) > 1:
                try:
                    intensity = int(parts[1].strip())
                    # Clamp intensity to valid range [1, 5]
                    intensity = max(1, min(5, intensity))
                except ValueError:
                    self.get_logger().warning(
                        f'Invalid intensity: "{parts[1]}", using default 3'
                    )

            # Write to I2C registers
            # Note: Both writes must succeed for consistent expression state
            success = True
            success &= self._write_register(REG_EXPRESSION_TYPE, emotion_value)
            success &= self._write_register(REG_EXPRESSION_INTENSITY, intensity)

            if success:
                self.get_logger().info(
                    f'Expression set: {emotion} (type={emotion_value}) intensity={intensity}'
                )
            else:
                self.get_logger().error(
                    f'Failed to set expression: {emotion} - I2C communication error'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing expression message: {e}')

    def _blink_callback(self, msg: Empty) -> None:
        """Handle blink trigger messages.

        Triggers a synchronized blink animation on both eye displays.
        The ESP32 firmware handles the blink timing and animation (typically 150-200ms).

        Args:
            msg: Empty message (blink has no parameters).

        I2C Operations:
        - Writes any value (we use 0x01) to register 0x12 (REG_BLINK_TRIGGER)
        - ESP32 firmware detects write and initiates blink sequence
        - Blink is non-blocking: firmware returns immediately, animation runs in background
        """
        # Trigger blink by writing to blink register (value doesn't matter)
        success = self._write_register(REG_BLINK_TRIGGER, 0x01)

        if success:
            self.get_logger().info('Blink triggered')
        else:
            self.get_logger().error('Failed to trigger blink - I2C communication error')

    def _status_callback(self) -> None:
        """Periodic callback to read and publish module status.

        Called at 10Hz by timer. Reads status register from Head module and
        publishes human-readable status string.

        Status mapping:
        - STATUS_ERROR (bit 2) → "ERROR" (hardware failure, initialization error)
        - STATUS_BUSY (bit 1) → "BUSY" (animating transition, processing command)
        - STATUS_READY (bit 0) → "READY" (idle, ready for commands)
        - No bits set → "UNKNOWN" (unexpected state)

        Error handling:
        - If I2C read fails, uses cached status from last successful read
        - Publishes "ERROR" status to indicate communication failure
        - Higher-level nodes can monitor status to detect module failures
        """
        # Read status register (0x02) from Head module
        status_byte = self._read_register(REG_STATUS)

        if status_byte is not None:
            # Read succeeded - update cache for future error recovery
            self._last_status = status_byte

            # Map status byte bit flags to human-readable string
            # Priority: ERROR > BUSY > READY (check most critical first)
            if status_byte & STATUS_ERROR:
                status_str = 'ERROR'
            elif status_byte & STATUS_BUSY:
                status_str = 'BUSY'
            elif status_byte & STATUS_READY:
                status_str = 'READY'
            else:
                status_str = 'UNKNOWN'
        else:
            # I2C read failed - report error status
            # Note: We could use cached status, but ERROR is more accurate
            # to indicate current communication failure
            status_str = 'ERROR'

        # Publish status message
        status_msg = String()
        status_msg.data = status_str
        self._status_publisher.publish(status_msg)

    # ==========================================================================
    # CLEANUP
    # ==========================================================================

    def destroy_node(self) -> None:
        """Clean up resources when node is destroyed.

        Closes I2C bus connection to free hardware resources.
        Called automatically by ROS2 on node shutdown.
        """
        if self._i2c_bus is not None:
            self._i2c_bus.close()
            self.get_logger().info('I2C bus closed')

        super().destroy_node()


# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================

def main(args=None):
    """Main entry point for the head driver node.

    Initializes ROS2, creates the node, and spins (processes callbacks) until
    interrupted or shutdown.

    Args:
        args: Command-line arguments (default: None).
    """
    # Initialize ROS2 Python client library
    rclpy.init(args=args)

    try:
        # Create and run the head driver node
        node = HeadDriverNode()
        rclpy.spin(node)  # Block and process callbacks until shutdown
    except KeyboardInterrupt:
        # User pressed Ctrl+C - normal shutdown
        pass
    except Exception as e:
        # Unexpected error - log and exit
        print(f'Error in head driver node: {e}')
    finally:
        # Clean shutdown of ROS2
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
