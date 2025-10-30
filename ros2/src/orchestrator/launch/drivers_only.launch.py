"""
OLAF Hardware Driver Nodes Launch File (Raspberry Pi Only)

This launch file starts ONLY the hardware driver nodes that require direct I2C access.
These nodes MUST run on the Raspberry Pi as they need /dev/i2c-1 hardware access.

Use this launch file during PC+Pi hybrid development:
  - Pi runs: ros2 launch orchestrator drivers_only.launch.py
  - PC runs: ros2 launch orchestrator app_nodes.launch.py

For production (all nodes on Pi):
  - Pi runs: ros2 launch orchestrator olaf_full.launch.py

Requirements:
  - I2C enabled on Pi (sudo raspi-config -> Interface Options -> I2C)
  - User in i2c group (sudo usermod -a -G i2c $USER)
  - smbus2 installed (pip3 install smbus2)
  - ROS_DOMAIN_ID environment variable set (e.g., export ROS_DOMAIN_ID=42)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for hardware driver nodes only."""

    # Declare launch arguments
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number (default: 1 for /dev/i2c-1)'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Launch configuration
    i2c_bus = LaunchConfiguration('i2c_bus')
    log_level = LaunchConfiguration('log_level')

    # Hardware driver nodes
    head_driver_node = Node(
        package='orchestrator',
        executable='head_driver_node',
        name='head_driver',
        namespace='olaf',
        output='screen',
        parameters=[{
            'i2c_bus': i2c_bus,
            'i2c_address': 0x08,
            'i2c_timeout_ms': 100,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    ears_neck_driver_node = Node(
        package='orchestrator',
        executable='ears_neck_driver_node',
        name='ears_neck_driver',
        namespace='olaf',
        output='screen',
        parameters=[{
            'i2c_bus': i2c_bus,
            'i2c_address': 0x09,
            'i2c_timeout_ms': 100,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    body_driver_node = Node(
        package='orchestrator',
        executable='body_driver_node',
        name='body_driver',
        namespace='olaf',
        output='screen',
        parameters=[{
            'i2c_bus': i2c_bus,
            'i2c_address': 0x0A,
            'i2c_timeout_ms': 100,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    base_driver_node = Node(
        package='orchestrator',
        executable='base_driver_node',
        name='base_driver',
        namespace='olaf',
        output='screen',
        parameters=[{
            'i2c_bus': i2c_bus,
            'i2c_address': 0x0B,
            'i2c_timeout_ms': 100,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Startup log message
    startup_message = LogInfo(
        msg='='*60 + '\n' +
            'OLAF Hardware Driver Nodes (Pi Only)\n' +
            '='*60 + '\n' +
            'Starting I2C driver nodes for 4-module architecture:\n' +
            '  - Head Module (0x08): Eyes, mmWave, speaker\n' +
            '  - Ears+Neck Module (0x09): Servos (4x ears + 3x neck)\n' +
            '  - Body Module (0x0A): Heart LCD, projector, LEDs\n' +
            '  - Base Module (0x0B): Self-balancing, motors, kickstand\n' +
            '\n' +
            'These nodes require direct I2C hardware access (/dev/i2c-1).\n' +
            'Application nodes can run on PC - see app_nodes.launch.py\n' +
            '='*60
    )

    return LaunchDescription([
        i2c_bus_arg,
        log_level_arg,
        startup_message,
        head_driver_node,
        ears_neck_driver_node,
        body_driver_node,
        base_driver_node,
    ])
