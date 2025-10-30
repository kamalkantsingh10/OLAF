"""
OLAF Full System Launch File (Production - All Nodes on Pi)

This launch file starts ALL OLAF nodes on the Raspberry Pi for production use.
Combines both hardware driver nodes and application nodes in a single launch.

Use this launch file for:
  - Production deployment (robot running autonomously)
  - Full system testing on Pi
  - Final integration before deployment

For development (PC+Pi hybrid):
  - Pi runs: ros2 launch orchestrator drivers_only.launch.py
  - PC runs: ros2 launch orchestrator app_nodes.launch.py

Requirements:
  - All Story 1.3 setup complete (ROS2, I2C, smbus2)
  - ESP32 modules connected via I2C
  - API keys configured in .env file (for AI features)
  - Sensors/actuators connected (camera, motors, etc.)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for full OLAF system."""

    # Get package directory
    pkg_dir = get_package_share_directory('orchestrator')

    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='1',
        description='I2C bus number (default: 1 for /dev/i2c-1)'
    )

    # Launch configuration
    log_level = LaunchConfiguration('log_level')
    i2c_bus = LaunchConfiguration('i2c_bus')

    # Include driver nodes launch file
    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'drivers_only.launch.py')
        ),
        launch_arguments={
            'log_level': log_level,
            'i2c_bus': i2c_bus,
        }.items()
    )

    # Application nodes
    personality_coordinator_node = Node(
        package='orchestrator',
        executable='personality_coordinator_node',
        name='personality_coordinator',
        namespace='olaf',
        output='screen',
        parameters=[{
            'expression_sync_timeout_ms': 500,
            'default_emotion': 'neutral',
            'default_intensity': 2,
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # AI nodes (uncomment when implemented)
    # ai_agent_node = Node(...)
    # whisper_stt_node = Node(...)

    # Navigation nodes (uncomment when implemented)
    # cartographer_node = Node(...)
    # nav2_launch = IncludeLaunchDescription(...)

    # Startup log message
    startup_message = LogInfo(
        msg='='*60 + '\n' +
            'OLAF Full System Launch (Production)\n' +
            '='*60 + '\n' +
            'Starting complete OLAF system:\n' +
            '\n' +
            'Hardware Drivers:\n' +
            '  - Head Module (0x08)\n' +
            '  - Ears+Neck Module (0x09)\n' +
            '  - Body Module (0x0A)\n' +
            '  - Base Module (0x0B)\n' +
            '\n' +
            'Application Nodes:\n' +
            '  - Personality Coordinator\n' +
            '  - AI Agent (if configured)\n' +
            '  - Navigation (if configured)\n' +
            '\n' +
            'All nodes running on Raspberry Pi.\n' +
            '='*60
    )

    return LaunchDescription([
        log_level_arg,
        i2c_bus_arg,
        startup_message,
        drivers_launch,
        personality_coordinator_node,
        # Add more application nodes as implemented
    ])
