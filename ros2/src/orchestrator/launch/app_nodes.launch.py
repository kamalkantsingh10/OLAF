"""
OLAF Application Nodes Launch File (Can Run on PC or Pi)

This launch file starts application-level nodes that do NOT require hardware access.
These nodes can run on your development PC during hybrid PC+Pi development.

Use this launch file during PC+Pi hybrid development:
  - Pi runs: ros2 launch orchestrator drivers_only.launch.py
  - PC runs: ros2 launch orchestrator app_nodes.launch.py (THIS FILE)

For production (all nodes on Pi):
  - Pi runs: ros2 launch orchestrator olaf_full.launch.py

Requirements:
  - ROS2 Humble installed on PC
  - ROS_DOMAIN_ID matching Pi (e.g., export ROS_DOMAIN_ID=42)
  - PC and Pi on same network
  - Python dependencies: pip3 install -r orchestrator/requirements.txt

Network Configuration:
  - Ensure ROS_DOMAIN_ID is identical on PC and Pi
  - Check firewall allows DDS multicast (UDP ports 7400-7500)
  - Verify: ros2 topic list (should see /olaf/* topics from Pi)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for application nodes (no hardware dependencies)."""

    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    enable_ai_arg = DeclareLaunchArgument(
        'enable_ai',
        default_value='false',
        description='Enable AI agent and STT nodes (requires API keys)'
    )

    enable_nav_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='false',
        description='Enable SLAM and navigation nodes'
    )

    # Launch configuration
    log_level = LaunchConfiguration('log_level')
    enable_ai = LaunchConfiguration('enable_ai')
    enable_nav = LaunchConfiguration('enable_navigation')

    # Core application nodes (always run)
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

    # AI nodes (optional - requires API keys)
    ai_agent_node = Node(
        package='orchestrator',
        executable='ai_agent_node',
        name='ai_agent',
        namespace='olaf',
        output='screen',
        parameters=[{
            'api_provider': 'claude',  # 'claude' or 'openai'
            'model': 'claude-3-5-sonnet-20241022',
        }],
        arguments=['--ros-args', '--log-level', log_level],
        condition=launch.conditions.IfCondition(enable_ai),
    )

    # Note: whisper_stt_node should run on Pi (needs Hailo hardware)
    # Placeholder here for documentation - actual node runs on Pi

    # Navigation nodes (optional - requires sensors/motors)
    # Note: These should eventually run on Pi for sensor access
    # Included here for initial development/testing

    # Startup log message
    startup_message = LogInfo(
        msg='='*60 + '\n' +
            'OLAF Application Nodes (PC or Pi)\n' +
            '='*60 + '\n' +
            'Starting application-level nodes:\n' +
            '  - Personality Coordinator: Expression orchestration\n' +
            '  - AI Agent: Cloud LLM reasoning (if enabled)\n' +
            '\n' +
            'These nodes do NOT require hardware access.\n' +
            'Ensure driver nodes are running on Pi (drivers_only.launch.py)\n' +
            '\n' +
            'Network Requirements:\n' +
            '  - ROS_DOMAIN_ID must match Pi (check: echo $ROS_DOMAIN_ID)\n' +
            '  - PC and Pi must be on same network\n' +
            '  - Verify connectivity: ros2 topic list (should see /olaf/*)\n' +
            '='*60
    )

    return LaunchDescription([
        log_level_arg,
        enable_ai_arg,
        enable_nav_arg,
        startup_message,
        personality_coordinator_node,
        # ai_agent_node,  # Uncomment when AI nodes are implemented
    ])
