"""Launch file for OLAF ears node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="head_ears_driver",
            executable="ears_node",
            name="ears_node",
            namespace="olaf",
            parameters=[{
                "config_path": "",  # Empty = use default config path
            }],
            output="screen",
        ),
    ])
