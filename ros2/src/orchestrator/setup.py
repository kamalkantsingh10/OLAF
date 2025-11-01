from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.1.0',
    packages=['orchestrator', 'orchestrator.hardware_drivers', 'orchestrator.ros2_nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.env')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OLAF Team',
    maintainer_email='your-email@example.com',
    description='OLAF Orchestrator - ROS2 nodes for personality coordination, AI integration, and navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Minimal coordinator for Epic 1 validation
            'minimal_coordinator = orchestrator.ros2_nodes.minimal_coordinator:main',

            # Hardware driver nodes (only head_driver exists currently)
            'head_driver = orchestrator.hardware_drivers.head_driver:main',
            # TODO: Add other nodes as they're implemented
            # 'ears_neck_driver = orchestrator.hardware_drivers.ears_neck_driver:main',
            # 'body_driver = orchestrator.hardware_drivers.body_driver:main',
            # 'base_driver = orchestrator.hardware_drivers.base_driver:main',

            # TODO: Personality nodes
            # 'personality_coordinator = orchestrator.personality.personality_coordinator:main',

            # TODO: AI integration nodes
            # 'ai_agent = orchestrator.ai_integration.ai_agent:main',
            # 'whisper_stt = orchestrator.ai_integration.whisper_stt:main',

            # TODO: Navigation nodes
            # 'cartographer = orchestrator.navigation.cartographer:main',
            # 'nav2_integration = orchestrator.navigation.nav2_integration:main',
        ],
    },
)
