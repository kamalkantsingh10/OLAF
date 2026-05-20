from setuptools import setup

package_name = 'head_ears_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'scservo_sdk'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kamal Kant Singh',
    maintainer_email='kamalkantsingh10@gmail.com',
    description='OLAF Head+Ears hardware logic (I2C 0x08). Servo/I2C drivers '
               'and expression presets, imported in-process by the expression '
               'engine. ROS-node wrappers archived 2026-05-15 (Phase 2 SCP).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
