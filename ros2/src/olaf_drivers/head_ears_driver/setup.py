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
    description='ROS2 driver node for OLAF Head+Ears module (I2C 0x08)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_ears_driver_node = head_ears_driver.head_ears_driver_node:main',
            'ears_node = head_ears_driver.ears_node:main',
        ],
    },
)
