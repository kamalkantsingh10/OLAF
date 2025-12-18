from setuptools import setup

package_name = 'torso_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kamal Kant Singh',
    maintainer_email='kamalkantsingh10@gmail.com',
    description='ROS2 driver node for OLAF Torso module (I2C 0x0A)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'torso_driver_node = torso_driver.torso_driver_node:main',
        ],
    },
)
