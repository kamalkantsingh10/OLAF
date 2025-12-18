from setuptools import setup

package_name = 'neck_driver'

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
    description='ROS2 driver node for OLAF Neck module (I2C 0x09)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neck_driver_node = neck_driver.neck_driver_node:main',
        ],
    },
)
