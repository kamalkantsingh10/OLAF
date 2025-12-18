from setuptools import setup

package_name = 'base_driver'

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
    description='ROS2 driver node for OLAF Base module (I2C 0x0B)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver_node = base_driver.base_driver_node:main',
        ],
    },
)
