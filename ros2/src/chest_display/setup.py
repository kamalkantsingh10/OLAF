from setuptools import setup
import os
from glob import glob

package_name = 'chest_display'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.views'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # systemd unit for boot-to-app (install/enable per README)
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*.service')),
    ],
    install_requires=[
        'setuptools',
        # Maintained pygame fork with KMSDRM support (imports as `pygame`).
        # Installed via the project poetry env on the Pi (`poetry add pygame-ce`).
        'pygame-ce>=2.4',
    ],
    zip_safe=True,
    maintainer='Kamal Kant Singh',
    maintainer_email='kamalkantsingh10@gmail.com',
    description='OLAF chest-display app — portrait 2x2 dashboard (animated heart '
                '+ log panels) on the 4.3" DSI panel. Epic 8.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chest_display = chest_display.app:main',
        ],
    },
)
