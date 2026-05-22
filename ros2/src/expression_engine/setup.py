from setuptools import setup
import os
from glob import glob

package_name = 'expression_engine'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Renderer mapping + service config (populated in Epic 6)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Body-owned interface contract (Story 7.6) — spec + VERSION +
        # generated JSON Schemas. Installed so they resolve at runtime
        # and ship with the package. Kept self-contained for later
        # extraction into a neutral interface package (git mv).
        (os.path.join('share', package_name, 'contract'),
            glob('contract/*.md') + glob('contract/VERSION')),
        (os.path.join('share', package_name, 'contract', 'schemas'),
            glob('contract/schemas/*.json')),
    ],
    install_requires=[
        'setuptools',
        # Wire-schema validation (Story 6.1, AR13 — re-derived, NOT
        # importing olaf_companion). Pinned to pydantic v2 to match the
        # companion contract @ tag v3.0.0.
        'pydantic>=2,<3',
        # tomllib is stdlib on 3.11+; tomli only on the pinned 3.10 floor.
        'tomli;python_version<"3.11"',
    ],
    zip_safe=True,
    maintainer='Kamal Kant Singh',
    maintainer_email='kamalkantsingh10@gmail.com',
    description='OLAF expression engine — renders the companion pipeline\'s '
                '4 canonical topics onto the body. See the Phase 2 SCP.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'expression_engine_node = expression_engine.node:main',
        ],
    },
)
