from setuptools import setup
import os
from glob import glob

package_name = 'perimeter_roamer_v2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Add this line to install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'transforms3d',  # For tf_transformations compatibility
        'numpy',
    ],
    zip_safe=True,
    maintainer='Michael Wimble',
    maintainer_email='mike@wimblerobotics.com',
    description='Advanced perimeter roaming robot controller for house patrolling',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perimeter_roamer = perimeter_roamer_v2.perimeter_roamer:main',
        ],
    },
) 