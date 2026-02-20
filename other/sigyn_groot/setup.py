from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sigyn_groot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools', 'flask', 'flask-socketio', 'pyzmq'],
    zip_safe=True,
    maintainer='wimblerobotics',
    maintainer_email='wimblerobotics@gmail.com',
    description='Web-based behavior tree monitor for Sigyn',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_monitor = sigyn_groot.bt_monitor:main',
        ],
    },
)
