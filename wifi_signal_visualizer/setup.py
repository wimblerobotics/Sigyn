import os
from glob import glob
from setuptools import setup

package_name = 'wifi_signal_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='mike@wimblerobotics.com',
    description='rviz2 visualizer of wifi signal strength',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_signal_visualizer_node = wifi_signal_visualizer.wifi_signal_visualizer_node:main',
        ],
    },
)
