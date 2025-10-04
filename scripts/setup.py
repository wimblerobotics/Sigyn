# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

from setuptools import find_packages, setup

package_name = 'py_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Wimble',
    maintainer_email='mike@wimblerobotics.com',
    description='Sigyn scripts',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'measure_wifi = py_scripts.measure_wifi:main',
            'show_time_skew = py_scripts.show_time_skew:main',
            'topic_analysis = py_scripts.topic_analysis:main',
        ],
    },
)
