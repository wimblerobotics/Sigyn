# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import os
from setuptools import find_packages, setup
from glob import glob
from setuptools.command.install import install
import distutils.log
import sys

package_name = 'oakd_detector'
venv_source_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'venv')

class PostInstallCommand(install):
    def run(self):
        super().run()
        
        target_install_dir = os.path.join(self.install_lib, package_name)
        venv_symlink_target = os.path.join(target_install_dir, 'venv')

        os.makedirs(target_install_dir, exist_ok=True)

        if os.path.exists(venv_source_path):
            if not os.path.exists(venv_symlink_target):
                if os.path.isdir(venv_source_path):
                    os.symlink(venv_source_path, venv_symlink_target, target_is_directory=True)
                    self.announce(f"Created symlink from {venv_source_path} to {venv_symlink_target}", level=distutils.log.INFO)
                else:
                    self.announce(f"Source venv path {venv_source_path} is not a directory. Symlink not created.", level=distutils.log.WARN)
            else:
                self.announce(f"Symlink {venv_symlink_target} already exists.", level=distutils.log.INFO)
        else:
            self.announce(f"Source venv path {venv_source_path} does not exist. Symlink not created.", level=distutils.log.WARN)

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'resources'), glob(os.path.join('resources', '*.pt'))),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'opencv-python',
        'cv_bridge',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'tf_transformations',
        'numpy<2'
    ],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='OAK-D OIV7 Detector Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo26_oakd_detector_node = oakd_detector.yolo26_oakd_detector_node:main',
        ],
    },
    cmdclass={
        'install': PostInstallCommand,
    },
)
