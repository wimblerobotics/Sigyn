from setuptools import setup

package_name = 'sigyn_websocket'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets>=12.0', 'cbor2>=5.6.0'],
    zip_safe=True,
    maintainer='Wimble Robotics',
    maintainer_email='michael@wimblerobotics.com',
    description='ROS 2 → WebSocket bridge with CBOR for telemetry and control.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'server = sigyn_websocket.server:main',
        ],
    },
)
