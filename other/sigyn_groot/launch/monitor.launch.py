#!/usr/bin/env python3
"""Launch file for Sigyn Groot BT Monitor."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description."""
    
    # Declare arguments
    zmq_address_arg = DeclareLaunchArgument(
        'zmq_address',
        default_value='localhost',
        description='ZMQ server address (where BT publisher is running)'
    )
    
    zmq_port_arg = DeclareLaunchArgument(
        'zmq_port',
        default_value='1666',
        description='ZMQ server port'
    )
    
    # BT Monitor node
    bt_monitor_node = Node(
        package='sigyn_groot',
        executable='bt_monitor',
        name='bt_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'zmq_server_address': LaunchConfiguration('zmq_address'),
            'zmq_server_port': LaunchConfiguration('zmq_port'),
        }]
    )
    
    return LaunchDescription([
        zmq_address_arg,
        zmq_port_arg,
        bt_monitor_node,
    ])
