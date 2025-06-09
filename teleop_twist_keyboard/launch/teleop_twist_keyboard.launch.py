import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # NOTE: This teleop node requires direct terminal access for keyboard input.
    # When using this launch file, the node may fail with a terminal I/O error.
    # For proper keyboard teleop functionality, run directly with:
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    
    # Declare launch arguments
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='0.2',
        description='Linear speed (m/s)'
    )
    
    turn_arg = DeclareLaunchArgument(
        'turn',
        default_value='0.35',
        description='Angular speed (rad/s)'
    )
    
    repeat_rate_arg = DeclareLaunchArgument(
        'repeat_rate',
        default_value='0.0',
        description='Repeat rate for publishing commands'
    )
    
    key_timeout_arg = DeclareLaunchArgument(
        'key_timeout',
        default_value='0.5',
        description='Timeout for key input'
    )
    
    stamped_arg = DeclareLaunchArgument(
        'stamped',
        default_value='false',
        description='Use TwistStamped messages instead of Twist'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='',
        description='Frame ID for stamped messages'
    )

    # Define the teleop node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{
            'speed': LaunchConfiguration('speed'),
            'turn': LaunchConfiguration('turn'),
            'repeat_rate': LaunchConfiguration('repeat_rate'),
            'key_timeout': LaunchConfiguration('key_timeout'),
            'stamped': LaunchConfiguration('stamped'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(speed_arg)
    ld.add_action(turn_arg)
    ld.add_action(repeat_rate_arg)
    ld.add_action(key_timeout_arg)
    ld.add_action(stamped_arg)
    ld.add_action(frame_id_arg)

    # Add the teleop node
    ld.add_action(teleop_node)

    return ld
