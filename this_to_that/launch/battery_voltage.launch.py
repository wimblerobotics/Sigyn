import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    this_to_that_share_dir = get_package_share_directory('this_to_that')

    # Construct the full path to the config file
    config_file = os.path.join(
        this_to_that_share_dir,
        'config',
        'battery_voltage.yaml'
    )

    # Define the node action
    field_mapper_node = Node(
        package='this_to_that',
        executable='this_to_that_node.py',
        name='batter_voltage_overlay', # Must match the node name in config.yaml and the code
        output='screen',
        parameters=[config_file] # Pass the config file path
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the node action to the launch description
    ld.add_action(field_mapper_node)

    return ld