from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
import os

multiplexer_directory_path = get_package_share_directory('twist_multiplexer');
base_directory_path = get_package_share_directory('base')
description_directory_path = get_package_share_directory('description')
urdf_path = os.path.join(description_directory_path, 'urdf/base.urdf.xacro')
use_sim_time = LaunchConfiguration('use_sim_time')
ekf_config_path = os.path.join(base_directory_path, 'config/ekf.yaml')

ld = LaunchDescription()

ld.add_action(DeclareLaunchArgument(
    name= 'map', 
    default_value='map_20240306.yaml',
    description='map file to use for navigation'))

ld.add_action(DeclareLaunchArgument(
    name='use_sim_time', 
    default_value='False',
    description='Use simulation (Gazebo) clock if true'))

map = LaunchConfiguration('map')
map_path = os.path.join(base_directory_path, 'maps', 'map_20240306.yaml')
