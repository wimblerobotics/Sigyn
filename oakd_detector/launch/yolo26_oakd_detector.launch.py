#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_share_dir = get_package_share_directory('oakd_detector')
    
    # Declare launch arguments
    mx_id_arg = DeclareLaunchArgument(
        'mxId',
        default_value='14442C1051B665D700',
        description='MX ID of the OAK-D device'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_share_dir, 'resources', 'best.pt'),
        description='Path to YOLO26 model file (RoboFlow v3 model)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for detections'
    )
    
    image_size_arg = DeclareLaunchArgument(
        'image_size',
        default_value='640',
        description='Input image size for YOLO26'
    )
    
    trained_images_dir_arg = DeclareLaunchArgument(
        'trained_images_dir',
        default_value='/home/ros/sigyn_ws/src/Sigyn/trained_images',
        description='Directory to save captured images'
    )
    
    # Create the node
    yolo26_oakd_node = Node(
        package='oakd_detector',
        executable='yolo26_oakd_detector_node',
        name='yolo26_oakd_detector_node',
        output='screen',
        parameters=[{
            'mxId': LaunchConfiguration('mxId'),
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'image_size': LaunchConfiguration('image_size'),
            'trained_images_dir': LaunchConfiguration('trained_images_dir'),
        }]
    )
    
    return LaunchDescription([
        mx_id_arg,
        model_path_arg,
        confidence_threshold_arg,
        image_size_arg,
        trained_images_dir_arg,
        yolo26_oakd_node
    ])
