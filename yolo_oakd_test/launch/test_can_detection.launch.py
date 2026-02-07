from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = os.popen('ros2 pkg prefix yolo_oakd_test').read().strip() + '/share/yolo_oakd_test'
    
    # Path to the blob
    blob_path = os.path.join(pkg_share, 'models', 'can_detector.blob')
    
    # RViz config
    rviz_config = os.path.join(pkg_share, 'config', 'can_detection.rviz')


    # Include the robot description launch to get the TF tree
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('description'), 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false', 
            'publish_joints': 'false', # We don't need joint_state_publisher_gui
            'do_rviz': 'false'         # We will launch our own specific rviz
        }.items()
    )

    expected_target_base_arg = DeclareLaunchArgument(
        'expected_target_base',
        default_value='[0.65, 0.0, 0.6]',
        description='Expected can position in base_link frame for axis-map suggestion'
    )
    suggest_axis_map_arg = DeclareLaunchArgument(
        'suggest_axis_map',
        default_value='true',
        description='Enable axis-map suggestion logging'
    )

    return LaunchDescription([
        expected_target_base_arg,
        suggest_axis_map_arg,
        description_launch,
        
        # 2. The Custom Debug Node
        Node(
            package='yolo_oakd_test',
            executable='oakd_can_detector.py',
            name='oakd_can_detector',
            output='screen',
            parameters=[{
                'blob_path': blob_path,
                # Frame to publish camera points in before TF to base_link
                'camera_frame': 'oak_rgb_camera_optical_frame',
                # DepthAI spatial coords (x,y,z) -> camera_frame axis map
                # Suggested by live solver for current URDF mount orientation
                'spatial_axis_map': '-z,-x,y',
                'log_tf_debug': True,
                'expected_target_base': LaunchConfiguration('expected_target_base'),
                'suggest_axis_map': LaunchConfiguration('suggest_axis_map'),
            }]
        ),
        
        # # 3. Robot State Publisher (Replaced by description.launch.py)
        
        # 4. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
