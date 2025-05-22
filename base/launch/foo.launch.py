from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # Define arguments
    declare_launch_argument = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to launch RViz."
    )
    declare_launch_argument_confidence_threshold = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.5",
        description="Minimum confidence score for detections."
    )

    # Launch the DepthAI ROS driver (example, adjust parameters as needed)
    depthai_ros_driver_node = Node(
        package="depthai_ros_driver",
        executable="depthai_ros_driver_node",
        name="depthai_driver",
        parameters=[{"device_name": "OAK-D"}], # Add necessary device parameters
        output="screen",
    )

    # Launch your object detection node (example, adjust parameters and node name)
    object_detection_node = Node(
        package="your_package",  # Replace with your package name
        executable="object_detection_node", # Replace with your node executable
        name="object_detection_node",
        parameters=[
            {"model_path": "/path/to/your/yolov4_mini.onnx"}, # Replace with your model path
            {"confidence_threshold": LaunchConfiguration("confidence_threshold")},
        ],
        output="screen",
    )

    # Launch RViz (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', '/path/to/your/rviz_config.rviz'], # Replace with your RViz config
        condition=LaunchConfiguration("use_rviz"),
        output="screen"
    )

    # Return the launch description
    return LaunchDescription(
        [
            declare_launch_argument,
            declare_launch_argument_confidence_threshold,
            depthai_ros_driver_node,
            object_detection_node,
            rviz_node,
        ]
    )
