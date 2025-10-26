# Options:
# bt_xml = Full path to behavior tree overriding default_nav_to_pose_bt_xml in the navigation yaml file.
# do_joint_state_gui (false) - Flag to enable joint_state_publisher_gui.
# do_rviz (true) - Launch RViz if true.
# make_map (false) - Make a map vs navigate.
# urdf_file_name (sigyn.urdf.xacro) - URDF file name.
# use_sim_time (true) - Use simulation vs a real robot.
# world (home.world) - World to load if simulating.

import os
import platform
import xacro

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    articubot_package_name='articubot_one' #<--- CHANGE ME
    robot_model='stingray'
    articubot_package_path = get_package_share_directory(articubot_package_name)
    robot_path = os.path.join(articubot_package_path, 'robots', robot_model)

    make_map = LaunchConfiguration("make_map")
    make_map_arg = DeclareLaunchArgument(
        "make_map", default_value="False", description="Make a map vs navigate"
    )
    ld.add_action(make_map_arg)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Simulation mode vs real robot",
    )
    ld.add_action(use_sim_time_arg)


    log_info_action = LogInfo(
        msg=[
            "do_joint_state_gui: [",
            do_joint_state_gui,
            "], do_rviz: [",
            do_rviz,
            "], make_map: [",
            make_map,
            "], urdf_file_name: [",
            urdf_file_name,
            "], use_sim_time: [",
            use_sim_time,
            "], world: [",
            world,
            "]",
        ]
    )
    ld.add_action(log_info_action)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(package_path,'launch','rsp.launch.py')]
                ), launch_arguments={'use_sim_time': use_sim_time, 'robot_model' : robot_model}.items()
    )
    ld.add_action(rsp)


    # Note: controller_manager is provided by Gazebo's gz_ros2_control plugin
    # No need for separate ros2_control_node in simulation
    controller_params_file = PathJoinSubstitution(
        [description_pkg, robot_path, "config", "my_controllers.yaml"])

    # Add delays to ensure Gazebo's controller manager is ready
    delayed_joint_broad_spawner = TimerAction(
        period=3.0,  # Wait for Gazebo controller manager
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(use_sim_time),
            arguments=["joint_broad"],
        )]
    )
    ld.add_action(delayed_joint_broad_spawner)
    
    delayed_fwcommand_spawner = TimerAction(
        period=4.0,  # Wait after joint broadcaster
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(use_sim_time),
            arguments=["forward_position_controller", "--param-file", controller_params_file],
        )]
    )
    ld.add_action(delayed_fwcommand_spawner)

    delayed_diff_drive_spawner = TimerAction(
        period=5.0,  # Wait after other controllers
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(use_sim_time),
            arguments=["diff_cont"],
        )]
    )
    ld.add_action(delayed_diff_drive_spawner)

    # Bring of the EKF node.
    ekf_config_path = os.path.join(
        robot_path, "config", "ekf_odom_params.yaml"
    )
    start_robot_localization_cmd = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        condition=UnlessCondition(use_sim_time),
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ekf_config_path,
        ],
        remappings=[("odometry/filtered", "odometry/local")],
!!!        remappings=[("/odometry/filtered", "odom"), ("/odom/unfiltered", "/sigyn/wheel_odom")],
    )
    ld.add_action(start_robot_localization_cmd)

    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        respawn=True,
        respawn_delay=10,
        parameters=[
          {'product_name': 'LDLiDAR_LD14'},
          {'laser_scan_topic_name': 'scan'},
          {'point_cloud_2d_topic_name': 'pointcloud2d'},
          {'frame_id': 'laser_frame'},
          {'port_name': '/dev/ttyUSBLDR'},
          {'serial_baudrate' : 115200},
          {'laser_scan_dir': True},
          {'enable_angle_crop_func': False},
          {'angle_crop_min': 135.0},
          {'angle_crop_max': 225.0}
        ]
    )
    ld.add_action(ldlidar_node)

    mpu9250driver_node = Node(
        package="mpu9250",
        executable="mpu9250",
        name="mpu9250",
        output='screen',
        respawn=True,
        respawn_delay=4,
        emulate_tty=True,
        parameters=[
          {
              #"print" : True,
              "frequency" : 30,
              "i2c_address" : 0x68,
              "i2c_port" : 1,
              "frame_id" : "imu_link",
              "acceleration_scale": [1.0072387165748442, 1.0081436035838134, 0.9932769089604535],
              "acceleration_bias": [0.17038044467587418, 0.20464685207217453, -0.12461014438322202],
              "gyro_bias": [0.0069376404996494, -0.0619247665634732, 0.05717760948453845],
              "magnetometer_scale": [1.0, 1.0, 1.0],
              #"magnetometer_bias": [1.3345253592582676, 2.6689567513691685, -2.5294210260199957],
              #"magnetometer_bias": [1.335, 4.0, 1.0],
              #"magnetometer_bias": [1.335, 3.8, -2.5294210260199957],
              "magnetometer_bias": [1.335, 4.0, -2.53],
              "magnetometer_transform": [1.0246518952703103, -0.0240401565528902, 0.0030740476998857395,
                                        -0.024040156552890175, 0.9926708357001245, 0.002288563295390304,
                                         0.0030740476998857356, 0.0022885632953903268, 0.9837206150979054]
          }
        ],
        remappings=[("imu", "imu/data")]
    )

    # Bring up the twist multiplexer.
    multiplexer_directory_path = get_package_share_directory("twist_multiplexer")
    multiplexer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [multiplexer_directory_path, "/launch/twist_multiplexer.launch.py"]
        ),
        condition=UnlessCondition(use_sim_time),
    )
    ld.add_action(multiplexer_launch)

    map_path = os.path.join(articubot_package_path, "assets", "maps", "empty_map.yaml")
    
    nav2_config_path = os.path.join(robot_path, 'config', 'nav2_params.yaml')

    # Bring up the navigation stack.
    navigation_launch_path = PathJoinSubstitution(
        [robot_path, "launch", "nav2_bringup.launch.py"]
    )
    !!!

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            "autostart": "True",
            "map": map_path,
            "params_file": nav2_config_path,  # Use original config file
            "slam": "False",
            "use_composition": "True",
            "use_respawn": "True",
            "use_sim_time": use_sim_time,
            "use_localization": "True",
            "container_name": "nav2_container",
        }.items(),
    )
    ld.add_action(nav2_launch)
    
    # Include the SLAM Toolbox launch file for mapping
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ],
        ),
        condition=IfCondition(make_map),
        launch_arguments={
            "use_lifecycle_manager": "False",
            "use_sim_time": use_sim_time,
            "slam_params_file": os.path.join(
                base_pgk, "config", "mapper_params_online_async.yaml"
            ),
            # "params_file": "/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml",
        }.items(),
    )
    ld.add_action(slam_toolbox)

    tf_localizer = Node(package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )
    !!!


    return ld
