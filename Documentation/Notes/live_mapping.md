[INFO] [launch]: All log files can be found below /home/ros/.ros/log/2024-12-10-11-33-34-710621-sigyn7900-3686
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: do_joint_state_gui: [False], do_rviz: [true], make_map: [true], use_sim_time: [false], world: [/home/ros/sigyn_ws/install/description/share/description/worlds/home.world]
[OpaqueFunction] file_name: sigyn.urdf.xacro
[OpaqueFunction] xacro_file_path: /home/ros/sigyn_ws/install/description/share/description/urdf/sigyn.urdf.xacro
[OpaqueFunction] use_sim_time: false
[WARNING] [lifelong_slam_toolbox_node-3]: there are now at least 2 nodes with the name /slam_toolbox created within this launch context
[INFO] [twist_multiplexer-5]: process started with pid [3706]
[INFO] [robot_state_publisher-1]: process started with pid [3702]
[INFO] [async_slam_toolbox_node-2]: process started with pid [3703]
[INFO] [lifelong_slam_toolbox_node-3]: process started with pid [3704]
[INFO] [echo-4]: process started with pid [3705]
[INFO] [echo-4]: process has finished cleanly [pid 3705]
[INFO] [ldlidar-6]: process started with pid [3707]
[INFO] [scan_to_scan_filter_chain-7]: process started with pid [3708]
[INFO] [ekf_node-8]: process started with pid [3709]
[INFO] [joint_state_publisher-9]: process started with pid [3710]
[INFO] [rviz2-10]: process started with pid [3711]
[echo-4] [sim] Rviz config file path: /home/ros/sigyn_ws/install/rviz/share/rviz/config/config.rviz
[ldlidar-6] [INFO] [1733859215.020235265] [rclcpp]: Using port /dev/lidar_front_center
[ldlidar-6] [INFO] [1733859215.021910218] [rclcpp]: LiDAR_LD06 started successfully
[robot_state_publisher-1] [INFO] [1733859215.025895685] [robot_state_publisher]: Robot initialized
[async_slam_toolbox_node-2] [INFO] [1733859215.046561603] [slam_toolbox]: Node using stack size 40000000
[lifelong_slam_toolbox_node-3] [INFO] [1733859215.046562505] [slam_toolbox]: Node using stack size 40000000
[lifelong_slam_toolbox_node-3] [WARN] [1733859215.046694530] [slam_toolbox]: Lifelong mapping mode in SLAM Toolbox is considered experimental and should be understood before proceeding. Please visit: https://github.com/SteveMacenski/slam_toolbox/wiki/Experimental-Lifelong-Mapping-Node for more information.
[lifelong_slam_toolbox_node-3] [INFO] [1733859215.239647718] [slam_toolbox]: Configuring
[async_slam_toolbox_node-2] [INFO] [1733859215.239676311] [slam_toolbox]: Configuring
[lifelong_slam_toolbox_node-3] [INFO] [1733859215.247224750] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[async_slam_toolbox_node-2] [INFO] [1733859215.247248013] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[lifelong_slam_toolbox_node-3] [INFO] [1733859215.247412899] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
[async_slam_toolbox_node-2] [INFO] [1733859215.247428288] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
[INFO] [launch.user]: [LifecycleLaunch] Slamtoolbox node is activating.
[ERROR] [launch_ros.actions.lifecycle_node]: Failed to make transition 'TRANSITION_CONFIGURE' for LifecycleNode '/slam_toolbox'
[INFO] [launch.user]: [LifecycleLaunch] Slamtoolbox node is activating.
[async_slam_toolbox_node-2] [WARN] [1733859215.260567507] [rcl_lifecycle]: No transition matching 1 found for current state inactive
[async_slam_toolbox_node-2] [ERROR] [1733859215.260578869] [slam_toolbox]: Unable to start transition 1 from current state inactive: Transition is not registered., at ./src/rcl_lifecycle.c:355
[INFO] [launch.user]: [LifecycleLaunch] Slamtoolbox node is activating.
[INFO] [launch.user]: [LifecycleLaunch] Slamtoolbox node is activating.
[lifelong_slam_toolbox_node-3] [WARN] [1733859215.260772238] [rcl_lifecycle]: No transition matching 1 found for current state inactive
[lifelong_slam_toolbox_node-3] [ERROR] [1733859215.260783349] [slam_toolbox]: Unable to start transition 1 from current state inactive: Transition is not registered., at ./src/rcl_lifecycle.c:355
[lifelong_slam_toolbox_node-3] [INFO] [1733859215.263014485] [slam_toolbox]: Activating
[async_slam_toolbox_node-2] [INFO] [1733859215.263023802] [slam_toolbox]: Activating
[lifelong_slam_toolbox_node-3] [WARN] [1733859215.266313266] [rcl_lifecycle]: No transition matching 3 found for current state active
[lifelong_slam_toolbox_node-3] [ERROR] [1733859215.266321141] [slam_toolbox]: Unable to start transition 3 from current state active: Transition is not registered., at ./src/rcl_lifecycle.c:355
[async_slam_toolbox_node-2] [WARN] [1733859215.266363930] [rcl_lifecycle]: No transition matching 3 found for current state active
[async_slam_toolbox_node-2] [ERROR] [1733859215.266372236] [slam_toolbox]: Unable to start transition 3 from current state active: Transition is not registered., at ./src/rcl_lifecycle.c:355
[async_slam_toolbox_node-2] [WARN] [1733859215.266434902] [rcl_lifecycle]: No transition matching 3 found for current state active
[async_slam_toolbox_node-2] [ERROR] [1733859215.266440322] [slam_toolbox]: Unable to start transition 3 from current state active: Transition is not registered., at ./src/rcl_lifecycle.c:355
[async_slam_toolbox_node-2] [WARN] [1733859215.266468495] [rcl_lifecycle]: No transition matching 3 found for current state active
[ERROR] [launch_ros.actions.lifecycle_node]: Failed to make transition 'TRANSITION_ACTIVATE' for LifecycleNode '/slam_toolbox'
[ERROR] [launch_ros.actions.lifecycle_node]: Failed to make transition 'TRANSITION_ACTIVATE' for LifecycleNode '/slam_toolbox'
[ERROR] [launch_ros.actions.lifecycle_node]: Failed to make transition 'TRANSITION_ACTIVATE' for LifecycleNode '/slam_toolbox'
[lifelong_slam_toolbox_node-3] [WARN] [1733859215.266368930] [rcl_lifecycle]: No transition matching 3 found for current state active
[lifelong_slam_toolbox_node-3] [ERROR] [1733859215.266373368] [slam_toolbox]: Unable to start transition 3 from current state active: Transition is not registered., at ./src/rcl_lifecycle.c:355
[lifelong_slam_toolbox_node-3] [WARN] [1733859215.266403233] [rcl_lifecycle]: No transition matching 3 found for current state active
[lifelong_slam_toolbox_node-3] [ERROR] [1733859215.266405878] [slam_toolbox]: Unable to start transition 3 from current state active: Transition is not registered., at ./src/rcl_lifecycle.c:355
[async_slam_toolbox_node-2] [ERROR] [1733859215.266471150] [slam_toolbox]: Unable to start transition 3 from current state active: Transition is not registered., at ./src/rcl_lifecycle.c:355
[joint_state_publisher-9] [INFO] [1733859215.273290483] [joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[lifelong_slam_toolbox_node-3] [INFO] [1733859215.423386800] [slam_toolbox]: Message Filter dropping message: frame 'lidar_frame_top_lidar' at time 1733859215.246 for reason 'discarding message because the queue is full'
[lifelong_slam_toolbox_node-3] [WARN] [1733859215.423534796] [slam_toolbox]: maximum laser range setting (20.0 m) exceeds the capabilities of the used Lidar (15.0 m)
[lifelong_slam_toolbox_node-3] Registering sensor: [Custom Described Lidar]
[async_slam_toolbox_node-2] [WARN] [1733859216.123514240] [slam_toolbox]: maximum laser range setting (20.0 m) exceeds the capabilities of the used Lidar (15.0 m)
[async_slam_toolbox_node-2] Registering sensor: [Custom Described Lidar]
[rviz2-10] [INFO] [1733859218.605160414] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-10] [INFO] [1733859218.605236526] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-10] [INFO] [1733859218.864126486] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-10] [WARN] [1733859219.001677668] [rcl.logging_rosout]: Publisher already registered for node name: 'rviz2'. If this is due to multiple nodes with the same name then all logs for the logger named 'rviz2' will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-10] [WARN] [1733859219.011338182] [rcl.logging_rosout]: Publisher already registered for node name: 'rviz2'. If this is due to multiple nodes with the same name then all logs for the logger named 'rviz2' will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[rviz2-10] [INFO] [1733859219.019238195] [rviz2]: Start the slam_toolbox node state check.
[rviz2-10] [INFO] [1733859220.320567296] [rviz2]: Trying to create a map of size 61 x 118 using 1 swatches
[rviz2-10] [ERROR] [1733859220.353926004] [rviz2]: rviz/glsl120/indexed_8bit_image.vert
[rviz2-10] rviz/glsl120/indexed_8bit_image.frag
[rviz2-10]  GLSL link result : 
[rviz2-10] active samplers with a different type refer to the same texture image unit
