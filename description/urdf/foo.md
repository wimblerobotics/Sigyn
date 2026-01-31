[INFO] [launch]: All log files can be found below /home/ros/.ros/log/2026-01-31-10-43-53-602354-sigyn7900a-32327
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: on_a_mac: False
[INFO] [launch.user]: do_joint_state_gui: [False], do_rviz: [true], make_map: [false], urdf_file_name: [sigyn.urdf.xacro], use_sim_time: [true], world: [/home/ros/sigyn_ws/install/description/share/description/worlds/can_challenge.world]
[launch_robot_state_publisher] file_name: sigyn.urdf.xacro
[launch_robot_state_publisher] xacro_file_path: /home/ros/sigyn_ws/install/description/share/description/urdf/sigyn.urdf.xacro
[launch_robot_state_publisher] use_sim_time: true
[INFO] [robot_state_publisher-1]: process started with pid [32344]
[INFO] [gazebo-2]: process started with pid [32345]
[INFO] [create-3]: process started with pid [32346]
[INFO] [parameter_bridge-4]: process started with pid [32347]
[INFO] [image_bridge-5]: process started with pid [32349]
[INFO] [image_bridge-6]: process started with pid [32350]
[INFO] [component_container_isolated-7]: process started with pid [32351]
[INFO] [echo-8]: process started with pid [32352]
[INFO] [echo-8]: process has finished cleanly [pid 32352]
[INFO] [SaySomethingActionServer-9]: process started with pid [32353]
[INFO] [MoveAShortDistanceAheadActionServer-10]: process started with pid [32354]
[INFO] [battery_overlay_publisher-11]: process started with pid [32355]
[INFO] [rviz2-12]: process started with pid [32356]
[echo-8] [sim] Rviz config file path: /home/ros/sigyn_ws/install/rviz/share/rviz/config/config.rviz
[robot_state_publisher-1] [INFO] [1769885034.385489813] [robot_state_publisher]: Robot initialized
[create-3] [INFO] [1769885034.397998254] [ros_gz_sim]: Requesting list of world names.
[component_container_isolated-7] [INFO] [1769885034.452307928] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libmap_server_core.so
[component_container_isolated-7] [INFO] [1769885034.462246131] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_map_server::CostmapFilterInfoServer>
[component_container_isolated-7] [INFO] [1769885034.462269081] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_map_server::MapSaver>
[component_container_isolated-7] [INFO] [1769885034.462272577] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_map_server::MapServer>
[component_container_isolated-7] [INFO] [1769885034.462275341] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_map_server::MapServer>
[component_container_isolated-7] [INFO] [1769885034.465836883] [map_server]: 
[component_container_isolated-7] 	map_server lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.465861606] [map_server]: Creating
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/map_server' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.469823085] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libamcl_core.so
[component_container_isolated-7] [INFO] [1769885034.472230634] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_amcl::AmclNode>
[component_container_isolated-7] [INFO] [1769885034.472240090] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_amcl::AmclNode>
[component_container_isolated-7] [INFO] [1769885034.474797426] [amcl]: 
[component_container_isolated-7] 	amcl lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.475122384] [amcl]: Creating
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/amcl' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.476646614] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libnav2_lifecycle_manager_core.so
[component_container_isolated-7] [INFO] [1769885034.477216404] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-7] [INFO] [1769885034.477230017] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-7] [INFO] [1769885034.479824126] [lifecycle_manager_localization]: Creating
[component_container_isolated-7] [INFO] [1769885034.482309008] [lifecycle_manager_localization]: Creating and initializing lifecycle service clients
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/lifecycle_manager_localization' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.483134807] [lifecycle_manager_localization]: Starting managed nodes bringup...
[component_container_isolated-7] [INFO] [1769885034.483161393] [lifecycle_manager_localization]: Configuring map_server
[component_container_isolated-7] [INFO] [1769885034.483241010] [map_server]: Configuring
[component_container_isolated-7] [INFO] [1769885034.483274738] [map_io]: Loading yaml file: /home/ros/sigyn_ws/install/base/share/base/maps/my_map.yaml
[component_container_isolated-7] [INFO] [1769885034.483410321] [map_io]: resolution: 0.0261762
[component_container_isolated-7] [INFO] [1769885034.483416301] [map_io]: origin[0]: 0
[component_container_isolated-7] [INFO] [1769885034.483419006] [map_io]: origin[1]: 0
[component_container_isolated-7] [INFO] [1769885034.483420739] [map_io]: origin[2]: 0
[component_container_isolated-7] [INFO] [1769885034.483422662] [map_io]: free_thresh: 0.25
[component_container_isolated-7] [INFO] [1769885034.483424505] [map_io]: occupied_thresh: 0.65
[component_container_isolated-7] [INFO] [1769885034.483427691] [map_io]: mode: trinary
[component_container_isolated-7] [INFO] [1769885034.483429955] [map_io]: negate: 0
[component_container_isolated-7] [INFO] [1769885034.483532581] [map_io]: Loading image_file: /home/ros/sigyn_ws/install/base/share/base/maps/my_map2.pgm
[component_container_isolated-7] [INFO] [1769885034.487332723] [map_io]: Read map /home/ros/sigyn_ws/install/base/share/base/maps/my_map2.pgm: 573 X 589 map @ 0.0261762 m/cell
[component_container_isolated-7] [INFO] [1769885034.488408713] [lifecycle_manager_localization]: Configuring amcl
[component_container_isolated-7] [INFO] [1769885034.488457957] [amcl]: Configuring
[component_container_isolated-7] [INFO] [1769885034.488501522] [amcl]: initTransforms
[component_container_isolated-7] [INFO] [1769885034.491444130] [amcl]: initPubSub
[component_container_isolated-7] [INFO] [1769885034.492082558] [amcl]: Subscribed to map topic.
[component_container_isolated-7] [INFO] [1769885034.493014940] [lifecycle_manager_localization]: Activating map_server
[component_container_isolated-7] [INFO] [1769885034.493058996] [map_server]: Activating
[component_container_isolated-7] [INFO] [1769885034.493344036] [map_server]: Creating bond (map_server) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885034.493506045] [amcl]: Received a 573 X 589 map @ 0.026 m/pix
[component_container_isolated-7] [INFO] [1769885034.594311271] [lifecycle_manager_localization]: Server map_server connected with bond.
[component_container_isolated-7] [INFO] [1769885034.594337867] [lifecycle_manager_localization]: Activating amcl
[component_container_isolated-7] [INFO] [1769885034.594429354] [amcl]: Activating
[component_container_isolated-7] [INFO] [1769885034.594466198] [amcl]: initialPoseReceived
[component_container_isolated-7] [INFO] [1769885034.594545584] [amcl]: Setting pose (0.000000): 11.100 7.500 0.000
[component_container_isolated-7] [INFO] [1769885034.595248232] [amcl]: Creating bond (amcl) to lifecycle manager.
[battery_overlay_publisher-11] [INFO] [1769885034.601183494] [battery_overlay_publisher]: Battery overlay publisher started
[battery_overlay_publisher-11] [INFO] [1769885034.601396440] [battery_overlay_publisher]:   Battery topic: /sigyn/teensy_bridge/battery/status
[battery_overlay_publisher-11] [INFO] [1769885034.601579034] [battery_overlay_publisher]:   Overlay topic: /battery_overlay_text
[battery_overlay_publisher-11] [INFO] [1769885034.601760747] [battery_overlay_publisher]:   Filter battery ID: 36VLIPO
[component_container_isolated-7] [INFO] [1769885034.614219562] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libcontroller_server_core.so
[component_container_isolated-7] [INFO] [1769885034.616124737] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_controller::ControllerServer>
[component_container_isolated-7] [INFO] [1769885034.616134884] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_controller::ControllerServer>
[component_container_isolated-7] [INFO] [1769885034.619562987] [controller_server]: 
[component_container_isolated-7] 	controller_server lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.623185184] [controller_server]: Creating controller server
[component_container_isolated-7] [INFO] [1769885034.626537055] [local_costmap.local_costmap]: 
[component_container_isolated-7] 	local_costmap lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.626894930] [local_costmap.local_costmap]: Creating Costmap
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/controller_server' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.628295688] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libsmoother_server_core.so
[component_container_isolated-7] [INFO] [1769885034.629422084] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_smoother::SmootherServer>
[component_container_isolated-7] [INFO] [1769885034.629429787] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_smoother::SmootherServer>
[component_container_isolated-7] [INFO] [1769885034.632304048] [smoother_server]: 
[component_container_isolated-7] 	smoother_server lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.633580442] [smoother_server]: Creating smoother server
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/smoother_server' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.634497769] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libplanner_server_core.so
[component_container_isolated-7] [INFO] [1769885034.634992730] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_planner::PlannerServer>
[component_container_isolated-7] [INFO] [1769885034.635000283] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_planner::PlannerServer>
[component_container_isolated-7] [INFO] [1769885034.637915884] [planner_server]: 
[component_container_isolated-7] 	planner_server lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.639156066] [planner_server]: Creating
[component_container_isolated-7] [INFO] [1769885034.642313333] [global_costmap.global_costmap]: 
[component_container_isolated-7] 	global_costmap lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.642607078] [global_costmap.global_costmap]: Creating Costmap
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/planner_server' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.644111024] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libbehavior_server_core.so
[component_container_isolated-7] [INFO] [1769885034.644646304] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<behavior_server::BehaviorServer>
[component_container_isolated-7] [INFO] [1769885034.644657122] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<behavior_server::BehaviorServer>
[component_container_isolated-7] [INFO] [1769885034.647615077] [behavior_server]: 
[component_container_isolated-7] 	behavior_server lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/behavior_server' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.650021403] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libbt_navigator_core.so
[component_container_isolated-7] [INFO] [1769885034.652152747] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_bt_navigator::BtNavigator>
[component_container_isolated-7] [INFO] [1769885034.652162684] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_bt_navigator::BtNavigator>
[component_container_isolated-7] [INFO] [1769885034.655735105] [bt_navigator]: 
[component_container_isolated-7] 	bt_navigator lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.656992265] [bt_navigator]: Creating
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/bt_navigator' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.657727459] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libwaypoint_follower_core.so
[component_container_isolated-7] [INFO] [1769885034.658349799] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_waypoint_follower::WaypointFollower>
[component_container_isolated-7] [INFO] [1769885034.658356671] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_waypoint_follower::WaypointFollower>
[component_container_isolated-7] [INFO] [1769885034.661645825] [waypoint_follower]: 
[component_container_isolated-7] 	waypoint_follower lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[component_container_isolated-7] [INFO] [1769885034.661964723] [waypoint_follower]: Creating
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/waypoint_follower' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.662919284] [nav2_container]: Load Library: /opt/ros/jazzy/lib/libvelocity_smoother_core.so
[component_container_isolated-7] [INFO] [1769885034.663395041] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_velocity_smoother::VelocitySmoother>
[component_container_isolated-7] [INFO] [1769885034.663401102] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_velocity_smoother::VelocitySmoother>
[component_container_isolated-7] [INFO] [1769885034.666516216] [velocity_smoother]: 
[component_container_isolated-7] 	velocity_smoother lifecycle node launched. 
[component_container_isolated-7] 	Waiting on external lifecycle transitions to activate
[component_container_isolated-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/velocity_smoother' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.667323564] [nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-7] [INFO] [1769885034.667332169] [nav2_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<nav2_lifecycle_manager::LifecycleManager>
[component_container_isolated-7] [INFO] [1769885034.669991059] [lifecycle_manager_navigation]: Creating
[component_container_isolated-7] [INFO] [1769885034.670699898] [lifecycle_manager_navigation]: Creating and initializing lifecycle service clients
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/lifecycle_manager_navigation' in container '/nav2_container'
[component_container_isolated-7] [INFO] [1769885034.673367814] [lifecycle_manager_navigation]: Starting managed nodes bringup...
[component_container_isolated-7] [INFO] [1769885034.673386967] [lifecycle_manager_navigation]: Configuring controller_server
[component_container_isolated-7] [INFO] [1769885034.673474357] [controller_server]: Configuring controller interface
[component_container_isolated-7] [INFO] [1769885034.673489343] [controller_server]: getting progress checker plugins..
[component_container_isolated-7] [INFO] [1769885034.673555506] [controller_server]: getting goal checker plugins..
[component_container_isolated-7] [INFO] [1769885034.673600484] [controller_server]: Controller frequency set to 20.0000Hz
[component_container_isolated-7] [INFO] [1769885034.673619496] [local_costmap.local_costmap]: Configuring
[component_container_isolated-7] [INFO] [1769885034.675025814] [local_costmap.local_costmap]: Using plugin "voxel_layer"
[component_container_isolated-7] [INFO] [1769885034.676767067] [local_costmap.local_costmap]: Subscribed to Topics: scan_low
[component_container_isolated-7] [INFO] [1769885034.678106579] [local_costmap.local_costmap]: Initialized plugin "voxel_layer"
[component_container_isolated-7] [INFO] [1769885034.678115455] [local_costmap.local_costmap]: Using plugin "oakd_top_layer"
[component_container_isolated-7] [INFO] [1769885034.678396127] [local_costmap.local_costmap]: Subscribed to Topics: oakd_top
[component_container_isolated-7] [WARN] [1769885034.679141859] [local_costmap.local_costmap]: obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.
[component_container_isolated-7] [INFO] [1769885034.679153819] [local_costmap.local_costmap]: Initialized plugin "oakd_top_layer"
[component_container_isolated-7] [INFO] [1769885034.679157055] [local_costmap.local_costmap]: Using plugin "range_sensor_layer"
[component_container_isolated-7] [INFO] [1769885034.679591802] [local_costmap.local_costmap]: range_sensor_layer: ALL as input_sensor_type given
[component_container_isolated-7] [INFO] [1769885034.679769177] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sigyn/teensy_bridge/range/vl53l0x_0
[component_container_isolated-7] [INFO] [1769885034.679892469] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sigyn/teensy_bridge/range/vl53l0x_1
[component_container_isolated-7] [INFO] [1769885034.680014609] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sigyn/teensy_bridge/range/vl53l0x_2
[component_container_isolated-7] [INFO] [1769885034.680127743] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sigyn/teensy_bridge/range/vl53l0x_3
[component_container_isolated-7] [INFO] [1769885034.680235949] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sigyn/teensy_bridge/range/vl53l0x_4
[component_container_isolated-7] [INFO] [1769885034.680340940] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sigyn/teensy_bridge/range/vl53l0x_5
[component_container_isolated-7] [INFO] [1769885034.680442274] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sigyn/teensy_bridge/range/vl53l0x_6
[component_container_isolated-7] [INFO] [1769885034.680544380] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sigyn/teensy_bridge/range/vl53l0x_7
[component_container_isolated-7] [INFO] [1769885034.680551041] [local_costmap.local_costmap]: Initialized plugin "range_sensor_layer"
[component_container_isolated-7] [INFO] [1769885034.680554277] [local_costmap.local_costmap]: Using plugin "inflation_layer"
[component_container_isolated-7] [INFO] [1769885034.680837684] [local_costmap.local_costmap]: Initialized plugin "inflation_layer"
[component_container_isolated-7] [ERROR] [1769885034.684625275] [local_costmap.local_costmap]: The configured inflation radius (0.150) is smaller than the computed inscribed radius (0.284) of your footprint, it is highly recommended to set inflation radius to be at least as big as the inscribed radius to avoid collisions
[component_container_isolated-7] [INFO] [1769885034.685963655] [controller_server]: Created progress_checker : progress_checker of type nav2_controller::SimpleProgressChecker
[component_container_isolated-7] [INFO] [1769885034.686130833] [controller_server]: Controller Server has progress_checker  progress checkers available.
[component_container_isolated-7] [INFO] [1769885034.686440024] [controller_server]: Created goal checker : precise_goal_checker of type nav2_controller::SimpleGoalChecker
[component_container_isolated-7] [INFO] [1769885034.686616347] [controller_server]: Controller Server has precise_goal_checker  goal checkers available.
[component_container_isolated-7] [INFO] [1769885034.687406355] [controller_server]: Created controller : FollowPath of type nav2_mppi_controller::MPPIController
[component_container_isolated-7] [INFO] [1769885034.688381922] [controller_server]: Controller period is equal to model dt. Control sequence shifting is ON
[component_container_isolated-7] [INFO] [1769885034.689322479] [controller_server]: ConstraintCritic instantiated with 1 power and 4.000000 weight.
[component_container_isolated-7] [INFO] [1769885034.689337304] [controller_server]: Critic loaded : mppi::critics::ConstraintCritic
[component_container_isolated-7] [ERROR] [1769885034.689960826] [computeCircumscribedCost]: The inflation radius (0.150000) is smaller than the circumscribed radius (0.294142) If this is an SE2-collision checking plugin, it cannot use costmap potential field to speed up collision checking by only checking the full footprint when robot is within possibly-inscribed radius of an obstacle. This may significantly slow down planning times!
[component_container_isolated-7] [ERROR] [1769885034.689973027] [controller_server]: Inflation layer either not found or inflation is not set sufficiently for optimized non-circular collision checking capabilities. It is HIGHLY recommended to set the inflation radius to be at MINIMUM half of the robot's largest cross-section. See github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields for full instructions. This will substantially impact run-time performance.
[component_container_isolated-7] [INFO] [1769885034.689980470] [controller_server]: InflationCostCritic instantiated with 1 power and 300.000000 / 0.015000 weights. Critic will collision check based on circular cost.
[component_container_isolated-7] [INFO] [1769885034.689985278] [controller_server]: Critic loaded : mppi::critics::CostCritic
[component_container_isolated-7] [INFO] [1769885034.690234496] [controller_server]: GoalCritic instantiated with 1 power and 5.000000 weight.
[component_container_isolated-7] [INFO] [1769885034.690252447] [controller_server]: Critic loaded : mppi::critics::GoalCritic
[component_container_isolated-7] [INFO] [1769885034.690454164] [controller_server]: GoalAngleCritic instantiated with 1 power, 3.000000 weight, and 0.500000 angular threshold.
[component_container_isolated-7] [INFO] [1769885034.690462939] [controller_server]: Critic loaded : mppi::critics::GoalAngleCritic
[component_container_isolated-7] [INFO] [1769885034.690817118] [controller_server]: ReferenceTrajectoryCritic instantiated with 1 power and 14.000000 weight
[component_container_isolated-7] [INFO] [1769885034.690833205] [controller_server]: Critic loaded : mppi::critics::PathAlignCritic
[component_container_isolated-7] [INFO] [1769885034.691049197] [controller_server]: Critic loaded : mppi::critics::PathFollowCritic
[component_container_isolated-7] [INFO] [1769885034.691368826] [controller_server]: PathAngleCritic instantiated with 1 power and 2.000000 weight. Mode set to: Forward Preference
[component_container_isolated-7] [INFO] [1769885034.691379595] [controller_server]: Critic loaded : mppi::critics::PathAngleCritic
[component_container_isolated-7] [INFO] [1769885034.691556939] [controller_server]: PreferForwardCritic instantiated with 1 power and 5.000000 weight.
[component_container_isolated-7] [INFO] [1769885034.691567107] [controller_server]: Critic loaded : mppi::critics::PreferForwardCritic
[component_container_isolated-7] [INFO] [1769885034.693250821] [controller_server]: Optimizer reset
[component_container_isolated-7] [INFO] [1769885034.694463836] [MPPIController]: Configured MPPI Controller: FollowPath
[component_container_isolated-7] [INFO] [1769885034.694471710] [controller_server]: Controller Server has FollowPath  controllers available.
[component_container_isolated-7] [INFO] [1769885034.696100740] [lifecycle_manager_localization]: Server amcl connected with bond.
[component_container_isolated-7] [INFO] [1769885034.696124050] [lifecycle_manager_localization]: Managed nodes are active
[component_container_isolated-7] [INFO] [1769885034.696134157] [lifecycle_manager_localization]: Creating bond timer...
[component_container_isolated-7] [INFO] [1769885034.696364043] [lifecycle_manager_navigation]: Configuring smoother_server
[component_container_isolated-7] [INFO] [1769885034.696417074] [smoother_server]: Configuring smoother server
[component_container_isolated-7] [INFO] [1769885034.698303206] [smoother_server]: Created smoother : simple_smoother of type nav2_smoother::SimpleSmoother
[component_container_isolated-7] [INFO] [1769885034.698681486] [smoother_server]: Smoother Server has simple_smoother  smoothers available.
[component_container_isolated-7] [INFO] [1769885034.699982803] [lifecycle_manager_navigation]: Configuring planner_server
[component_container_isolated-7] [INFO] [1769885034.700031346] [planner_server]: Configuring
[component_container_isolated-7] [INFO] [1769885034.700055979] [global_costmap.global_costmap]: Configuring
[component_container_isolated-7] [INFO] [1769885034.701511070] [global_costmap.global_costmap]: Using plugin "static_layer"
[component_container_isolated-7] [INFO] [1769885034.702032206] [global_costmap.global_costmap]: Subscribing to the map topic (/map) with transient local durability
[component_container_isolated-7] [INFO] [1769885034.702206155] [global_costmap.global_costmap]: Initialized plugin "static_layer"
[component_container_isolated-7] [INFO] [1769885034.702219558] [global_costmap.global_costmap]: Using plugin "obstacle_layer"
[component_container_isolated-7] [INFO] [1769885034.702560844] [global_costmap.global_costmap]: Subscribed to Topics: scan
[component_container_isolated-7] [INFO] [1769885034.703509365] [global_costmap.global_costmap]: Initialized plugin "obstacle_layer"
[component_container_isolated-7] [INFO] [1769885034.703523389] [global_costmap.global_costmap]: Using plugin "inflation_layer"
[component_container_isolated-7] [INFO] [1769885034.703820309] [global_costmap.global_costmap]: Initialized plugin "inflation_layer"
[component_container_isolated-7] [INFO] [1769885034.707497891] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 573 X 589 at 0.026176 m/pix
[component_container_isolated-7] [INFO] [1769885034.707661953] [planner_server]: Created global planner plugin GridBased of type nav2_navfn_planner::NavfnPlanner
[component_container_isolated-7] [INFO] [1769885034.707677019] [planner_server]: Configuring plugin GridBased of type NavfnPlanner
[component_container_isolated-7] [INFO] [1769885034.708133574] [planner_server]: Planner Server has GridBased  planners available.
[component_container_isolated-7] [INFO] [1769885034.710274424] [lifecycle_manager_navigation]: Configuring behavior_server
[component_container_isolated-7] [INFO] [1769885034.710360572] [behavior_server]: Configuring
[component_container_isolated-7] [INFO] [1769885034.711291241] [behavior_server]: Creating behavior plugin spin of type nav2_behaviors::Spin
[component_container_isolated-7] [INFO] [1769885034.711864296] [behavior_server]: Creating behavior plugin backup of type nav2_behaviors::BackUp
[component_container_isolated-7] [INFO] [1769885034.712378991] [behavior_server]: Creating behavior plugin drive_on_heading of type nav2_behaviors::DriveOnHeading
[component_container_isolated-7] [INFO] [1769885034.712878990] [behavior_server]: Creating behavior plugin assisted_teleop of type nav2_behaviors::AssistedTeleop
[component_container_isolated-7] [INFO] [1769885034.713892663] [behavior_server]: Creating behavior plugin wait of type nav2_behaviors::Wait
[component_container_isolated-7] [INFO] [1769885034.714698128] [behavior_server]: Configuring spin
[component_container_isolated-7] [INFO] [1769885034.716151716] [behavior_server]: Configuring backup
[component_container_isolated-7] [INFO] [1769885034.717130739] [behavior_server]: Configuring drive_on_heading
[component_container_isolated-7] [INFO] [1769885034.718035905] [behavior_server]: Configuring assisted_teleop
[component_container_isolated-7] [INFO] [1769885034.719458591] [behavior_server]: Configuring wait
[component_container_isolated-7] [INFO] [1769885034.720775264] [lifecycle_manager_navigation]: Configuring velocity_smoother
[component_container_isolated-7] [INFO] [1769885034.720832122] [velocity_smoother]: Configuring velocity smoother
[component_container_isolated-7] [INFO] [1769885034.722172395] [lifecycle_manager_navigation]: Configuring bt_navigator
[component_container_isolated-7] [INFO] [1769885034.722219055] [bt_navigator]: Configuring
[component_container_isolated-7] [INFO] [1769885034.723153091] [bt_navigator]: Creating navigator id navigate_to_pose of type nav2_bt_navigator::NavigateToPoseNavigator
[component_container_isolated-7] [INFO] [1769885034.754428291] [bt_navigator]: Creating navigator id navigate_through_poses of type nav2_bt_navigator::NavigateThroughPosesNavigator
[component_container_isolated-7] [INFO] [1769885034.761532242] [lifecycle_manager_navigation]: Configuring waypoint_follower
[component_container_isolated-7] [INFO] [1769885034.761586415] [waypoint_follower]: Configuring
[component_container_isolated-7] [INFO] [1769885034.765597770] [waypoint_follower]: Created waypoint_task_executor : wait_at_waypoint of type nav2_waypoint_follower::WaitAtWaypoint
[component_container_isolated-7] [INFO] [1769885034.765982602] [lifecycle_manager_navigation]: Activating controller_server
[component_container_isolated-7] [INFO] [1769885034.766035763] [controller_server]: Activating
[component_container_isolated-7] [INFO] [1769885034.766052882] [local_costmap.local_costmap]: Activating
[component_container_isolated-7] [INFO] [1769885034.766058672] [local_costmap.local_costmap]: Checking transform
[component_container_isolated-7] [INFO] [1769885034.766070813] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[gazebo-2] [Dbg] [gz.cc:166] Subscribing to [/gazebo/starting_world].
[gazebo-2] [Dbg] [gz.cc:168] Waiting for a world to be set from the GUI...
[gazebo-2] [Msg] Received world [/home/ros/sigyn_ws/install/description/share/description/worlds/can_challenge.world] from the GUI.
[gazebo-2] [Dbg] [gz.cc:172] Unsubscribing from [/gazebo/starting_world].
[gazebo-2] [Msg] Gazebo Sim Server v8.10.0
[gazebo-2] [Msg] Loading SDF world file[/home/ros/sigyn_ws/install/description/share/description/worlds/can_challenge.world].
[gazebo-2] [Msg] Serving entity system service on [/entity/system/add]
[gazebo-2] [Dbg] [Physics.cc:899] Loaded [gz::physics::dartsim::Plugin] from library [/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins/libgz-physics-dartsim-plugin.so]
[gazebo-2] [Dbg] [SystemManager.cc:80] Loaded system [gz::sim::systems::Physics] for entity [1]
[gazebo-2] [Msg] Create service on [/world/can_challenge/create_multiple] (async)
[gazebo-2] [Msg] Create service on [/world/can_challenge/create_multiple/blocking] (blocking)
[gazebo-2] [Msg] Remove service on [/world/can_challenge/remove] (async)
[gazebo-2] [Msg] Remove service on [/world/can_challenge/remove/blocking] (blocking)
[gazebo-2] [Msg] Pose service on [/world/can_challenge/set_pose] (async)
[gazebo-2] [Msg] Pose service on [/world/can_challenge/set_pose/blocking] (blocking)
[gazebo-2] [Msg] Pose service on [/world/can_challenge/set_pose_vector] (async)
[gazebo-2] [Msg] Pose service on [/world/can_challenge/set_pose_vector/blocking] (blocking)
[gazebo-2] [Msg] Light configuration service on [/world/can_challenge/light_config] (async)
[gazebo-2] [Msg] Light configuration service on [/world/can_challenge/light_config/blocking] (blocking)
[gazebo-2] [Msg] Physics service on [/world/can_challenge/set_physics] (async)
[gazebo-2] [Msg] Physics service on [/world/can_challenge/set_physics/blocking] (blocking)
[gazebo-2] [Msg] SphericalCoordinates service on [/world/can_challenge/set_spherical_coordinates] (async)
[gazebo-2] [Msg] SphericalCoordinates service on [/world/can_challenge/set_spherical_coordinates/blocking] (blocking)
[gazebo-2] [Msg] Enable collision service on [/world/can_challenge/enable_collision] (async)
[gazebo-2] [Msg] Enable collision service on [/world/can_challenge/enable_collision/blocking] (blocking)
[gazebo-2] [Msg] Disable collision service on [/world/can_challenge/disable_collision] (async)
[gazebo-2] [Msg] Disable collision service on [/world/can_challenge/disable_collision/blocking] (blocking)
[gazebo-2] [Msg] Material service on [/world/can_challenge/visual_config] (async)
[create-3] [INFO] [1769885034.947757369] [ros_gz_sim]: Waiting messages on topic [robot_description].
[create-3] [INFO] [1769885034.949720804] [ros_gz_sim]: Entity creation successful.
[INFO] [create-3]: process has finished cleanly [pid 32346]
[component_container_isolated-7] [INFO] [1769885035.266153003] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[parameter_bridge-4] [INFO] [1769885035.409952986] [ros_gz_bridge]: Creating GZ->ROS Bridge: [clock (gz.msgs.Clock) -> clock (rosgraph_msgs/msg/Clock)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.410897069] [ros_gz_bridge]: Creating GZ->ROS Bridge: [scan (gz.msgs.LaserScan) -> scan (sensor_msgs/msg/LaserScan)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.412132653] [ros_gz_bridge]: Creating GZ->ROS Bridge: [scan_cup (gz.msgs.LaserScan) -> scan_cup (sensor_msgs/msg/LaserScan)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.412550842] [ros_gz_bridge]: Creating GZ->ROS Bridge: [odom (gz.msgs.Odometry) -> odom (nav_msgs/msg/Odometry)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.413287419] [ros_gz_bridge]: Creating GZ->ROS Bridge: [tf (gz.msgs.Pose_V) -> tf (tf2_msgs/msg/TFMessage)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.414196662] [ros_gz_bridge]: Creating ROS->GZ Bridge: [cmd_vel_smoothed (geometry_msgs/msg/Twist) -> cmd_vel (gz.msgs.Twist)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.414691022] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/gripper_elevator_plate_to_gripper_extender/position (std_msgs/msg/Float64) -> /gripper_elevator_plate_to_gripper_extender/position (gz.msgs.Double)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.415004841] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/elevator_connector_plate/position (std_msgs/msg/Float64) -> /elevator_connector_plate/position (gz.msgs.Double)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.415237121] [ros_gz_bridge]: Creating GZ->ROS Bridge: [joint_states (gz.msgs.Model) -> joint_states (sensor_msgs/msg/JointState)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.415567529] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/oakd_top/color/camera_info (gz.msgs.CameraInfo) -> /oakd_top/color/camera_info (sensor_msgs/msg/CameraInfo)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.415898207] [ros_gz_bridge]: Creating GZ->ROS Bridge: [/gripper/camera/camera_info (gz.msgs.CameraInfo) -> /gripper/camera/camera_info (sensor_msgs/msg/CameraInfo)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.416188306] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/parallel_gripper_base_plate_to_left_finger/position (std_msgs/msg/Float64) -> /parallel_gripper_base_plate_to_left_finger/position (gz.msgs.Double)] (Lazy 0)
[parameter_bridge-4] [INFO] [1769885035.416450146] [ros_gz_bridge]: Creating ROS->GZ Bridge: [/parallel_gripper_base_plate_to_right_finger/position (std_msgs/msg/Float64) -> /parallel_gripper_base_plate_to_right_finger/position (gz.msgs.Double)] (Lazy 0)
[component_container_isolated-7] [INFO] [1769885035.766175028] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[gazebo-2] failed to create drawable
[gazebo-2] failed to create drawable
[component_container_isolated-7] [INFO] [1769885036.266427565] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[gazebo-2] [Msg] Material[Msg] Gazebo Sim GUI    v8.10.0
[gazebo-2] [Dbg] [Gui.cc:275] Waiting for subscribers to [/gazebo/starting_world]...
[gazebo-2] [Dbg] [Application.cc:96] Initializing application.
[gazebo-2] [Dbg] [Application.cc:170] Qt using OpenGL graphics interface
[gazebo-2] [GUI] [Dbg] [Application.cc:657] Create main window
[gazebo-2] [GUI] [Dbg] [PathManager.cc:68] Requesting resource paths through [/gazebo/resource_paths/get]
[gazebo-2] [GUI] [Dbg] [Gui.cc:355] GUI requesting list of world names. The server may be busy downloading resources. Please be patient.
[gazebo-2] [GUI] [Dbg] [PathManager.cc:57] Received resource paths.
[gazebo-2] [GUI] [Dbg] [Gui.cc:413] Requesting GUI from [/world/can_challenge/gui/info]...
[gazebo-2] [GUI] [Dbg] [GuiRunner.cc:149] Requesting initial state from [/world/can_challenge/state]...
[gazebo-2] [GUI] [Msg] Loading config [/home/ros/.gz/sim/8/gui.config]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [MinimalScene]
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:986] Creating render thread interface for OpenGL
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:986] Creating render thread interface for OpenGL
[gazebo-2] [GUI] [Msg] Added plugin [3D View] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [MinimalScene] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libMinimalScene.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [EntityContextMenuPlugin]
[gazebo-2] [GUI] [Msg] Currently tracking topic on [/gui/currently_tracked]
[gazebo-2] [GUI] [Msg] Added plugin [Entity Context Menu] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [EntityContextMenuPlugin] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libEntityContextMenuPlugin.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [GzSceneManager]
[gazebo-2] [GUI] [Msg] Added plugin [Scene Manager] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [GzSceneManager] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libGzSceneManager.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [InteractiveViewControl]
[gazebo-2] [GUI] [Msg] Camera view controller topic advertised on [/gui/camera/view_control]
[gazebo-2] [GUI] [Msg] Camera reference visual topic advertised on [/gui/camera/view_control/reference_visual]
[gazebo-2] [GUI] [Msg] Camera view control sensitivity advertised on [/gui/camera/view_control/sensitivity]
[gazebo-2] [GUI] [Msg] Added plugin [Interactive view control] to main[0m service on [/world/can_challenge/visual_config/blocking] (blocking)
[gazebo-2] [Msg] Material service on [/world/can_challenge/wheel_slip] (async)
[gazebo-2] [Msg] Material service on [/world/can_challenge/wheel_slip/blocking] (blocking)
[gazebo-2] [Dbg] [SystemManager.cc:80] Loaded system [gz::sim::systems::UserCommands] for entity [1]
[gazebo-2] [Dbg] [SystemManager.cc:80] Loaded system [gz::sim::systems::SceneBroadcaster] for entity [1]
[gazebo-2] [Dbg] [Sensors.cc:697] Configuring Sensors system
[gazebo-2] [Dbg] [Sensors.cc:557] SensorsPrivate::Run
[gazebo-2] [Dbg] [SystemManager.cc:80] Loaded system [gz::sim::systems::Sensors] for entity [1]
[gazebo-2] [Dbg] [Sensors.cc:532] SensorsPrivate::RenderThread started
[gazebo-2] [Dbg] [Sensors.cc:337] Waiting for init
[gazebo-2] [Msg] Loaded level [default]
[gazebo-2] [Msg] Serving world controls on [/world/can_challenge/control], [/world/can_challenge/control/state] and [/world/can_challenge/playback/control]
[gazebo-2] [Msg] Serving GUI information on [/world/can_challenge/gui/info]
[gazebo-2] [Msg] World [can_challenge] initialized with [1ms] physics profile.
[gazebo-2] [Msg] Serving world SDF generation service on [/world/can_challenge/generate_world_sdf]
[gazebo-2] [Msg] Serving world names on [/gazebo/worlds]
[gazebo-2] [Msg] Resource path add service on [/gazebo/resource_paths/add].
[gazebo-2] [Msg] Resource path get service on [/gazebo/resource_paths/get].
[gazebo-2] [Msg] Resource path resolve service on [/gazebo/resource_paths/resolve].
[gazebo-2] [Msg] Resource paths published on [/gazebo/resource_paths].
[gazebo-2] [Msg] Server control service on [/server_control].
[gazebo-2] [Msg] Found no publishers on /stats, adding root stats topic
[gazebo-2] [Msg] Found no publishers on /clock, adding root clock topic
[gazebo-2] [Dbg] [SimulationRunner.cc:533] Creating PostUpdate worker threads: 3
[gazebo-2] [Dbg] [SimulationRunner.cc:544] Creating postupdate worker thread (0)
[gazebo-2] [Dbg] [SimulationRunner.cc:544] Creating postupdate worker thread (1)
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2] Warning [parser_urdf.cc:1220] Attribute value string not set
[gazebo-2]  window
[gazebo-2] [GUI] [Msg] Loaded plugin [InteractiveViewControl] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libInteractiveViewControl.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [CameraTracking]
[gazebo-2] [GUI] [Msg] Added plugin [Camera tracking] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [CameraTracking] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libCameraTracking.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [MarkerManager]
[gazebo-2] [GUI] [Msg] Listening to stats on [/world/can_challenge/stats]
[gazebo-2] [GUI] [Msg] Added plugin [Marker Manager] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [MarkerManager] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libMarkerManager.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [SelectEntities]
[gazebo-2] [GUI] [Msg] Added plugin [Select entities] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [SelectEntities] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libSelectEntities.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [Spawn]
[gazebo-2] [GUI] [Msg] Added plugin [Spawn] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [Spawn] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libSpawn.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [VisualizationCapabilities]
[gazebo-2] [GUI] [Msg] View as transparent service on [/gui/view/transparent]
[gazebo-2] [GUI] [Msg] View as wireframes service on [/gui/view/wireframes]
[gazebo-2] [GUI] [Msg] View center of mass service on [/gui/view/com]
[gazebo-2] [GUI] [Msg] View inertia service on [/gui/view/inertia]
[gazebo-2] [GUI] [Msg] View collisions service on [/gui/view/collisions]
[gazebo-2] [GUI] [Msg] View joints service on [/gui/view/joints]
[gazebo-2] [GUI] [Msg] View frames service on [/gui/view/frames]
[gazebo-2] [GUI] [Msg] Added plugin [Visualization capabilities] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [VisualizationCapabilities] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libVisualizationCapabilities.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [WorldControl]
[gazebo-2] [GUI] [Msg] Using world control service [/world/can_challenge/control]
[gazebo-2] [GUI] [Msg] Listening to stats on [/world/can_challenge/stats]
[gazebo-2] [GUI] [Dbg] [WorldControl.cc:237] Using an event to share WorldControl msgs with the server
[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="Sigyn"]/link[@name="base_link"]/sensor[@name="lidar_frame_cup_lidar"]/lidar/ray:<urdf-string>:L0]: XML Element[ray], child of element[lidar], not defined in SDF. Copying[ray] as children of [lidar].
[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="Sigyn"]/link[@name="base_link"]/sensor[@name="lidar_frame_cup_lidar"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="Sigyn"]/link[@name="base_link"]/plugin[@name="gazebo_ros_laser_controller"]:<urdf-string>:L0]: XML Element[plugin], child of element[link], not defined in SDF. Copying[plugin] as children of [link].
[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="Sigyn"]/link[@name="base_link"]/sensor[@name="lidar_frame_top_lidar"]/lidar/ray:<urdf-string>:L0]: XML Element[ray], child of element[lidar], not defined in SDF. Copying[ray] as children of [lidar].
[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="Sigyn"]/link[@name="base_link"]/sensor[@name="lidar_frame_top_lidar"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="Sigyn"]/link[@name="base_link"]/plugin[@name="gazebo_ros_laser_controller"]:<urdf-string>:L0]: XML Element[plugin], child of element[link], not defined in SDF. Copying[plugin] as children of [link].
[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="Sigyn"]/link[@name="base_link"]/sensor[@name="oakd_camera"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
[gazebo-2] Warning [Utils.cc:132] [/sdf/model[@name="Sigyn"]/link[@name="parallel_gripper_base_plate"]/sensor[@name="pi_camera"]/gz_frame_id:<urdf-string>:L0]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Grey
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Black
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Black
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Black
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Black
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Black
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Black
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Black
[gazebo-2] [Wrn] [SdfEntityCreator.cc:933] Gazebo does not support Ogre material scripts. See https://gazebosim.org/api/sim/8/migrationsdf.html#:~:text=Materials for details.
[gazebo-2] [Wrn] [SdfEntityCreator.cc:949] Using an internal gazebo.material to parse Gazebo/Black
[gazebo-2] [ERROR] [1769885036.606043115] [rcl]: Failed to parse global arguments
[gazebo-2] terminate called after throwing an instance of 'rclcpp::exceptions::RCLInvalidROSArgsError'
[gazebo-2]   what():  failed to initialize rcl: Couldn't parse params file: '--params-file /home/ros/sigyn_ws/install/description/share/description/config/my_controllers.yaml'. Error: No value at line 28, at ./src/parse.c:258, at ./src/rcl/arguments.c:415
[gazebo-2] Stack trace (most recent call last):
[gazebo-2] #31   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x74152a25013e, in 
[gazebo-2] #30   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x74152a24d92f, in 
[gazebo-2] #29   Object "/usr/lib/x86_64-linux-gnu/ruby/3.2.0/fiddle.so", at 0x741529b87b13, in 
[gazebo-2] #28   Object "/lib/x86_64-linux-gnu/libruby-3.2.so.3.2", at 0x74152a21637b, in rb_nogvl
[gazebo-2] #27   Object "/usr/lib/x86_64-linux-gnu/ruby/3.2.0/fiddle.so", at 0x741529b8743b, in 
[gazebo-2] #26   Object "/lib/x86_64-linux-gnu/libffi.so.8", at 0x741529ed80bd, in ffi_call
[gazebo-2] #25   Object "/lib/x86_64-linux-gnu/libffi.so.8", at 0x741529ed53ee, in 
[gazebo-2] #24   Object "/lib/x86_64-linux-gnu/libffi.so.8", at 0x741529ed8b15, in 
[gazebo-2] #23   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8-gz.so.8.10.0", at 0x7415243d45bc, in runServer
[gazebo-2] #22   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d49bdc, in 
[gazebo-2] #21   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d5782e, in gz::sim::v8::SimulationRunner::Run(unsigned long)
[gazebo-2] #20   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d56ebe, in gz::sim::v8::SimulationRunner::Step(gz::sim::v8::UpdateInfo const&)
[gazebo-2] #19   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d55119, in gz::sim::v8::SimulationRunner::UpdateSystems()
[gazebo-2] #18   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-user-commands-system.so", at 0x741503f77c43, in gz::sim::v8::systems::UserCommands::PreUpdate(gz::sim::v8::UpdateInfo const&, gz::sim::v8::EntityComponentManager&)
[gazebo-2] #17   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-user-commands-system.so", at 0x741503f7cba0, in gz::sim::v8::systems::CreateCommand::CreateFromMsg(gz::msgs::EntityFactory const&)
[gazebo-2] #16   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d2d74f, in gz::sim::v8::SdfEntityCreator::CreateEntities(sdf::v14::Model const*)
[gazebo-2] #15   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d24798, in gz::sim::v8::SdfEntityCreator::LoadModelPlugins()
[gazebo-2] #14   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d2441e, in 
[gazebo-2] #13   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d5536c, in gz::sim::v8::SimulationRunner::LoadPlugins(unsigned long, std::vector<sdf::v14::Plugin, std::allocator<sdf::v14::Plugin> > const&)
[gazebo-2] #12   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d68dc2, in gz::sim::v8::SystemManager::LoadPlugin(unsigned long, sdf::v14::Plugin const&)
[gazebo-2] #11   Object "/opt/ros/jazzy/opt/gz_sim_vendor/lib/libgz-sim8.so.8", at 0x741523d68830, in gz::sim::v8::SystemManager::AddSystemImpl(gz::sim::v8::SystemInternal, std::shared_ptr<sdf::v14::Element const>)
[gazebo-2] #10   Object "/opt/ros/jazzy/lib/libgz_ros2_control-system.so", at 0x7415033c6f16, in gz_ros2_control::GazeboSimROS2ControlPlugin::Configure(unsigned long const&, std::shared_ptr<sdf::v14::Element const> const&, gz::sim::v8::EntityComponentManager&, gz::sim::v8::EventManager&)
[gazebo-2] #9    Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x741500bd9af7, in rclcpp::init(int, char const* const*, rclcpp::InitOptions const&, rclcpp::SignalHandlerOptions)
[gazebo-2] #8    Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x741500afb121, in rclcpp::Context::init(int, char const* const*, rclcpp::InitOptions const&)
[gazebo-2] #7    Object "/opt/ros/jazzy/lib/librclcpp.so", at 0x741500aff797, in rclcpp::exceptions::throw_from_rcl_error(int, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, rcutils_error_state_s const*, void (*)())
[gazebo-2] #6    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x741524cbb0c0, in std::rethrow_exception(std::__exception_ptr::exception_ptr)
[gazebo-2] #5    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x741524ca5a54, in std::terminate()
[gazebo-2] #4    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x741524cbb0d9, in 
[gazebo-2] #3    Object "/lib/x86_64-linux-gnu/libstdc++.so.6", at 0x741524ca5ff4, in 
[gazebo-2] #2    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x741529c288fe, in abort
[gazebo-2] #1    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x741529c4527d, in gsignal
[gazebo-2] #0    Object "/lib/x86_64-linux-gnu/libc.so.6", at 0x741529c9eb2c, in pthread_kill
[gazebo-2] Aborted (Signal sent by tkill() 32482 1000)
[gazebo-2] [GUI] [Msg] Added plugin [World control] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [WorldControl] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libWorldControl.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [WorldStats]
[gazebo-2] [GUI] [Msg] Listening to stats on [/world/can_challenge/stats]
[gazebo-2] [GUI] [Msg] Added plugin [World stats] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [WorldStats] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libWorldStats.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [Shapes]
[gazebo-2] [GUI] [Msg] Added plugin [Shapes] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [Shapes] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libShapes.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [Lights]
[gazebo-2] [GUI] [Msg] Added plugin [Lights] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [Lights] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libLights.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [TransformControl]
[gazebo-2] [GUI] [Msg] Added plugin [Transform control] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [TransformControl] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libTransformControl.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [Screenshot]
[gazebo-2] [GUI] [Msg] Screenshot service on [/gui/screenshot]
[gazebo-2] [GUI] [Msg] Added plugin [Screenshot] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [Screenshot] from path [/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins/libScreenshot.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [CopyPaste]
[gazebo-2] [GUI] [Msg] Added plugin [Copy/Paste] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [CopyPaste] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libCopyPaste.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [ComponentInspector]
[gazebo-2] [GUI] [Msg] Added plugin [Component inspector] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [ComponentInspector] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libComponentInspector.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:528] Loading plugin [EntityTree]
[gazebo-2] [GUI] [Msg] Currently tracking topic on [/gui/currently_tracked]
[component_container_isolated-7] [INFO] [1769885036.766150800] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[gazebo-2] failed to create drawable
[gazebo-2] [GUI] [Wrn] [Application.cc:908] [QT] file::/WorldStats/WorldStats.qml:53:3: QML RowLayout: Binding loop detected for property "x"
[rviz2-12] [INFO] [1769885037.127882052] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-12] [INFO] [1769885037.127983807] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[component_container_isolated-7] [INFO] [1769885037.266135830] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[rviz2-12] [INFO] [1769885037.303148644] [rviz2]: Stereo is NOT SUPPORTED
[INFO] [spawner-13]: process started with pid [32693]
[gazebo-2] failed to create drawable
[spawner-13] [INFO] [1769885037.585123170] [spawner_joint_broadcaster]: waiting for service /controller_manager/list_controllers to become available...
[gazebo-2] libEGL warning: failed to open /dev/dri/renderD128: Permission denied
[gazebo-2] 
[gazebo-2] libEGL warning: failed to open /dev/dri/renderD128: Permission denied
[gazebo-2] 
[gazebo-2] libEGL warning: failed to open /dev/dri/card0: Permission denied
[gazebo-2] 
[component_container_isolated-7] [INFO] [1769885037.766131426] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[gazebo-2] [GUI] [Wrn] [Ogre2RenderTarget.cc:668] Anti-aliasing level of ' is not supported; valid FSAA levels are: [ 0 ]. Setting to 1
[rviz2-12] [INFO] [1769885038.051094145] [rviz2]: Stereo is NOT SUPPORTED
[gazebo-2] [GUI] [Msg] Added plugin [Entity tree] to main window
[gazebo-2] [GUI] [Msg] Loaded plugin [EntityTree] from path [/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui/libEntityTree.so]
[gazebo-2] [GUI] [Dbg] [Application.cc:398] Loading window config
[gazebo-2] [GUI] [Msg] Using server control service [/server_control]
[gazebo-2] [GUI] [Dbg] [Application.cc:671] Applying config
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:802] Creating gz-rendering interface for OpenGL
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:986] Creating render thread interface for OpenGL
[gazebo-2] [GUI] [Msg] Loading plugin [gz-rendering-ogre2]
[gazebo-2] [GUI] [Dbg] [SignalHandler.cc:142] Received signal[2].
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:749] Create scene [scene]
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:1037] Creating texture node render interface for OpenGL
[gazebo-2] [GUI] [Dbg] [Gui.cc:535] Shutting down gz-sim-gui
[gazebo-2] [GUI] [Dbg] [Application.cc:237] Terminating application.
[gazebo-2] [GUI] [Dbg] [TransformControl.cc:453] TransformControl plugin is using camera [scene::Camera(65527)]
[gazebo-2] [GUI] [Dbg] [Spawn.cc:308] Spawn plugin is using camera [scene::Camera(65527)]
[gazebo-2] [GUI] [Dbg] [SelectEntities.cc:452] SelectEntities plugin is using camera [scene::Camera(65527)]
[gazebo-2] [GUI] [Dbg] [MarkerManager.cc:169] Advertise /marker/list service.
[gazebo-2] [GUI] [Dbg] [MarkerManager.cc:179] Advertise /marker/list.
[gazebo-2] [GUI] [Dbg] [MarkerManager.cc:189] Advertise /marker_array.
[gazebo-2] [GUI] [Dbg] [CameraTracking.cc:205] CameraTrackingPrivate plugin is moving camera [scene::Camera(65527)]
[gazebo-2] [GUI] [Msg] Move to service on [/gui/move_to]
[gazebo-2] [GUI] [Msg] Follow service on [/gui/follow] (deprecated)
[gazebo-2] [GUI] [Msg] Tracking topic on [/gui/track]
[gazebo-2] [GUI] [Msg] Tracking status topic on [/gui/currently_tracked]
[gazebo-2] [GUI] [Msg] Move to pose service on [/gui/move_to/pose]
[gazebo-2] [GUI] [Msg] Camera pose topic advertised on [/gui/camera/pose]
[gazebo-2] [GUI] [Msg] Follow offset service on [/gui/follow/offset] (deprecated)
[gazebo-2] [GUI] [Dbg] [InteractiveViewControl.cc:176] InteractiveViewControl plugin is moving camera [scene::Camera(65527)]
[gazebo-2] [GUI] [Dbg] [EntityContextMenuPlugin.cc:79] Entity context menu plugin is using camera [scene::Camera(65527)]
[gazebo-2] [GUI] [Dbg] [MinimalScene.cc:841] Destroy scene [scene]
[INFO] [gazebo-2]: process has finished cleanly [pid 32345]
[INFO] [launch]: process[gazebo-2] was required: shutting down launched system
[INFO] [spawner-13]: sending signal 'SIGINT' to process[spawner-13]
[INFO] [rviz2-12]: sending signal 'SIGINT' to process[rviz2-12]
[INFO] [battery_overlay_publisher-11]: sending signal 'SIGINT' to process[battery_overlay_publisher-11]
[INFO] [MoveAShortDistanceAheadActionServer-10]: sending signal 'SIGINT' to process[MoveAShortDistanceAheadActionServer-10]
[INFO] [SaySomethingActionServer-9]: sending signal 'SIGINT' to process[SaySomethingActionServer-9]
[INFO] [component_container_isolated-7]: sending signal 'SIGINT' to process[component_container_isolated-7]
[INFO] [image_bridge-6]: sending signal 'SIGINT' to process[image_bridge-6]
[INFO] [image_bridge-5]: sending signal 'SIGINT' to process[image_bridge-5]
[INFO] [parameter_bridge-4]: sending signal 'SIGINT' to process[parameter_bridge-4]
[INFO] [robot_state_publisher-1]: sending signal 'SIGINT' to process[robot_state_publisher-1]
[component_container_isolated-7] [INFO] [1769885038.266129141] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[component_container_isolated-7] [INFO] [1769885038.766126873] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[rviz2-12] [INFO] [1769885038.439617804] [rviz2]: Trying to create a map of size 573 x 589 using 1 swatches
[rviz2-12] [ERROR] [1769885038.475125330] [rviz2]: rviz/glsl120/indexed_8bit_image.vert
[rviz2-12] rviz/glsl120/indexed_8bit_image.frag
[rviz2-12]  GLSL link result : 
[rviz2-12] active samplers with a different type refer to the same texture image unit
[rviz2-12] [INFO] [1769885038.905892530] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[rviz2-12] 
[rviz2-12] >>> [rcutils|error_handling.c:108] rcutils_set_error_state()
[rviz2-12] This error state is being overwritten:
[rviz2-12] 
[rviz2-12]   'rcl node's context is invalid, at ./src/rcl/node.c:404'
[rviz2-12] 
[rviz2-12] with this new error message:
[rviz2-12] 
[rviz2-12]   'the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at ./src/rcl/wait.c:130'
[rviz2-12] 
[rviz2-12] rcutils_reset_error() should be called after error handling to avoid this.
[rviz2-12] <<<
[rviz2-12] terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
[rviz2-12]   what():  failed to initialize wait set: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at ./src/rcl/wait.c:130
[spawner-13] [INFO] [1769885038.906168535] [spawner_joint_broadcaster]: KeyboardInterrupt received! Exiting....
[MoveAShortDistanceAheadActionServer-10] [INFO] [1769885038.907212110] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[SaySomethingActionServer-9] [INFO] [1769885038.908040685] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[component_container_isolated-7] [INFO] [1769885038.908936186] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[component_container_isolated-7] [INFO] [1769885038.908984258] [map_server]: Running Nav2 LifecycleNode rcl preshutdown (map_server)
[component_container_isolated-7] [INFO] [1769885038.909026110] [map_server]: Deactivating
[component_container_isolated-7] [INFO] [1769885038.909040154] [map_server]: Destroying bond (map_server) to lifecycle manager.
[image_bridge-6] [INFO] [1769885038.909646728] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[image_bridge-5] [INFO] [1769885038.910307064] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[parameter_bridge-4] [INFO] [1769885038.911023977] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[robot_state_publisher-1] [INFO] [1769885038.911651947] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[INFO] [parameter_bridge-4]: process has finished cleanly [pid 32347]
[component_container_isolated-7] [INFO] [1769885038.919169948] [map_server]: Cleaning up
[component_container_isolated-7] [INFO] [1769885038.919201312] [map_server]: Destroying bond (map_server) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885038.919207533] [amcl]: Running Nav2 LifecycleNode rcl preshutdown (amcl)
[component_container_isolated-7] [INFO] [1769885038.919219784] [amcl]: Deactivating
[component_container_isolated-7] [INFO] [1769885038.919232436] [amcl]: Destroying bond (amcl) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885038.929354496] [amcl]: Cleaning up
[component_container_isolated-7] [INFO] [1769885038.933185606] [amcl]: Destroying bond (amcl) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885038.933239559] [lifecycle_manager_localization]: Running Nav2 LifecycleManager rcl preshutdown (lifecycle_manager_localization)
[component_container_isolated-7] [INFO] [1769885038.933257199] [lifecycle_manager_localization]: Terminating bond timer...
[component_container_isolated-7] [INFO] [1769885038.961728532] [controller_server]: Running Nav2 LifecycleNode rcl preshutdown (controller_server)
[component_container_isolated-7] [INFO] [1769885038.961748867] [controller_server]: Destroying bond (controller_server) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885038.961756680] [smoother_server]: Running Nav2 LifecycleNode rcl preshutdown (smoother_server)
[component_container_isolated-7] [INFO] [1769885038.961772167] [smoother_server]: Cleaning up
[component_container_isolated-7] [INFO] [1769885038.968759787] [smoother_server]: Destroying bond (smoother_server) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885038.968775303] [planner_server]: Running Nav2 LifecycleNode rcl preshutdown (planner_server)
[component_container_isolated-7] [INFO] [1769885038.968788135] [planner_server]: Cleaning up
[component_container_isolated-7] [INFO] [1769885038.978864928] [global_costmap.global_costmap]: Cleaning up
[component_container_isolated-7] [INFO] [1769885038.982406891] [planner_server]: Cleaning up plugin GridBased of type NavfnPlanner
[component_container_isolated-7] [INFO] [1769885038.982535462] [planner_server]: Destroying plugin GridBased of type NavfnPlanner
[component_container_isolated-7] [INFO] [1769885039.010391809] [planner_server]: Destroying bond (planner_server) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885039.010402498] [behavior_server]: Running Nav2 LifecycleNode rcl preshutdown (behavior_server)
[component_container_isolated-7] [INFO] [1769885039.010412595] [behavior_server]: Cleaning up
[component_container_isolated-7] [INFO] [1769885039.011434193] [behavior_server]: Destroying bond (behavior_server) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885039.011442617] [bt_navigator]: Running Nav2 LifecycleNode rcl preshutdown (bt_navigator)
[component_container_isolated-7] [INFO] [1769885039.011451532] [bt_navigator]: Cleaning up
[component_container_isolated-7] [INFO] [1769885039.046282684] [bt_navigator]: Completed Cleaning up
[component_container_isolated-7] [INFO] [1769885039.046299123] [bt_navigator]: Destroying bond (bt_navigator) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885039.046303761] [waypoint_follower]: Running Nav2 LifecycleNode rcl preshutdown (waypoint_follower)
[component_container_isolated-7] [INFO] [1769885039.046313157] [waypoint_follower]: Cleaning up
[component_container_isolated-7] [INFO] [1769885039.053865950] [waypoint_follower]: Destroying bond (waypoint_follower) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885039.053877370] [velocity_smoother]: Running Nav2 LifecycleNode rcl preshutdown (velocity_smoother)
[component_container_isolated-7] [INFO] [1769885039.053889932] [velocity_smoother]: Cleaning up
[component_container_isolated-7] [INFO] [1769885039.053904176] [velocity_smoother]: Destroying bond (velocity_smoother) to lifecycle manager.
[component_container_isolated-7] [INFO] [1769885039.053909896] [lifecycle_manager_navigation]: Running Nav2 LifecycleManager rcl preshutdown (lifecycle_manager_navigation)
[component_container_isolated-7] [INFO] [1769885039.266126996] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[component_container_isolated-7] [INFO] [1769885039.766423043] [local_costmap.local_costmap]: Timed out waiting for transform from base_link to odom to become available, tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist
[INFO] [MoveAShortDistanceAheadActionServer-10]: process has finished cleanly [pid 32354]
[INFO] [SaySomethingActionServer-9]: process has finished cleanly [pid 32353]
