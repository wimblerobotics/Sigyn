[ros2_control_node-2] [INFO] [1750887903.951760244] [controller_manager]: Loading hardware 'endgripper_servo' 
[ros2_control_node-2] [ERROR] [1750887903.951788834] [controller_manager]: Caught exception of type : N9pluginlib20LibraryLoadExceptionE while loading hardware: According to the loaded plugin descriptions the class pi_servo1/EndGripperJoint with base class type hardware_interface::ActuatorInterface does not exist. Declared types are 
[ros2_control_node-2] [WARN] [1750887903.951816232] [controller_manager]: Could not load and initialize hardware. Please check previous output for more details. After you have corrected your URDF, try to publish robot description again.


[component_container_isolated-18] [INFO] [1750887904.021987536] [map_server]: Configuring
[component_container_isolated-18] [INFO] [1750887904.022022194] [map_io]: Loading yaml file: /home/ros/sigyn_ws/install/base/share/base/maps/my_map.yaml
[component_container_isolated-18] [INFO] [1750887904.022181808] [map_io]: resolution: 0.0261762
[component_container_isolated-18] [INFO] [1750887904.022188638] [map_io]: origin[0]: 0
[component_container_isolated-18] [INFO] [1750887904.022190901] [map_io]: origin[1]: 0
[component_container_isolated-18] [INFO] [1750887904.022193394] [map_io]: origin[2]: 0
[component_container_isolated-18] [INFO] [1750887904.022195557] [map_io]: free_thresh: 0.25
[component_container_isolated-18] [INFO] [1750887904.022197690] [map_io]: occupied_thresh: 0.65
[component_container_isolated-18] [INFO] [1750887904.022200825] [map_io]: mode: trinary
[component_container_isolated-18] [INFO] [1750887904.022203078] [map_io]: negate: 0
[component_container_isolated-18] [INFO] [1750887904.022306012] [map_io]: Loading image_file: /home/ros/sigyn_ws/install/base/share/base/maps/my_map2.pgm
[component_container_isolated-18] [INFO] [1750887904.041724422] [map_io]: Read map /home/ros/sigyn_ws/install/base/share/base/maps/my_map2.pgm: 573 X 589 map @ 0.0261762 m/cell
[component_container_isolated-18] [INFO] [1750887904.043030405] [lifecycle_manager_localization]: Configuring amcl


this_to_that_node.py-12] [INFO] [1750887904.094278931] [batter_voltage_overlay]: Declared parameter 'field_mappings_str' as STRING
[this_to_that_node.py-12] [INFO] [1750887904.094586002] [batter_voltage_overlay]: Declared parameter 'static_fields_str' as STRING
[this_to_that_node.py-12] [INFO] [1750887904.094809826] [batter_voltage_overlay]: Loaded field_mappings_str: 'voltage:current'
[this_to_that_node.py-12] [INFO] [1750887904.095005721] [batter_voltage_overlay]: Loaded static_fields_str: 'min=0.0,max=42.0,compact=true,title=Batt'
[this_to_that_node.py-12] [INFO] [1750887904.095174027] [batter_voltage_overlay]: Parsed field mappings from string: {'voltage': 'current'}
[this_to_that_node.py-12] [INFO] [1750887904.095355201] [batter_voltage_overlay]: Parsed static fields from string: {'min': 0.0, 'max': 42.0, 'compact': True, 'title': 'Batt'}


[INFO] [twist_multiplexer-7]: process started with pid [14168]
[INFO] [robot_state_publisher-1]: process started with pid [14162]
[INFO] [ros2_control_node-2]: process started with pid [14163]
...
[ros2_control_node-2] [INFO] [1750887903.912703094] [controller_manager]: Using Steady (Monotonic) clock for triggering controller manager cycles.
[ros2_control_node-2] [INFO] [1750887903.915397491] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[ros2_control_node-2] [INFO] [1750887903.948606647] [controller_manager]: update rate is 31 Hz
[ros2_control_node-2] [INFO] [1750887903.948629359] [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
[ros2_control_node-2] [INFO] [1750887903.948687610] [controller_manager]: Successful set up FIFO RT scheduling policy with priority 50.
[ros2_control_node-2] [INFO] [1750887903.949211210] [controller_manager]: Received robot description from topic.
[ros2_control_node-2] [INFO] [1750887903.951760244] [controller_manager]: Loading hardware 'endgripper_servo' 
[ros2_control_node-2] [ERROR] [1750887903.951788834] [controller_manager]: Caught exception of type : N9pluginlib20LibraryLoadExceptionE while loading hardware: According to the loaded plugin descriptions the class pi_servo1/EndGripperJoint with base class type hardware_interface::ActuatorInterface does not exist. Declared types are 
[ros2_control_node-2] [WARN] [1750887903.951816232] [controller_manager]: Could not load and initialize hardware. Please check previous output for more details. After you have corrected your URDF, try to publish robot description again.
[ros2_control_node-2] [WARN] [1750887904.915281705] [controller_manager]: Waiting for data on 'robot_description' topic to finish initialization


note /main_battery and batter_voltage_overlay
ros@sigyn7900:~$ ros2 topic info -v /main_battery
Type: sensor_msgs/msg/BatteryState

Publisher count: 0

Subscription count: 1

Node name: batter_voltage_overlay
Node namespace: /
Topic type: sensor_msgs/msg/BatteryState
Topic type hash: RIHS01_4bee5dfce981c98faa6828b868307a0a73f992ed0789f374ee96c8f840e69741
Endpoint type: SUBSCRIPTION
GID: 01.10.50.bf.cd.8f.f5.0e.d2.81.d5.b9.00.00.16.04
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

ros@sigyn7900:~$ ros2 topic info -v /battery_voltage_overlay 
Type: min_max_curr_rviz_overlay/msg/MinMaxCurr

Publisher count: 1

Node name: batter_voltage_overlay
Node namespace: /
Topic type: min_max_curr_rviz_overlay/msg/MinMaxCurr
Topic type hash: RIHS01_497700db269f8d67f285cafe41dbb3fd1a85897530184474a00e5b519853d8f4
Endpoint type: PUBLISHER
GID: 01.10.50.bf.cd.8f.f5.0e.d2.81.d5.b9.00.00.17.03
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (10)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0


ros@sigyn7900:~$ ros2 topic echo --once /main_battery_state 
header:
  stamp:
    sec: 1750890556
    nanosec: 532953384
  frame_id: base_link
voltage: 40.16999816894531
temperature: 0.0
current: 0.0
charge: 0.0
capacity: 1.0
design_capacity: 1.0
percentage: 81.66000366210938
power_supply_status: 2
power_supply_health: 1
power_supply_technology: 3
present: true
cell_voltage: []
cell_temperature: []
location: ''
serial_number: ''

os@sigyn7900:~$ ros2 topic list
/amcl/transition_event
/amcl_pose
/battery_voltage_overlay
/behavior_server/transition_event
/behavior_tree_log
/bluetoothJoystick
/bond
/bt_navigator/transition_event
/clicked_point
/clock
/cmd_vel
/cmd_vel_gripper
/cmd_vel_joystick
/cmd_vel_keyboard
/cmd_vel_nav
/cmd_vel_smoothed
/cmd_vel_teleop
/controller_manager/activity
/controller_manager/introspection_data/full
/controller_manager/introspection_data/names
/controller_manager/introspection_data/values
/controller_selector
/controller_server/transition_event
/diagnostics
/global_costmap/costmap
/global_costmap/costmap_raw
/global_costmap/costmap_raw_updates
/global_costmap/costmap_updates
/global_costmap/footprint
/global_costmap/global_costmap/transition_event
/global_costmap/obstacle_layer
/global_costmap/obstacle_layer_raw
/global_costmap/obstacle_layer_raw_updates
/global_costmap/obstacle_layer_updates
/global_costmap/published_footprint
/global_costmap/static_layer
/global_costmap/static_layer_raw
/global_costmap/static_layer_raw_updates
/global_costmap/static_layer_updates
/goal_pose
/initialpose
/joint_states
/local_costmap/costmap
/local_costmap/costmap_raw
/local_costmap/costmap_raw_updates
/local_costmap/costmap_updates
/local_costmap/footprint
/local_costmap/lidar_layer
/local_costmap/lidar_layer_raw
/local_costmap/lidar_layer_raw_updates
/local_costmap/lidar_layer_updates
/local_costmap/local_costmap/transition_event
/local_costmap/oakd_top_layer
/local_costmap/oakd_top_layer_raw
/local_costmap/oakd_top_layer_raw_updates
/local_costmap/oakd_top_layer_updates
/local_costmap/published_footprint
/local_costmap/static_layer
/local_costmap/static_layer_raw
/local_costmap/static_layer_raw_updates
/local_costmap/static_layer_updates
/main_battery
/main_battery_state
/map
/map_server/transition_event
/map_updates
/odom
/optimal_trajectory
/parameter_events
/particle_cloud
/plan
/plan_smoothed
/planner_selector
/planner_server/transition_event
/preempt_teleop
/raw_scan
/roboclaw_status
/robot_description
/rosout
/scan
/set_pose
/smoother_server/transition_event
/speed_limit
/stereo/points2
/teensy_elevator_diagnostics
/teensy_sensor_diagnostics
/tf
/tf_static
/trajectories
/transformed_global_plan
/velocity_smoother/transition_event
/waypoint_follower/transition_event
/waypoints
/wheel_odom
