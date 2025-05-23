# Aliases of interest on sigyn7900
```bash
alias cb='colcon build --symlink-install'
alias cgcm='ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap'
alias clcm='ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap'
alias dla='ros2 run --prefix '\''gdbserver localhost:3000'\'' line_finder laser_accumulator'
alias fr='ros2 run tf2_tools view_frames'
alias gripper='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_gripper'
alias record='/home/ros/sigyn_ws/src/Sigyn/scripts/bag_record_sim.sh'
alias redoudev='sudo service udev restart;sudo udevadm control --reload-rules;sudo udevadm trigger'
alias rms='ros2 launch nav2_map_server map_saver_server.launch.py'
alias rvs='rviz2 -d ~/sigyn_ws/src/Sigyn/rviz/config/config.rviz'
alias savem='ros2 run nav2_map_server map_saver_cli -f my_map'
alias sim='clear;ros2 launch base sigyn.launch.py use_sim_time:=true do_rviz:=true'
alias stele='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_keyboard'
alias teensy='ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_sensor'
alias tele='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
```

# Capturing images
## On terminal 1
```bash
ssh sigyn7900
teensy
```

This introduces the topics:
* /cmd_vel
* /main_battery
* /roboclaw_status
* /teensy_diagnostics
* /teensy_stats
* /wheel_odom


## On terminal 2
```bash 
ssh sigyn7900
gripper
```

This introduces the topics:
* elevator_cmd
* extender_cmd
* gripper_diagnostics
* gripper_stats

  ros2 topic pub -1 /extender_cmd std_msgs/msg/Float32 "data: 0.2"
  ros2 topic pub -1 /elevator_cmd std_msgs/msg/Float32 "data: 0.2"
  ros2 action send_goal /move_elevator sigyn_interfaces/action/MoveElevator "{goal_position: 0.02}" --feedback 
  ros2 action send_goal /move_extender sigyn_interfaces/action/MoveExtender "{goal_position: 0.02}" --feedback

## On terminal 3
```bash
ssh sigynPiServo
python3 videoServer.py
```

## On terminal 4
```bash
ssh sigyn7900
nav
```



