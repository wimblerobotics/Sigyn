# Perimeter Roamer V2

An advanced ROS2 Python package for autonomous house patrolling with intelligent room detection, hallway navigation, and doorway handling capabilities.

## Features

- **Intelligent Space Classification**: Automatically detects rooms, hallways, and doorways using LIDAR data
- **Room Patrol**: Maintains 1-meter distance from walls while patrolling room perimeters
- **Hallway Navigation**: Centers the robot in narrow hallways for safe passage
- **Doorway Handling**: Carefully approaches, centers, and navigates through narrow doorways
- **Collision Avoidance**: Real-time obstacle detection and avoidance using LIDAR
- **Recovery Behaviors**: Automatic recovery from stuck situations
- **Visualization**: RViz markers for monitoring space type and robot state

## Robot Requirements

This package is designed for robots with:
- 2-wheel differential drive with rear caster wheel
- LIDAR sensor (270° viewing angle recommended)
- Wheel encoders for odometry
- IMU sensors for orientation
- ROS2 Jazzy distribution

## Frame Transformations

The package automatically handles frame transformations for both simulation and real robot scenarios:

### Simulation
- Uses ROS time (`/clock` topic) when `use_sim_time=true`
- Automatically detects simulation clock type
- Handles frame transformations with simulation time

### Real Robot
- Uses wall clock time when `use_sim_time=false`
- Automatically detects real-time clock type
- Handles frame transformations with wall time

### Required Frames
- `base_link`: Robot's base frame
- `odom`: Odometry frame
- LIDAR frame (usually `base_link` or `laser_link`)

The package validates that sensor data is in the expected frames and logs warnings if frame mismatches are detected.

## Installation

1. Place this package in your ROS2 workspace `src` directory
2. Build the workspace:
   ```bash
   colcon build --packages-select perimeter_roamer_v2
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Real Robot Launch

Launch the perimeter roamer on a real robot with default parameters:

```bash
ros2 launch perimeter_roamer_v2 perimeter_roamer_v2.launch.py
```

### Simulation Launch

Launch the perimeter roamer in simulation (Gazebo, etc.):

```bash
ros2 launch perimeter_roamer_v2 perimeter_roamer_v2_sim.launch.py
```

Or with custom simulation time setting:

```bash
ros2 launch perimeter_roamer_v2 perimeter_roamer_v2_sim.launch.py use_sim_time:=true
```

### Topics

#### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): LIDAR scan data
- `/odom` (nav_msgs/Odometry): Robot odometry

#### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Robot velocity commands
- `/space_type` (std_msgs/String): Current space classification
- `/roamer_state` (std_msgs/String): Current robot state
- `/roamer_markers` (visualization_msgs/MarkerArray): RViz visualization markers

### Parameters

Key parameters can be configured in `config/perimeter_roamer_v2_params.yaml`:

#### Speed Settings
- `forward_speed`: Normal forward speed (default: 0.12 m/s)
- `turn_speed`: Turning speed (default: 0.25 rad/s)
- `narrow_space_speed`: Speed in narrow spaces (default: 0.05 m/s)

#### Space Classification
- `doorway_width_threshold`: Maximum width for doorway classification (default: 0.8m)
- `narrow_hallway_threshold`: Maximum width for narrow hallway (default: 1.2m)
- `room_detection_threshold`: Minimum space for room classification (default: 2.0m)

#### Safety Parameters
- `min_obstacle_distance`: Minimum safe distance (default: 0.20m)
- `comfortable_distance`: Preferred clearance distance (default: 0.35m)

#### Robot Dimensions
- `robot_radius`: Robot radius (default: 0.44m)
- `robot_diameter`: Robot diameter (default: 0.88m)

## States

The robot operates in the following states:

1. **IDLE**: Initial state, transitions to FIND_WALL
2. **FIND_WALL**: Looking for a wall or space to navigate
3. **ROOM_PATROL**: Patrolling room perimeter at 1m from walls
4. **HALLWAY_NAVIGATION**: Centering and navigating through hallways
5. **DOORWAY_APPROACH**: Approaching and centering in doorway
6. **DOORWAY_ENTRY**: Moving through doorway
7. **POST_DOORWAY_FORWARD**: Moving forward after clearing doorway
8. **RECOVERY_BACKUP**: Backing up when stuck
9. **RECOVERY_TURN**: Turning when stuck

## Space Types

The robot classifies spaces into:

- **ROOM**: Large open spaces (>2m front clearance)
- **HALLWAY**: Narrow passages (1.2-2m width)
- **DOORWAY**: Very narrow passages (<0.8m width)
- **VERY_NARROW**: Extremely tight spaces (<0.6m width)
- **UNKNOWN**: Unable to classify

## Safety Features

- **Collision Detection**: Monitors front, left, and right distances
- **Stuck Detection**: Detects when robot is not moving
- **Recovery Behaviors**: Automatic backup and turn when stuck
- **Conservative Speeds**: Slow, safe speeds for house navigation
- **Distance Maintenance**: Keeps safe distances from walls and obstacles

## Visualization

The package publishes visualization markers for RViz:

- **Space Type Marker**: Shows current space classification above robot
- **State Information**: Available via `/roamer_state` topic

To view in RViz, add a MarkerArray display and subscribe to `/roamer_markers`.

## Customization

### Adding New Sensors

The package is designed to be easily extensible. To add new sensors:

1. Add sensor subscribers in `_setup_communication()`
2. Implement sensor processing methods
3. Integrate sensor data into space classification or safety checks

### Modifying Navigation Behavior

- **Room Patrol**: Modify `get_room_patrol_command()`
- **Hallway Navigation**: Modify `get_hallway_centering_command()`
- **Doorway Handling**: Modify `approach_doorway()` and `enter_doorway()`

### Parameter Tuning

For different environments, adjust:

- **Speed parameters** for different robot capabilities
- **Distance thresholds** for different space sizes
- **Safety parameters** for different risk tolerances
- **Classification thresholds** for different building layouts

## Troubleshooting

### Common Issues

1. **Robot not moving**: Check LIDAR and odometry topics are publishing
2. **Collision warnings**: Adjust `min_obstacle_distance` parameter
3. **Stuck in recovery**: Check for physical obstacles or sensor issues
4. **Poor space classification**: Adjust classification thresholds in config
5. **Frame transformation errors**: Check that TF is publishing and frames are correct
6. **Simulation time issues**: Ensure `use_sim_time` parameter is set correctly
7. **Clock type mismatches**: Verify clock type detection matches your environment

### Debugging

Enable debug output by setting log level:
```bash
ros2 run perimeter_roamer_v2 perimeter_roamer --ros-args --log-level debug
```

Monitor state changes and space classification via published topics.

## License

MIT License - see LICENSE file for details.

## Maintainer

Michael Wimble (mike@wimblerobotics.com) 