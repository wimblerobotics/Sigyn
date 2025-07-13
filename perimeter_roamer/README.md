# Perimeter Roamer

A ROS2 node that implements autonomous perimeter roaming behavior for mobile robots. The robot follows walls while avoiding obstacles and can navigate through doorways using LIDAR sensor data and local costmap information.

## Overview

The `perimeter_roamer` node enables a robot to autonomously explore and patrol an environment by following walls and boundaries. It uses a state machine approach to handle different navigation scenarios including wall finding, wall following, obstacle avoidance, doorway navigation, and recovery behaviors.

### Key Features

- **Wall Following**: Maintains a consistent distance from walls using LIDAR feedback
- **Obstacle Avoidance**: Uses both LIDAR scans and costmap data for collision avoidance
- **Doorway Navigation**: Detects and navigates through doorways and narrow passages
- **Recovery Behaviors**: Handles stuck situations with backup and turn maneuvers
- **Adaptive Control**: Adjusts speed and turning behavior based on environment conditions

## State Machine

The node operates using the following states:

1. **FIND_WALL**: Moves forward until a wall or obstacle is detected
2. **TURN_LEFT**: Turns left until properly aligned with a wall on the right side
3. **FOLLOW_WALL**: Follows the wall while maintaining desired distance
4. **NAVIGATE_DOORWAY**: Special handling for doorway passage
5. **RECOVERY_BACKUP**: Backs up when stuck
6. **RECOVERY_TURN**: Turns to escape stuck situations
7. **STOPPED**: Emergency stop state

## Usage

### Launch the Node

```bash
ros2 run perimeter_roamer roaming_node
```

### With Parameters

```bash
ros2 run perimeter_roamer roaming_node --ros-args -p forward_speed:=0.2 -p wall_follow_distance:=0.6
```

### Launch File Example

```xml
<launch>
  <node pkg="perimeter_roamer" exec="roaming_node" name="perimeter_roamer">
    <param name="forward_speed" value="0.15"/>
    <param name="wall_follow_distance" value="0.5"/>
    <param name="turn_speed" value="0.4"/>
    <!-- Add other parameters as needed -->
  </node>
</launch>
```

## Topics

### Subscribed Topics

- `/scan` (sensor_msgs/LaserScan): LIDAR scan data
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/local_costmap/costmap` (nav_msgs/OccupancyGrid): Local costmap for obstacle detection

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to robot
- `~/costmap_check_points` (visualization_msgs/MarkerArray): Debug markers for costmap collision checking

## Parameters

### Motion Control Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `forward_speed` | double | 0.15 | Forward speed in m/s when moving straight |
| `turn_speed` | double | 0.4 | Angular velocity in rad/s when turning |
| `max_angular_speed` | double | 0.4 | Maximum angular velocity during wall following |

### Wall Following Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wall_follow_distance` | double | 0.5 | Desired distance from wall in meters |
| `wall_dist_error_gain` | double | 0.3 | Proportional gain for wall distance error correction |
| `side_scan_angle_min_deg` | double | -80.0 | Minimum angle for right-side wall detection (degrees) |
| `side_scan_angle_max_deg` | double | -40.0 | Maximum angle for right-side wall detection (degrees) |
| `wall_presence_threshold` | double | 0.3 | Maximum deviation from desired wall distance to consider wall present |

### Obstacle Detection Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `front_obstacle_distance` | double | 0.3 | Distance threshold for front obstacle detection |
| `front_angle_width_deg` | double | 20.0 | Angular width for front obstacle scanning (degrees) |
| `costmap_check_distance` | double | 0.2 | Look-ahead distance for costmap collision checking |
| `costmap_check_points_front` | int | 3 | Number of points to check in front of robot in costmap |
| `robot_width` | double | 0.3 | Width of robot for collision checking |
| `costmap_lethal_threshold` | int | 253 | Costmap value threshold for lethal obstacles |

### Doorway Navigation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `doorway_width_threshold` | double | 0.8 | Minimum width to consider an opening as a doorway |
| `doorway_approach_distance` | double | 1.0 | Distance to look ahead for doorway detection |
| `doorway_passage_speed` | double | 0.1 | Reduced speed when navigating through doorways |
| `narrow_passage_threshold` | double | 0.6 | Width threshold for narrow passage detection |

### Recovery Behavior Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `stuck_timeout` | double | 5.0 | Time in seconds before considering robot stuck |
| `stuck_distance_threshold` | double | 0.02 | Minimum distance movement to avoid stuck detection |
| `stuck_angle_threshold_deg` | double | 5.0 | Minimum angular movement to avoid stuck detection |
| `recovery_backup_dist` | double | -0.1 | Distance to backup during recovery (negative for backward) |
| `recovery_backup_time` | double | 1.5 | Time to spend backing up during recovery |
| `recovery_turn_angle` | double | 1.57 | Angle to turn during recovery (radians, ~90 degrees) |
| `recovery_turn_time` | double | 2.0 | Maximum time for recovery turn |

### System Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `control_frequency` | double | 10.0 | Control loop frequency in Hz |
| `goal_tolerance_dist` | double | 0.1 | Distance tolerance for goal achievement |
| `robot_base_frame` | string | "base_link" | Robot base frame ID |
| `map_frame` | string | "map" | Map frame ID |
| `odom_frame` | string | "odom" | Odometry frame ID |

### Topic Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scan_topic` | string | "/scan" | LIDAR scan topic name |
| `odom_topic` | string | "/odom" | Odometry topic name |
| `cmd_vel_topic` | string | "/cmd_vel" | Velocity command topic name |
| `local_costmap_topic` | string | "/local_costmap/costmap" | Local costmap topic name |

## Tuning Guidelines

### For Stable Wall Following

- **Reduce oscillation**: Lower `wall_dist_error_gain` (try 0.2-0.4)
- **Smoother turns**: Reduce `max_angular_speed` (try 0.3-0.5)
- **Closer wall following**: Decrease `wall_follow_distance`
- **More responsive**: Increase `wall_dist_error_gain` (but may cause oscillation)

### For Better Obstacle Avoidance

- **Earlier detection**: Increase `front_obstacle_distance`
- **Wider detection**: Increase `front_angle_width_deg`
- **More conservative**: Lower `costmap_lethal_threshold`
- **Safer navigation**: Increase `costmap_check_distance`

### For Doorway Navigation

- **Detect narrower openings**: Reduce `doorway_width_threshold`
- **Slower passage**: Reduce `doorway_passage_speed`
- **Earlier detection**: Increase `doorway_approach_distance`
- **Handle tighter spaces**: Adjust `narrow_passage_threshold`

### For Recovery Behavior

- **Quicker stuck detection**: Reduce `stuck_timeout`
- **More sensitive**: Reduce `stuck_distance_threshold` and `stuck_angle_threshold_deg`
- **Longer backup**: Increase `recovery_backup_dist` (more negative) or `recovery_backup_time`
- **Bigger turns**: Increase `recovery_turn_angle`

## Dependencies

- ROS2 (tested with Humble/Iron)
- Python packages:
  - `rclpy`
  - `tf2_ros`
  - `tf_transformations`
  - `debugpy` (optional, for debugging)

## Debugging

The node includes debug markers for visualizing costmap collision check points. View these in RViz by adding a MarkerArray display subscribed to the `~/costmap_check_points` topic.

### Debug Mode

Uncomment the debugpy lines in the `main()` function to enable remote debugging:

```python
debugpy.listen(("0.0.0.0", 5678))
debugpy.wait_for_client()
```

## Troubleshooting

### Robot Oscillates While Following Wall

- Reduce `wall_dist_error_gain`
- Reduce `max_angular_speed`
- Check LIDAR noise and filtering

### Robot Gets Stuck in Doorways

- Reduce `doorway_passage_speed`
- Increase `doorway_width_threshold`
- Adjust `narrow_passage_threshold`

### Robot Doesn't Detect Walls

- Check `side_scan_angle_min_deg` and `side_scan_angle_max_deg`
- Verify `wall_presence_threshold` is appropriate
- Ensure LIDAR is publishing valid data

### Excessive Recovery Behaviors

- Increase `stuck_timeout`
- Reduce sensitivity of stuck detection parameters
- Check for mechanical issues or wheel slip

## License

[Add your license information here]

## Contributing

[Add contributing guidelines here]
