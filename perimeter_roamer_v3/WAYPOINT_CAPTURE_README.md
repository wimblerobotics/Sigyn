# Waypoint Capture System

This package includes a waypoint capture system that reads waypoints from the `/waypoints` topic and stores them in a SQLite database.

## Files Created

1. **`scripts/capture_waypoints.py`** - Main Python script that captures waypoints
2. **`launch/capture_waypoints_launch.py`** - Launch file for the waypoint capture node
3. **Database**: `~/sigyn_ws/patrol_waypoints.db` - SQLite database storing captured waypoints

## Database Schema

The SQLite database contains a single table `waypoints` with the following schema:

| Column | Type | Description |
|--------|------|-------------|
| id | INTEGER | Primary key (waypoint ID from marker) |
| x_pose | REAL | X position from pose.position.x |
| y_pose | REAL | Y position from pose.position.y |
| z_pose | REAL | Z position from pose.position.z |
| x_orientation | REAL | X component of quaternion |
| y_orientation | REAL | Y component of quaternion |
| z_orientation | REAL | Z component of quaternion |
| w_orientation | REAL | W component of quaternion |
| euler_orientation | REAL | Yaw angle in radians (computed from quaternion) |
| text | TEXT | Text field from marker |
| created_at | TIMESTAMP | Creation timestamp |

## Usage

### Launch the waypoint capture node:
```bash
ros2 launch perimeter_roamer_v3 capture_waypoints_launch.py
```

### Run directly:
```bash
ros2 run perimeter_roamer_v3 capture_waypoints.py
```

### With custom log level:
```bash
ros2 launch perimeter_roamer_v3 capture_waypoints_launch.py log_level:=debug
```

## Features

- **Selective Capture**: Only captures markers with `type=9` (TEXT_VIEW_FACING)
- **Database Management**: Automatically creates/clears database on startup
- **Quaternion Conversion**: Automatically converts quaternions to euler yaw angles
- **Unique ID Handling**: Uses INSERT OR REPLACE to handle duplicate waypoint IDs
- **ROS 2 Integration**: Full ROS 2 node with proper lifecycle management

## Dependencies

The following dependencies are automatically installed:
- `visualization_msgs` - For MarkerArray message type
- `python3-tf-transformations` - For quaternion to euler conversion
- `sqlite3` - For database operations

## Database Location

The database file `patrol_waypoints.db` is created in the `~/sigyn_ws` directory and is cleared/recreated each time the script runs.

## Example Query

To view captured waypoints:
```sql
sqlite3 ~/sigyn_ws/patrol_waypoints.db "SELECT id, x_pose, y_pose, ROUND(DEGREES(euler_orientation), 1) as yaw_degrees, text FROM waypoints ORDER BY id;"
```
