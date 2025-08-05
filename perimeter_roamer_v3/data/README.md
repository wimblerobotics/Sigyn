# Data Directory

This directory contains data files used by the perimeter_roamer_v3 package.

## Files

- `patrol_waypoints.db` - SQLite database containing waypoints captured from the `/waypoints` topic
- This database is automatically created when you run the waypoint capture system

## Database Schema

The waypoints database contains a single table `waypoints` with the following columns:

- `id` (INTEGER) - Unique waypoint identifier
- `x_pose` (REAL) - X position in meters
- `y_pose` (REAL) - Y position in meters  
- `z_pose` (REAL) - Z position in meters
- `x_orientation` (REAL) - Quaternion X component
- `y_orientation` (REAL) - Quaternion Y component
- `z_orientation` (REAL) - Quaternion Z component
- `w_orientation` (REAL) - Quaternion W component
- `euler_orientation` (REAL) - Yaw angle in radians
- `text` (TEXT) - Waypoint name/description
- `created_at` (TIMESTAMP) - Creation timestamp

## Usage

The waypoint following system will automatically load waypoints from this database when launched with:

```bash
ros2 launch perimeter_roamer_v3 patrol_using_waypoints_launch.py
```

You can specify a different database path with:

```bash
ros2 launch perimeter_roamer_v3 patrol_using_waypoints_launch.py waypoint_database_path:=/path/to/your/waypoints.db
```
