# Model to Wall Database

This ROS2 package extracts wall parameters from xacro files and creates an SQLite database.

## Description

The package parses xacro files containing `xacro:wall` macro calls and extracts the following parameters:
- `width_i` - Wall width in inches (converted to meters)
- `length_in` - Wall length in inches (converted to meters)
- `x_in` - Starting X coordinate in inches (converted to meters)
- `y_in` - Starting Y coordinate in inches (converted to meters)
- `name` - Wall name

The extracted data is stored in an SQLite database at `~/sigyn_ws/src/Sigyn/description/models/sigyn_house.db`.

## Dependencies

- ROS2 Jazzy
- Python 3
- lxml (for XML parsing)
- sqlite3 (built-in Python module)

## Installation

1. Build the package:
```bash
cd ~/sigyn_ws
colcon build --packages-select model_to_wall_db
source install/setup.bash
```

## Usage

### Extract walls from xacro file and create database:
```bash
ros2 run model_to_wall_db extract_walls
```

### Query the database:
```bash
python3 ~/sigyn_ws/src/Sigyn/experiments/model_to_wall_db/scripts/query_walls.py
```

## Expected xacro format

The script expects xacro wall macro calls in the following format:
```xml
<xacro:wall name="wall_name" width_i="6" length_in="120" x_in="0" y_in="0" />
```

or with expressions:
```xml
<xacro:wall name="wall_name" width_i="6" length_in="${10*12}" x_in="0" y_in="0" />
```

## Database Schema

The SQLite database contains a table called `walls` with the following fields:
- `id` - Auto-incrementing integer primary key
- `name` - Wall name (TEXT)
- `width` - Wall width in meters (REAL)
- `length` - Wall length in meters (REAL)
- `x` - Starting X coordinate in meters (REAL)
- `y` - Starting Y coordinate in meters (REAL)

## Files

- `extract_walls.py` - Main script to extract walls and create database
- `scripts/query_walls.py` - Utility script to query and display database contents
- `package.xml` - ROS2 package manifest
- `setup.py` - Python package setup

## Notes

- The script converts all inch measurements to meters using the conversion factor 0.0254
- Simple mathematical expressions in parameter values (e.g., `${10*12}`) are evaluated
- The database is recreated each time the extraction script is run
- Missing parameters will cause a wall to be skipped with a warning message
