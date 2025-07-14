# Wall Finder Package

A ROS2 node that detects and extracts wall segments from occupancy grid maps using edge detection and Hough line transforms.

## Features

- Detects wall edges (not centers) from static occupancy grid maps
- Wall-aware coalescing that prevents merging edges from opposite sides of walls
- Handles walls of typical thickness (4-8 inches)
- Stores detected walls in SQLite database
- Publishes detected walls as costmap for RViz2 visualization
- Configurable parameters for different environments

## Installation

1. Build the package:
   ```bash
   cd /path/to/your/ros2_workspace
   colcon build --packages-select wall_finder
   source install/setup.bash
   ```

## Usage

### Using Launch Files (Recommended)

#### Basic usage:
```bash
ros2 launch wall_finder wall_finder_simple.launch.py
```

#### With custom configuration:
```bash
ros2 launch wall_finder wall_finder_launch.py config_file:=/path/to/your/config.yaml
```

#### With RViz2 visualization:
```bash
ros2 launch wall_finder wall_finder_with_viz.launch.py use_rviz:=true
```

#### With custom parameters:
```bash
ros2 launch wall_finder wall_finder_launch.py \
    costmap_topic:=/your_costmap_topic \
    database_path:=/path/to/walls.db \
    publish_walls_costmap:=true
```

### Using ros2 run (Alternative)

```bash
ros2 run wall_finder wall_finder
```

## Configuration

The node can be configured using the YAML file in `config/wall_finder.yaml`. Key parameters include:

- `costmap_topic`: Input costmap topic (default: `/global_costmap/static_layer`)
- `costmap_threshold`: Threshold for wall detection (0-100%, default: 50)
- `min_wall_length_meters`: Minimum wall length to detect (default: 0.051m / 2 inches)
- `hough_threshold`: Hough transform sensitivity (default: 30)
- `angle_tolerance`: Tolerance for merging parallel walls (default: 10 degrees)
- `distance_tolerance`: Distance tolerance for wall coalescing (default: 0.2m)
- `proximity_tolerance`: Endpoint proximity tolerance (default: 0.15m)

## Topics

### Subscribed Topics
- `/global_costmap/static_layer` (nav_msgs/OccupancyGrid): Input occupancy grid

### Published Topics
- `/walls_costmap` (nav_msgs/OccupancyGrid): Detected walls as costmap for visualization

## Database

The node creates a SQLite database (`walls.db` by default) with the following schema:

```sql
CREATE TABLE walls (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    start_x REAL NOT NULL,
    start_y REAL NOT NULL,
    end_x REAL NOT NULL,
    end_y REAL NOT NULL,
    length REAL NOT NULL,
    angle REAL NOT NULL,
    room_name TEXT DEFAULT 'undefined',
    wall_name TEXT DEFAULT 'undefined'
);
```

## Algorithm

1. **Edge Detection**: Uses Canny edge detection to find wall boundaries
2. **Line Detection**: Applies Hough line transform to detect straight wall segments
3. **Wall-Aware Coalescing**: Merges nearby parallel segments while preventing cross-wall merging
4. **Filtering**: Removes segments shorter than minimum length threshold

## Launch File Options

### wall_finder_simple.launch.py
- Minimal launch file with hardcoded default parameters
- Good for quick testing

### wall_finder_launch.py
- Full-featured launch file with configurable parameters
- Loads configuration from YAML file
- Supports command-line parameter overrides

### wall_finder_with_viz.launch.py
- Includes optional RViz2 visualization
- Use `use_rviz:=true` to start RViz2 automatically

## Visualization in RViz2

To visualize the detected walls in RViz2:

1. Add a Map display
2. Set the topic to `/walls_costmap`
3. The detected walls will appear as occupied cells

## Troubleshooting

- **No walls detected**: Try lowering `costmap_threshold` or `hough_threshold`
- **Too many short segments**: Increase `min_wall_length_meters`
- **Missing wall connections**: Increase `hough_max_line_gap` or `proximity_tolerance`
- **Cross-wall merging**: Adjust `distance_tolerance` and wall thickness parameters
