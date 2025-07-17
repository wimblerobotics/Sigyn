# Wall Finder

A ROS2 package for detecting walls from occupancy grid maps using computer vision techniques.

## Overview

The `wall_finder` package processes occupancy grid data from the global costmap and uses the Hough Line Transform algorithm to detect wall segments. It identifies contiguous obstacle points that form lines, filters them based on configurable minimum length requirements, and stores the results in an SQLite database.

## Features

- **Occupancy Grid Processing**: Converts occupancy grid data to binary images for line detection
- **Hough Line Transform**: Uses OpenCV's probabilistic Hough line detection for robust wall identification
- **Wall Segment Merging**: Combines nearby parallel line segments into single wall segments
- **Length Filtering**: Filters walls based on configurable minimum length (default: 5 inches)
- **Database Storage**: Stores wall data in SQLite database with configurable location
- **90-degree Wall Detection**: Optimized for typical indoor environments with perpendicular walls

## Dependencies

- ROS2 Jazzy
- OpenCV 4
- SQLite3
- yaml-cpp
- nav_msgs
- geometry_msgs
- tf2

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/your_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select wall_finder
```

## Usage

### Launch the Wall Finder

```bash
ros2 launch wall_finder wall_finder.launch.py
```

### Manual Execution

```bash
ros2 run wall_finder wall_finder --config /path/to/config.yaml
```

## Configuration

The wall finder behavior can be configured through the `config/wall_finder.yaml` file:

```yaml
wall_finder:
  ros__parameters:
    # Input parameters
    costmap_topic: "/global_costmap/costmap"
    costmap_threshold: 50
    
    # Wall detection parameters
    min_wall_length_inches: 5.0
    min_wall_length_meters: 0.127
    
    # Hough line detection parameters
    hough_rho: 1.0
    hough_theta: 0.017453292519943295  # 1 degree
    hough_threshold: 50
    hough_min_line_length: 25
    hough_max_line_gap: 10
    
    # Line merging parameters
    angle_tolerance: 0.087266462599716477  # 5 degrees
    distance_tolerance: 0.1  # 10 cm
    
    # Database parameters
    database_path: "walls.db"
    
    # TF frame
    map_frame: "map"
```

### Key Parameters

- `costmap_threshold`: Threshold for converting occupancy probabilities to binary obstacles (0-100)
- `min_wall_length_meters`: Minimum wall length in meters to be considered valid
- `hough_threshold`: Minimum number of votes in Hough space for line detection
- `hough_min_line_length`: Minimum line length in pixels for initial detection
- `hough_max_line_gap`: Maximum gap between line segments to connect them
- `angle_tolerance`: Maximum angle difference for merging parallel lines
- `distance_tolerance`: Maximum distance for merging nearby parallel lines

## Database Schema

The wall data is stored in an SQLite database with the following schema:

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

### Database Fields

- `id`: Unique identifier for each wall segment
- `start_x`, `start_y`: Starting point coordinates in the map frame
- `end_x`, `end_y`: Ending point coordinates in the map frame
- `length`: Wall length in meters
- `angle`: Wall angle in radians relative to the x-axis
- `room_name`: Room identifier (currently set to "undefined")
- `wall_name`: Wall identifier (currently set to "undefined")

## Algorithm Details

### Wall Detection Process

1. **Occupancy Grid Conversion**: Convert occupancy grid to grayscale image
2. **Thresholding**: Apply binary threshold to identify obstacles
3. **Morphological Operations**: Clean up noise using closing operations
4. **Edge Detection**: Apply Canny edge detection to find obstacle boundaries
5. **Hough Line Detection**: Use probabilistic Hough transform to detect line segments
6. **Segment Merging**: Combine nearby parallel segments into single walls
7. **Length Filtering**: Remove walls shorter than minimum length
8. **Database Storage**: Store wall data in SQLite database

### Line Detection Algorithm

The package uses OpenCV's `HoughLinesP` (Probabilistic Hough Line Transform) which:
- Directly returns line segments with start and end points
- Is more efficient than standard Hough transform for this application
- Provides better results for detecting finite line segments in noisy environments

### Wall Merging Strategy

- Lines with similar angles (within `angle_tolerance`) are candidates for merging
- Lines must be within `distance_tolerance` of each other
- Merged segments span the full extent of the combined segments
- Process continues iteratively until no more merges are possible

## Integration with Gazebo

The node is designed to work with Gazebo simulation environments:

1. Launch Gazebo with your world file
2. Ensure the global costmap is being published to `/global_costmap/costmap`
3. Launch the wall finder node
4. The node will process the costmap once and then shutdown

## Future Enhancements

- Room detection and wall naming
- Support for curved walls
- Integration with SLAM for dynamic wall detection
- Wall classification (interior vs. exterior)
- Support for multi-floor environments

## Troubleshooting

### Common Issues

1. **No walls detected**: Check costmap threshold and Hough parameters
2. **Too many false positives**: Increase `hough_threshold` or `min_wall_length_meters`
3. **Walls too fragmented**: Decrease `hough_max_line_gap` or increase `distance_tolerance`
4. **Database errors**: Ensure write permissions for database file location

### Debug Tips

- Enable debug logging to see detected line counts
- Visualize intermediate images using OpenCV's `imshow` for development
- Check costmap data quality in RViz
- Verify coordinate transformations between pixel and world coordinates

## License

MIT License - See LICENSE file for details.
