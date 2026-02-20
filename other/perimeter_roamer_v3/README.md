# Perimeter Roamer V3

A sophisticated house patrolling robot system for ROS 2 Jazzy that uses Behavior Trees to intelligently navigate through different types of spaces (rooms, hallways, doorways) while avoiding obstacles and managing battery levels.

## Overview

This package implements a comprehensive perimeter roaming behavior for a house robot using BehaviorTree.CPP. The system is designed for a cylindrical robot with ~0.44m radius that needs to navigate through narrow doorways and hallways in a residential environment.

### Key Features

- **Intelligent Space Classification**: Analyzes LIDAR data to classify current space as Room, Hallway, Doorway, or Very Narrow passage
- **Wall Database Integration**: Uses SQLite database of house walls for enhanced localization and navigation planning
- **Adaptive Navigation**: Different navigation strategies based on space type:
  - **Rooms**: Patrol edges while maintaining 1m+ distance from walls
  - **Hallways**: Stay centered in corridor
  - **Doorways**: Careful alignment and passage through narrow openings
  - **Very Narrow**: Ultra-cautious movement in constrained spaces
- **Battery Management**: Monitors battery state and returns to charging when needed
- **LIDAR Health Monitoring**: Detects and handles sensor failures or excessive invalid readings
- **Nav2 Integration**: Uses Navigation2 stack for path planning and execution
- **Robust Error Handling**: Behavior tree structure allows graceful failure recovery

## Architecture

The system uses a hierarchical Behavior Tree with the following main branches:

1. **Battery Check Sequence**: Monitors battery and initiates charging when needed
2. **Patrol Sequence**: 
   - Checks LIDAR health
   - Classifies current space
   - Executes appropriate navigation strategy

### Behavior Tree Nodes

#### Condition Nodes
- `CheckBatteryState`: Monitors battery percentage and triggers charging behavior
- `CheckLidarHealth`: Validates LIDAR data quality and availability
- `IsRoom/IsHallway/IsDoorway/IsVeryNarrow`: Space type classification conditions

#### Action Nodes
- `ClassifySpace`: Analyzes LIDAR data and wall database to determine space type
- `NavigateToPose`: Interfaces with Nav2 to execute movement commands

## Building

To build this package, navigate to your ROS 2 workspace root and run:

```bash
colcon build --packages-select perimeter_roamer_v3 --symlink-install
```

## Running

After building, source your workspace:

```bash
source install/setup.bash
```

Then, launch the node:

```bash
ros2 launch perimeter_roamer_v3 perimeter_roamer_launch.py
```

For simulation with Gazebo:

```bash
ros2 launch perimeter_roamer_v3 perimeter_roamer_launch.py use_sim_time:=true
```