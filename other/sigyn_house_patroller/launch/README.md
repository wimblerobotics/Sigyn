# Sigyn House Patroller Launch Files

This directory contains individual launch files for each node in the sigyn_house_patroller package, allowing you to launch nodes independently or together.

## Individual Node Launch Files

### 1. Battery Monitor
```bash
ros2 launch sigyn_house_patroller battery_monitor.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Log level (default: info)
- `config_file`: Path to battery monitor config file

**Key Features:**
- Monitors battery state and sends alerts
- Provides battery level predictions
- Configurable thresholds for critical and low battery

### 2. Temperature Monitor
```bash
ros2 launch sigyn_house_patroller temperature_monitor.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Log level (default: info)
- `config_file`: Path to temperature monitor config file

**Key Features:**
- Monitors temperature anomalies
- Room-specific baseline temperatures
- Trend analysis and alerting

### 3. Door Monitor
```bash
ros2 launch sigyn_house_patroller door_monitor.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Log level (default: info)
- `config_file`: Path to door monitor config file

**Key Features:**
- Monitors door state changes using LIDAR
- Configurable door locations and expected states
- Room-aware monitoring

### 4. Change Detector
```bash
ros2 launch sigyn_house_patroller change_detector.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Log level (default: info)
- `config_file`: Path to change detector config file

**Key Features:**
- Visual and 3D change detection
- Reference model learning and updating
- Configurable detection thresholds

### 5. Behavior Tree Manager
```bash
ros2 launch sigyn_house_patroller behavior_tree_manager.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Log level (default: info)
- `config_file`: Path to behavior tree manager config file
- `behavior_tree_dir`: Directory containing behavior tree XML files

**Key Features:**
- Manages behavior tree switching based on system state
- Integrates with Nav2 stack
- Threat-responsive behavior adaptation

### 6. Threat Response
```bash
ros2 launch sigyn_house_patroller threat_response.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Log level (default: info)
- `config_file`: Path to threat response config file

**Key Features:**
- Automated threat response and investigation
- Configurable response waypoints
- Emergency stop functionality

### 7. Localization Corrector
```bash
ros2 launch sigyn_house_patroller localization_corrector.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Log level (default: info)
- `config_file`: Path to localization corrector config file

**Key Features:**
- Monitors localization health
- Automatic localization correction
- Pose jump detection and recovery

### 8. Patrol Coordinator
```bash
ros2 launch sigyn_house_patroller patrol_coordinator.launch.py
```

**Parameters:**
- `use_sim_time`: Use simulation time (default: false)
- `log_level`: Log level (default: info)
- `config_file`: Path to patrol config file
- `waypoint_file`: Path to waypoint config file
- `room_file`: Path to room config file

**Key Features:**
- Main patrol coordination and waypoint management
- Navigation integration
- System health monitoring

## Launch All Nodes

To launch all nodes together with individual control:

```bash
ros2 launch sigyn_house_patroller all_nodes.launch.py
```

### Selective Node Launching

You can enable/disable individual nodes:

```bash
ros2 launch sigyn_house_patroller all_nodes.launch.py \
    enable_battery_monitor:=true \
    enable_temperature_monitor:=true \
    enable_door_monitor:=false \
    enable_change_detector:=true \
    enable_behavior_tree_manager:=false \
    enable_threat_response:=true \
    enable_localization_corrector:=true \
    enable_patrol_coordinator:=true
```

### Common Parameters

All launch files support these common parameters:

- `use_sim_time`: Set to true when using simulation (default: false)
- `log_level`: ROS2 log level (debug, info, warn, error, fatal)

### Example Usage

#### Development and Testing
```bash
# Launch only monitoring nodes for testing
ros2 launch sigyn_house_patroller all_nodes.launch.py \
    enable_patrol_coordinator:=false \
    enable_behavior_tree_manager:=false \
    log_level:=debug
```

#### Production Deployment
```bash
# Launch all nodes for full system operation
ros2 launch sigyn_house_patroller all_nodes.launch.py \
    log_level:=info
```

#### Simulation Mode
```bash
# Launch with simulation time
ros2 launch sigyn_house_patroller all_nodes.launch.py \
    use_sim_time:=true
```

## Configuration Files

Each node expects its configuration file in the `config/` directory:

- `battery_monitor.yaml`
- `temperature_monitor.yaml`
- `door_monitor.yaml`
- `change_detector.yaml`
- `behavior_tree_manager.yaml`
- `threat_response.yaml`
- `localization_corrector.yaml`
- `patrol_config.yaml`
- `waypoints.yaml`
- `rooms.yaml`

## Topic Remapping

The launch files include default topic remappings. You can override these by modifying the launch files or using ROS2 remapping syntax:

```bash
ros2 launch sigyn_house_patroller battery_monitor.launch.py \
    --ros-args --remap /battery_state:=/my_robot/battery_state
```

## Dependencies

Make sure you have the following dependencies installed:

- ROS2 (Humble or later)
- Nav2 stack
- BehaviorTree.CPP
- OpenCV (for change detection)
- PCL (for point cloud processing)

## Troubleshooting

1. **Node fails to start**: Check that all dependencies are installed and sourced
2. **Configuration not loading**: Verify config file paths are correct
3. **Topics not publishing**: Check topic remappings match your robot's topic names
4. **Permission errors**: Ensure launch files are executable (`chmod +x *.launch.py`)

## Integration with Existing Systems

These launch files are designed to integrate with your existing robot system. Key integration points:

- **Navigation**: Integrates with Nav2 stack
- **Sensors**: Subscribes to standard sensor topics
- **Localization**: Works with AMCL or other localization systems
- **Mapping**: Compatible with SLAM or pre-built maps
