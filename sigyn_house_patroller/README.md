# Sigyn House Patroller

A comprehensive ROS2 package for autonomous house monitoring and security patrol using behavior trees, advanced threat detection, and intelligent navigation.

## Overview

The Sigyn House Patroller is an advanced autonomous security system designed for indoor surveillance and monitoring. It combines sophisticated navigation, threat detection, and notification systems to provide comprehensive house security coverage.

### Key Features

- **Autonomous Navigation**: Systematic room-by-room patrol with optimal path planning
- **Multi-Modal Threat Detection**: Battery monitoring, temperature anomalies, door state changes, and motion detection
- **Behavior Tree Architecture**: Robust decision-making with fallback behaviors
- **Real-time Notifications**: Email alerts for different threat severity levels
- **Room-Aware Navigation**: Intelligent waypoint management with room identification
- **Recovery Behaviors**: Automatic recovery from navigation failures and stuck situations
- **Comprehensive Logging**: Detailed system health monitoring and performance metrics

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        PatrolManager                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  WaypointManager │  │NavigationCoord  │  │ThreatDetection  │  │
│  │                 │  │                 │  │Manager          │  │
│  │ • Room mapping  │  │ • Nav2 client   │  │ • Battery       │  │
│  │ • Waypoint opt  │  │ • Path planning │  │ • Temperature   │  │
│  │ • Sequence gen  │  │ • Recovery      │  │ • Door state    │  │
│  └─────────────────┘  └─────────────────┘  │ • Change detect │  │
│                                            └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
         │                    │                         │
         ▼                    ▼                         ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│   Nav2 Stack    │  │   Sensor Data   │  │  Notifications  │
│                 │  │                 │  │                 │
│ • AMCL          │  │ • LIDAR         │  │ • Email alerts  │
│ • Costmaps      │  │ • Camera        │  │ • ROS topics    │
│ • Planners      │  │ • IMU           │  │ • Logging       │
│ • Controllers   │  │ • Battery       │  │                 │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

## Installation

### Prerequisites

- ROS2 Jazzy
- Navigation2 (Nav2) stack
- BehaviorTree.CPP v3
- PCL (Point Cloud Library)
- OpenCV
- sqlite3

### Dependencies

Install required system dependencies:

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-msgs \
    ros-jazzy-behaviortree-cpp-v3 \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    libsqlite3-dev \
    libeigen3-dev \
    libjsoncpp-dev \
    libyaml-cpp-dev
```

### Build Instructions

1. Clone the repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository_url> sigyn_house_patroller
```

2. Install dependencies using rosdep:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:

```bash
colcon build --packages-select sigyn_house_patroller --symlink-install
```

4. Source the workspace:

```bash
source install/setup.bash
```

## Configuration

### Waypoint Configuration

Edit `config/waypoints.yaml` to define patrol waypoints for your house layout:

```yaml
waypoints:
  living_room_center:
    id: "living_room_center"
    room_name: "living_room"
    waypoint_type: "navigation"
    priority: 1.0
    dwell_time: 5
    pose:
      position: {x: 2.0, y: 1.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    required_sensors: ["camera", "lidar"]
```

### Room Configuration

Edit `config/rooms.yaml` to define room boundaries and characteristics:

```yaml
rooms:
  living_room:
    name: "Living Room"
    type: "living_room"
    center: {x: 2.0, y: 1.0, z: 0.0}
    area: 20.0
    is_accessible: true
    typical_threats: ["motion_detection", "temperature_anomaly"]
    normal_conditions:
      temperature: 22.0
      humidity: 45.0
```

### Patrol Configuration

Edit `config/patrol_config.yaml` to customize patrol behavior:

```yaml
patrol_manager:
  patrol_timeout: 3600  # seconds
  waypoint_tolerance: 0.2  # meters
  navigation_timeout: 300  # seconds
  
  # Email notifications
  email_notifications:
    enabled: true
    smtp_server: "smtp.gmail.com"
    smtp_port: 587
    sender_email: "sigyn_robot@example.com"
    recipient_email: "owner@example.com"
```

## Usage

### Basic Launch

Launch the complete patrol system:

```bash
ros2 launch sigyn_house_patroller patrol_manager.launch.py
```

### Launch with Custom Configuration

```bash
ros2 launch sigyn_house_patroller patrol_manager.launch.py \
    config_file:=/path/to/your/config.yaml \
    waypoint_file:=/path/to/your/waypoints.yaml \
    room_file:=/path/to/your/rooms.yaml
```

### Service Interface

#### Set Patrol Mode

```bash
# Start full house patrol
ros2 service call /sigyn_house_patroller/set_patrol_mode \
    sigyn_house_patroller/srv/SetPatrolMode "{mode: 1}"

# Set to idle mode
ros2 service call /sigyn_house_patroller/set_patrol_mode \
    sigyn_house_patroller/srv/SetPatrolMode "{mode: 0}"
```

#### Get Room Information

```bash
ros2 service call /sigyn_house_patroller/get_room_info \
    sigyn_house_patroller/srv/GetRoomInfo "{room_name: 'living_room'}"
```

### Action Interface

#### Patrol to Specific Waypoint

```bash
ros2 action send_goal /sigyn_house_patroller/patrol_to_waypoint \
    sigyn_house_patroller/action/PatrolToWaypoint "{waypoint_id: 'living_room_center'}"
```

### Topic Monitoring

#### Monitor Threat Alerts

```bash
ros2 topic echo /sigyn_house_patroller/threat_alerts
```

#### Monitor Patrol Status

```bash
ros2 topic echo /sigyn_house_patroller/patrol_status
```

#### Monitor System Health

```bash
ros2 topic echo /sigyn_house_patroller/system_health
```

### Emergency Stop

```bash
ros2 topic pub /sigyn_house_patroller/emergency_stop std_msgs/msg/Bool "data: true"
```

## Message Types

### ThreatAlert

```yaml
header: std_msgs/Header
threat_id: string
threat_type: string
severity: int32
location: geometry_msgs/Point
description: string
confidence: float64
timestamp: builtin_interfaces/Time
sensor_data: string
```

### PatrolStatus

```yaml
header: std_msgs/Header
mode: int32
is_navigating: bool
current_waypoint: string
patrol_progress: float64
estimated_completion_time: builtin_interfaces/Time
```

### SystemHealth

```yaml
header: std_msgs/Header
overall_health: float64
navigation_health: float64
detection_health: float64
component_status: string[]
```

## Threat Detection

### Battery Monitoring

- **Critical Level**: Below 15% battery
- **Low Level**: Below 25% battery
- **Actions**: Automatic charging station navigation

### Temperature Monitoring

- **Anomaly Detection**: Deviation from room baseline
- **Threshold**: 5°C difference from normal
- **Actions**: Increased monitoring frequency

### Door State Monitoring

- **LIDAR-based**: Distance measurement to door positions
- **Expected States**: Configurable open/closed expectations
- **Actions**: Immediate alert and investigation

### Motion/Change Detection

- **Visual Comparison**: Reference model vs. current state
- **Point Cloud Analysis**: 3D environment changes
- **Actions**: Proximity investigation and alerting

## Behavior Trees

The system uses behavior trees for high-level decision making:

```
PatrolCoordinator
├── EmergencyCheck
│   ├── BatteryCheck
│   ├── ThreatResponse
│   └── EmergencyStop
├── PatrolSequence
│   ├── WaypointNavigation
│   ├── RoomInspection
│   └── ThreatDetection
└── RecoveryBehaviors
    ├── NavigationRecovery
    ├── SensorRecovery
    └── SystemRestart
```

## Integration with Existing Systems

### Perimeter Roamer Integration

The house patroller integrates with your existing `perimeter_roamer` system:

```cpp
#include "perimeter_roamer/perimeter_roamer.hpp"
#include "sigyn_house_patroller/core/patrol_manager.hpp"
```

### Wall Database Integration

Leverages existing wall database for navigation:

```cpp
#include "wall_finder/wall_database.hpp"
```

## Troubleshooting

### Navigation Issues

1. **Robot gets stuck**: Check `stuck_threshold` and `stuck_timeout` parameters
2. **Navigation timeout**: Increase `navigation_timeout` in config
3. **Path planning failures**: Verify costmap configuration

### Threat Detection Issues

1. **False positives**: Adjust `anomaly_threshold` values
2. **Missing detections**: Check sensor topics and data flow
3. **Email notifications not working**: Verify SMTP configuration

### Performance Issues

1. **High CPU usage**: Reduce `threat_detection_frequency`
2. **Memory leaks**: Check `max_pose_history` setting
3. **Slow navigation**: Optimize waypoint spacing

## Development

### Adding New Threat Detectors

1. Inherit from `ThreatDetector` base class
2. Implement required virtual methods
3. Register in `ThreatDetectionManager`

```cpp
class CustomThreatDetector : public ThreatDetector {
public:
  explicit CustomThreatDetector();
  bool Initialize(rclcpp::Node::SharedPtr node) override;
  std::vector<msg::ThreatAlert> CheckForThreats() override;
  bool IsHealthy() const override;
};
```

### Adding New Waypoint Types

1. Define in `waypoint_manager.hpp`
2. Add handling in `WaypointManager`
3. Update configuration schema

### Custom Behavior Trees

1. Create behavior tree XML files
2. Implement custom action nodes
3. Register with BehaviorTree.CPP

## Contributing

1. Follow Google C++ Style Guide
2. Add unit tests for new features
3. Update documentation
4. Test with real robot hardware

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
- Create GitHub issues for bugs
- Use discussions for questions
- Check existing documentation

## Changelog

### v1.0.0
- Initial release with core patrol functionality
- Multi-modal threat detection
- Behavior tree integration
- Email notification system
- Comprehensive configuration system
