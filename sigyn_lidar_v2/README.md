# Sigyn LIDAR v2

A ROS2 package for multi-LIDAR sensor management with motion correction support, specifically designed for the Sigyn robot.

## Features

- **Multi-LIDAR Support**: Handle multiple LIDAR sensors simultaneously
- **Modular Driver Architecture**: Easy to add support for new LIDAR types
- **LD06 Driver**: Based on official LDROBOT vendor implementation with proper CRC verification
- **Scan Fusion**: Combine multiple LIDAR scans into a single unified scan
- **Motion Correction**: Compensate for robot motion during scan acquisition (future implementation)
- **Proper Intensity Data**: Preserve and publish intensity values (vendor calls this "confidence")
- **450-Ray Output**: Matches vendor expectation for LD06 scanners

## Supported LIDAR Types

- **LD06**: LDROBOT LD06 360° LiDAR (fully implemented)
- **LD09**: LDROBOT LD09 (planned)
- Additional types can be easily added through the modular driver interface

## Package Structure

```
sigyn_lidar_v2/
├── include/sigyn_lidar_v2/
│   ├── lidar_types.hpp          # Common data structures
│   ├── lidar_base.hpp           # Base driver interface
│   ├── ld06_driver.hpp          # LD06 specific implementation
│   ├── lidar_fusion.hpp         # Multi-LIDAR fusion
│   ├── motion_corrector.hpp     # Motion compensation
│   └── multi_lidar_node.hpp     # Main node class
├── src/
│   ├── lidar_base.cpp
│   ├── ld06_driver.cpp          # Vendor-based LD06 implementation
│   ├── lidar_fusion.cpp
│   ├── motion_corrector.cpp
│   ├── multi_lidar_node.cpp
│   └── main.cpp
├── config/
│   └── multi_lidar_config.yaml  # Configuration file
├── launch/
│   └── multi_lidar.launch.py    # Launch file
└── README.md
```

## Configuration

The package is configured through `config/multi_lidar_config.yaml`:

```yaml
multi_lidar_node:
  ros__parameters:
    # LIDAR devices
    device_paths: ["/dev/lidar_front_center"]
    frame_ids: ["lidar_frame_top_lidar"]
    topic_names: ["/sigyn/lidar_front_center"]
    device_types: ["LD06"]
    
    # Fusion settings
    fused_topic_name: "/scan"
    fusion_frequency_hz: 10.0
    
    # 450-ray configuration (matching vendor)
    angle_increment: 0.013962634015954636  # ~0.8 degrees
```

### Key Configuration Parameters

- **device_paths**: USB device paths (e.g., `/dev/lidar_front_center`)
- **frame_ids**: ROS frame IDs for each LIDAR (from URDF)
- **topic_names**: Individual LIDAR topic names
- **device_types**: LIDAR types ("LD06", etc.)
- **fused_topic_name**: Combined scan topic (default: `/scan`)
- **fusion_frequency_hz**: Rate for publishing fused scans (default: 10 Hz)

## Usage

### Launch the Multi-LIDAR Node

```bash
ros2 launch sigyn_lidar_v2 multi_lidar.launch.py
```

### With Custom Configuration

```bash
ros2 launch sigyn_lidar_v2 multi_lidar.launch.py config_file:=/path/to/your/config.yaml
```

### Published Topics

- **Individual LIDAR**: `/sigyn/lidar_front_center` (configurable per device)
- **Fused Scan**: `/scan` (default, configurable)

### Subscribed Topics (for motion correction)

- **Command Velocity**: `/cmd_vel`
- **Wheel Odometry**: `/sigyn/wheel_odom`
- **IMU Data**: `/imu/data`

## LD06 Implementation Details

The LD06 driver is based on the official LDROBOT vendor implementation:

- **Packet Format**: 47-byte packets with header `0x54`
- **CRC Verification**: Full CRC8 validation using vendor lookup table
- **Angle Interpolation**: Proper interpolation between start/end angles
- **Wrap Detection**: Revolution completion detection via angle rollover
- **450-Ray Output**: Fixed 450-bin scan matching vendor expectation
- **Intensity Preservation**: Raw intensity values (0-255) preserved

### Key Features

1. **Vendor Compatibility**: Uses exact same protocol parsing as vendor
2. **CRC Validation**: Verifies packet integrity using vendor CRC8 table
3. **Proper Timing**: Accurate scan timing and frequency calculation
4. **450 Rays**: Outputs exactly 450 range measurements as expected
5. **Intensity Data**: Preserves intensity values without normalization

## Motion Correction (Future)

The architecture supports motion correction using:

- **Command Velocity**: `/cmd_vel` topic
- **Wheel Odometry**: `/sigyn/wheel_odom` topic  
- **IMU Data**: `/imu/data` topic

Motion correction compensates for robot movement during the ~100ms scan acquisition time.

## Development

### Adding New LIDAR Types

1. Create new driver class inheriting from `LidarBase`
2. Implement protocol parsing for the specific LIDAR
3. Add to `LidarDriverFactory::create_driver()`
4. Update supported types list

### Extending Motion Correction

The `MotionCorrector` class provides the framework for:
- Interpolating motion data across scan time
- Applying coordinate transformations to compensate for movement
- Integrating multiple motion sources (cmd_vel, odometry, IMU)

## Dependencies

- **ROS2 Humble** (or compatible)
- **sensor_msgs**: LaserScan message type
- **geometry_msgs**: Twist message type
- **nav_msgs**: Odometry message type
- **tf2**: Transform support

## Building

```bash
cd ~/sigyn_ws
colcon build --packages-select sigyn_lidar_v2
source install/setup.bash
```

## Troubleshooting

### Common Issues

1. **Permission Denied**: Ensure user has access to serial devices
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

2. **Device Not Found**: Check device path and permissions
   ```bash
   ls -l /dev/lidar_*
   ```

3. **CRC Errors**: Check baud rate and cable connection

4. **No Scans Published**: Verify LIDAR is spinning and has power

### Diagnostics

The node publishes diagnostic information every 5 seconds:
- Packet counts and CRC error rates
- Scan statistics
- Device status per LIDAR

## License

MIT License - Compatible with LDROBOT vendor implementation
