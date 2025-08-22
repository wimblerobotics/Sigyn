# sigyn_lidar

Custom LD06 (and future multi-lidar) driver for the Sigyn robot providing:

- Raw LD06 packet parsing with CRC8 validation
- Full revolution frame assembly with accurate scan_time & time_increment
- Per-beam relative timestamps (basis for motion deskew)
- Publish per-device LaserScan topics plus a fused scan topic
- Angle segmentation (future integration with MultiLidarFuser)
- Confidence (intensity) preservation for downstream docking / pattern detection

## Status
Initial parser & per-device publishing implemented. Deskew, advanced fusion, and configuration expansion pending.

## Rationale
The upstream / vendor LD06 drivers often:
- Provide scan_time=0.0 or inaccurate increments
- Do not expose per-beam timing needed for motion deskew
- Lack multi-lidar fusion & configurable angle filtering

This package establishes a modular architecture to support those advanced features.

## Architecture
Components (namespaces in `include/sigyn_lidar`):

- LD06Parser: Incremental byte stream parser -> validated packets -> assembled ScanFrame(s)
- Deskew (future): Uses IMU yaw + wheel odom velocity to correct beam angles for platform motion
- MultiLidarFuser (scaffold): Combine per-device frames with segment masks into a fused scan
- Node (`sigyn_lidar_node.cpp`): Serial device management, polling, publishing

## Packet Structure (LD06)
Each 47-byte packet:
```
0    Header (0x54)
1    Ver/Len  (low 5 bits length=0x2C)
2-3  Speed (little-endian)
4-5  Start Angle (hundredth deg)
6-41 12 * (2 bytes distance_mm + 1 byte intensity)
42-43 End Angle (hundredth deg)
44-45 Timestamp (ms modulo)
46   CRC8
```
Note: The LD06 protocol calls the intensity field "confidence" but it's actually the laser reflection intensity (0-255), not a measurement quality indicator.
Angles are nominal; true angle per point interpolated between start/end.
Wrap (end < start) indicates revolution boundary.

## Timing & Deskew
- `ScanFrame.scan_start_time` / `scan_end_time` measured using node wall clock when packets processed.
- Per-beam `relative_time` proportionally interpolated; upgraded logic will incorporate packet timestamps & RPM for higher fidelity.
- Deskew plan: For each beam, compute robot yaw delta & translational motion since scan start; adjust angle accordingly (pure rotation first, translation later if needed).

## Parameters (current)
```
devices: ["/dev/ttyUSB0"]
frame_id: "lidar_frame"
fused_frame_id: "lidar_fused"
fused_topic: "/scan"
```
Planned additions: baud_rate, range_min/max, min_confidence, yaw_source_topic, segment specifications per device, invert flag, publish_midpoint_timestamp.

## Topics
- Per-device: `/scan_<index>` (sensor_msgs/LaserScan)
- Fused: configurable (default `/scan`)

## TODO (abridged)
See `TODO.md` for detailed roadmap (to be created).
- Implement high fidelity per-beam timing using RPM & packet timestamp
- Deskew using IMU yaw interpolation
- Multi-lidar fusion with segment masks & overlap resolution
- Parameter expansion
- Intensity-based filtering / docking pattern detection hooks

## Development
Build:
```
colcon build --symlink-install --packages-select sigyn_lidar
```
Run (single device example):
```
ros2 run sigyn_lidar sigyn_lidar
```
Launch (future extended params):
```
ros2 launch sigyn_lidar sigyn_lidar.launch.py
```

## License
TBD (inherits workspace LICENSE unless overridden).
