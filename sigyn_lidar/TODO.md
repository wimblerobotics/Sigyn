# sigyn_lidar TODO

Priority 1 (Parser & Core Accuracy)
- [ ] Validate RPM derivation from `speed` field; compute precise scan_time
- [ ] Use packet `timestamp` delta (modulo handling) to refine per-beam relative_time
- [ ] Improve angle interpolation: vendor sometimes non-linear; verify against empirical data
- [ ] Robust wrap detection using cumulative angle & timestamp discontinuities
- [ ] Error counters (crc_fail, length_mismatch, desync_resync) exposed via diagnostics

Priority 2 (Deskew)
- [ ] Subscribe to configurable IMU yaw topic + odom twist
- [ ] Maintain yaw unwrapping & time-indexed buffer (already have interpolator skeleton)
- [ ] Compute yaw delta per beam (scan_start + relative_time)
- [ ] Option: midpoint vs start reference timestamp (parameter)
- [ ] (Future) Translational deskew using linear velocity & beam geometry

Priority 3 (Multi-Lidar Fusion)
- [ ] Load per-device angle segments (e.g., [[0,90],[180,270]])
- [ ] For overlapping segments choose beam with lower range / higher confidence
- [ ] Resample to uniform global angle grid if increments differ
- [ ] Assign fused_frame_id per configuration
- [ ] (Future) Support heterogeneous models (normalize specs)

Priority 4 (Config & Parameters)
- [ ] Parameters: baud_rate, range_min, range_max, min_confidence
- [ ] Parameter: yaw_source_topic, odom_topic, publish_midpoint_timestamp
- [ ] Per-device inversion / angle_offset correction
- [ ] Dynamic reconfigure (ROS 2 param events) for min_confidence & segments

Priority 5 (Quality & Diagnostics)
- [ ] Publish diagnostics msgs with error counters & RPM
- [ ] Self-test: detect no-rotation / stalled sensor
- [ ] Add unit tests for parser (feed known packet captures)
- [ ] Benchmark CPU usage & latency profiling

Priority 6 (Advanced Features)
- [ ] Intensity pattern detection hooks for docking target
- [ ] Output point cloud (sensor_msgs/PointCloud2) option
- [ ] Adaptive filtering (drop low confidence beyond distance threshold)
- [ ] Exposure for raw packets (rosbag analysis) via topic `/lidar_packets`

Priority 7 (Documentation)
- [ ] Expand README with fusion diagrams & deskew math derivation
- [ ] Add packet field reference table with units
- [ ] Provide example multi-lidar YAML config

Nice to Have
- [ ] Support for other LD series variants via traits
- [ ] Polynomial correction for systematic angle errors
- [ ] GPU accelerated fusion / deskew (future)
