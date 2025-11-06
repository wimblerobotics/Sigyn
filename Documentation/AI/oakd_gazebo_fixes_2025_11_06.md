# OAK-D Gazebo Simulation Fixes - November 6, 2025

## Problems Identified

### 1. RGB Camera Topic Mismatch
- **Issue**: Gazebo published to `/oakd_top/color` but RViz expected `/oakd_top/color/image_raw`
- **Symptom**: `/oakd_top/color/image_raw` had 0 publishers, only subscribers (RViz)
- **Root Cause**: Non-standard camera topic naming

### 2. Multiple Type Conflict on Point Cloud Topic
- **Issue**: `/oakd_top/stereo/points` had both `LaserScan` and `PointCloud2` types
- **Symptom**: `ros2 topic echo` refused to echo, RViz couldn't display
- **Root Cause**: Nav2 costmap configured with `data_type: "LaserScan"` subscribed as LaserScan while Gazebo rgbd_camera published PointCloud2

### 3. Duplicate Depth Sensors
- **Issue**: Both `depth_camera` and `rgbd_camera` sensors on same link
- **Symptom**: Redundant depth topics, potential conflicts
- **Root Cause**: Over-engineered sensor setup instead of using single rgbd_camera

### 4. Depth Image Orientation Wrong
- **Symptom**: Depth displayed tilted 90° upward toward ceiling
- **Root Cause**: Likely incorrect optical frame orientation (to be verified after restart)

## Solutions Applied

### 1. Standardized Camera Topics (oakd.urdf.xacro)

**Changed RGB camera topic to standard convention:**
```xml
<!-- OLD: <topic>oakd_top/color</topic> -->
<topic>oakd_top/color/image_raw</topic>
```

Gazebo camera sensors follow convention:
- RGB Camera: `base/image_raw` and `base/camera_info`
- RGBD Camera: `base/image`, `base/depth_image`, `base/points`, `base/camera_info`

### 2. Consolidated Depth + Point Cloud (oakd.urdf.xacro)

**Removed separate `depth_camera` sensor, kept only `rgbd_camera`:**
```xml
<!-- Single rgbd_camera replaces both depth_camera and separate rgbd -->
<sensor name="stereo_camera" type="rgbd_camera">
  <topic>oakd_top/stereo</topic>
  <!-- Publishes: /oakd_top/stereo/image -->
  <!--           /oakd_top/stereo/depth_image -->
  <!--           /oakd_top/stereo/points -->
  <!--           /oakd_top/stereo/camera_info -->
</sensor>
```

Benefits:
- No duplicate sensors
- All stereo data synchronized from single source
- Standard Gazebo rgbd_camera outputs

### 3. Updated Bridge Mappings (gz_bridge.yaml)

**Aligned bridge with actual Gazebo outputs:**
```yaml
# RGB Camera (standard naming)
- ros_topic_name: "oakd_top/color/image_raw"
  gz_topic_name: "oakd_top/color/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# RGBD outputs (depth_image not depth)
- ros_topic_name: "oakd_top/stereo/depth_image"
  gz_topic_name: "oakd_top/stereo/depth_image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# Point Cloud (PointCloud2 only)
- ros_topic_name: "oakd_top/stereo/points"
  gz_topic_name: "oakd_top/stereo/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
```

### 4. Fixed Nav2 Costmap Config (navigation_sim.yaml)

**Changed from LaserScan to PointCloud2:**
```yaml
oakd_top:
  data_type: "PointCloud2"  # Was: "LaserScan"
  inf_is_valid: False        # Was: True (lasers have infinite, point clouds don't)
  topic: /oakd_top/stereo/points
```

This eliminates the type conflict - now only PointCloud2 publishers and subscribers.

## Expected Results After Restart

1. **RGB Image (`/oakd_top/color/image_raw`)**:
   - ✅ Should publish at ~30 Hz
   - ✅ RViz should display camera feed
   - ✅ Images should update as robot moves
   - ✅ Camera info available at `/oakd_top/color/camera_info`

2. **Depth Image (`/oakd_top/stereo/depth_image`)**:
   - ✅ Should publish at ~30 Hz
   - ✅ RViz should display depth visualization
   - ✅ Orientation should be correct (not tilted upward)
   - ✅ Depth info at `/oakd_top/stereo/camera_info`

3. **Point Cloud (`/oakd_top/stereo/points`)**:
   - ✅ Single type: `sensor_msgs/msg/PointCloud2`
   - ✅ Should publish at ~30 Hz
   - ✅ Nav2 costmap should consume without errors
   - ✅ RViz should display 3D points

## Reference: Gazebo Camera Sensor Standards

### Standard `camera` Sensor
Publishes:
- `{topic}/image_raw` - RGB image
- `{topic}/camera_info` - Camera calibration

### Standard `rgbd_camera` Sensor
Publishes:
- `{topic}/image` - RGB image from color camera
- `{topic}/depth_image` - Depth image (float distances)
- `{topic}/points` - Point cloud (XYZ + RGB)
- `{topic}/camera_info` - Camera calibration

### Key Conventions
1. Always use `/image_raw` suffix for camera image topics
2. Use `/depth_image` not `/depth` for depth data
3. Frame IDs should be optical frames for correct ROS conventions
4. RGBD sensors should use single `rgbd_camera` type, not separate depth + RGB

## Verification Commands

After relaunch:
```bash
# Check all OAK-D topics exist
ros2 topic list | grep oakd_top

# Verify RGB publishes
ros2 topic hz /oakd_top/color/image_raw

# Verify depth publishes
ros2 topic hz /oakd_top/stereo/depth_image

# Verify points type is clean
ros2 topic info /oakd_top/stereo/points
# Should show ONLY: Type: sensor_msgs/msg/PointCloud2

# Check frame IDs
ros2 topic echo /oakd_top/color/camera_info --once | grep frame_id
# Should show: frame_id: oak_rgb_camera_optical_frame

ros2 topic echo /oakd_top/stereo/camera_info --once | grep frame_id
# Should show: frame_id: oak_left_camera_optical_frame
```

## Related Files Modified

1. `description/urdf/oakd.urdf.xacro` - Sensor definitions
2. `base/config/gz_bridge.yaml` - ROS↔Gazebo topic bridges
3. `base/config/navigation_sim.yaml` - Nav2 costmap sensor config

## Next Steps if Issues Persist

### If Depth Still Tilted Upward
- Check optical frame transform in TF tree: `ros2 run tf2_tools view_frames`
- Verify `oak_left_camera_optical_joint` has correct RPY: `[-π/2, 0, -π/2]`
- Ensure sensor pose in Gazebo is `[0 0 0.05 0 0 0]` (small Z offset only)

### If Images Still Static/Black
- Increase sensor pose Z offset: try `0.10` or `0.15` to clear mount geometry
- Check Gazebo GUI to see if camera frustum is visible and pointing correctly
- Verify `<visualize>true</visualize>` on RGB camera to see what it sees

### If Point Cloud Still Has Multiple Types
- Restart entire simulation (not just bridge) to clear stale subscriptions
- Check for old nodes still running: `ros2 node list`
- Verify no custom code subscribing as LaserScan: `ros2 topic info /oakd_top/stereo/points -v`
