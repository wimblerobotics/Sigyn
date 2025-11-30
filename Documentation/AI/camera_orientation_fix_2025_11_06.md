# Camera Orientation Fix - November 6, 2025

## Problem Analysis

### Symptoms
1. `/oakd_top/color/image_raw` displays but shows ceiling instead of floor
2. `/oakd_top/stereo/depth_image` is all black in rqt
3. Camera should point downward ~59° but appears to point upward

### Root Cause: Gazebo vs ROS Optical Frame Convention Mismatch

**ROS Optical Frame Convention:**
- +X = right in image
- +Y = down in image
- +Z = forward (optical axis)

**Gazebo Camera Sensor Convention:**
- +X = forward (optical axis)
- +Y = left in image
- +Z = up in image

**The Problem:**
When we attach a Gazebo camera sensor to a ROS optical frame with `<pose>0 0 0 0 0 0</pose>`, Gazebo assumes its +X is forward. But in a ROS optical frame, +Z is forward. This 90° mismatch causes the camera to look in the wrong direction.

### Transform Verification

From `ros2 run tf2_ros tf2_echo base_link oak_rgb_camera_optical_frame`:
```
Translation: [-0.090, 0.000, 1.650]
Rotation: RPY (degree) [-0.000, -58.957, -180.000]
```

The optical frame IS correctly positioned and oriented at -59° pitch. The issue is purely in how Gazebo interprets sensor poses within that frame.

## Solution

### Rotate Sensor Pose Within Optical Frame

Added a 90° pitch rotation to align Gazebo's +X-forward convention with ROS optical frame's +Z-forward:

```xml
<!-- OLD - sensor pointed wrong direction -->
<pose>0 0 0.05 0 0 0</pose>

<!-- NEW - rotate 90° around Y axis -->
<!-- This converts: Gazebo +X → ROS optical +Z (forward) -->
<pose>0.05 0 0 0 1.5708 0</pose>
```

**Rotation Breakdown:**
- **Before rotation**: Gazebo sensor +X, +Y, +Z
- **After pitch 90°**: 
  - Gazebo +X → optical +Z (correct forward direction!)
  - Gazebo +Y → optical +Y (unchanged)
  - Gazebo +Z → optical -X (correct up direction!)
- **Translation `0.05 0 0`**: Offset 5cm along rotated +X (which is now optical +Z forward) to clear mount

### Applied to Both Sensors

**RGB Camera (oakd.urdf.xacro):**
```xml
<gazebo reference="oak_rgb_camera_optical_frame">
  <sensor name="rgb_camera" type="camera">
    <pose>0.05 0 0 0 1.5708 0</pose>  <!-- X Y Z Roll Pitch Yaw -->
    ...
  </sensor>
</gazebo>
```

**RGBD Camera (oakd.urdf.xacro):**
```xml
<gazebo reference="oak_left_camera_optical_frame">
  <sensor name="stereo_camera" type="rgbd_camera">
    <pose>0.05 0 0 0 1.5708 0</pose>
    ...
  </sensor>
</gazebo>
```

## Why This Works

1. **Physical Mount** (sigyn.urdf.xacro):
   ```xml
   <origin xyz="-0.09 0.0 1.65" rpy="${-pi/2} ${(-pi/2)+1.029} 0"/>
   ```
   Places camera high on pole, tilted ~59° down

2. **Optical Frame Transform** (oakd.urdf.xacro):
   ```xml
   <origin rpy="-1.5708 0.0 -1.5708" xyz="0 0 0"/>
   ```
   Standard ROS optical frame: [-90°, 0°, -90°]

3. **Sensor Pose in Optical Frame**:
   ```xml
   <pose>0.05 0 0 0 1.5708 0</pose>
   ```
   Aligns Gazebo camera convention with ROS optical convention

**Combined Effect:**
- Physical mount → camera assembly points down at floor
- Optical frame → ROS convention satisfied
- Sensor pose → Gazebo sees correctly through optical frame

## Expected Results After Restart

### RGB Camera (`/oakd_top/color/image_raw`)
- ✅ Should show floor, front edge of robot, nearby walls
- ✅ Horizon should be level (horizontal)
- ✅ Should update as robot moves
- ✅ No ceiling/sky in view

### Depth Image (`/oakd_top/stereo/depth_image`)
- ✅ Should show grayscale depth gradient in rqt
- ✅ Closer objects (floor, robot edge) = darker
- ✅ Farther objects (walls) = lighter
- ✅ RViz DepthCloud should display 3D depth visualization
- ✅ Should change as robot moves

### Point Cloud (`/oakd_top/stereo/points`)
- ✅ Should show 3D colored points
- ✅ Points should represent floor and obstacles
- ✅ Should be oriented correctly (not tilted)

## RViz Configuration

The existing RViz config (`rviz/config/config.rviz`) is already correctly configured:

```yaml
- Class: rviz_default_plugins/DepthCloud
  Color Image Topic: /oakd_top/color/image_raw
  Depth Map Topic: /oakd_top/stereo/depth_image
  Enabled: true
  Name: OAKD DepthCloud
```

No changes needed - RViz will automatically display correctly once the camera orientation is fixed.

## Verification Steps

After restarting simulation:

```bash
# 1. Check RGB image orientation
ros2 run rqt_image_view rqt_image_view /oakd_top/color/image_raw
# Should see: floor, robot front edge, level horizon

# 2. Check depth image
ros2 run rqt_image_view rqt_image_view /oakd_top/stereo/depth_image
# Should see: grayscale gradient (not all black)

# 3. Verify camera info has correct frame
ros2 topic echo /oakd_top/color/camera_info --once | grep frame_id
# Should show: frame_id: oak_rgb_camera_optical_frame

# 4. Check TF is still correct
ros2 run tf2_ros tf2_echo base_link oak_rgb_camera_optical_frame
# Should show: Rotation RPY (degree) [-0.000, -58.957, -180.000]

# 5. View in RViz
# Open RViz, enable "OAKD DepthCloud"
# Should see colored 3D points representing floor and obstacles
```

## Understanding the Math

### Optical Frame Rotation Matrix

From TF echo `oak → oak_rgb_camera_optical_frame`:
```
Matrix:
  0.000  0.000  1.000  (oak +X becomes optical +Z)
 -1.000  0.000  0.000  (oak +Y becomes optical -X)
  0.000 -1.000  0.000  (oak +Z becomes optical -Y)
```

This means:
- Oak forward (+X) = Optical forward (+Z) ✓
- Oak left (+Y) = Optical right (-X) ✓
- Oak up (+Z) = Optical down (-Y) ✓

### Sensor Pose Rotation (Pitch 90°)

Pitch rotation around Y-axis by 90° (1.5708 radians):
```
Before: [+X, +Y, +Z] (Gazebo convention)
After:  [+Z, +Y, -X] (ROS optical convention)
```

This aligns Gazebo's +X-forward with optical +Z-forward.

## Technical Notes

### Why Not Rotate the Optical Frame Instead?

We MUST keep the optical frame at standard ROS convention [-90°, 0°, -90°] because:
1. ROS camera drivers expect this
2. `camera_info` messages reference this frame
3. TF tree consistency with real hardware
4. Image processing algorithms assume optical frame convention

### Why Gazebo Needs Special Handling?

Gazebo's camera sensors were designed with graphics/game engine conventions (+X forward), while ROS uses robotics conventions (+Z forward for optical frames). The `<pose>` in the sensor definition is our way to bridge this semantic gap.

### Pose Format

```xml
<pose>X Y Z Roll Pitch Yaw</pose>
```
- **Translation (X Y Z)**: Position offset in parent frame
- **Rotation (Roll Pitch Yaw)**: Orientation in parent frame (radians)
- Order: Yaw → Pitch → Roll (Z → Y → X extrinsic rotations)

For our case:
- `0.05 0 0`: Move 5cm forward (along parent's +X, which becomes sensor's +Z after rotation)
- `0 1.5708 0`: Rotate 90° around Y to align conventions

## References

- ROS REP 103: Standard Units of Measure and Coordinate Conventions
- ROS REP 105: Coordinate Frames for Mobile Platforms
- Gazebo Camera Sensor: http://sdformat.org/spec?elem=sensor&ver=1.9
- TF2 Tutorials: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/

## Files Modified

1. `description/urdf/oakd.urdf.xacro` - Both camera sensor poses updated

## Next Steps If Issues Persist

### If Still Looking at Ceiling
- Try negative pitch: `<pose>0.05 0 0 0 -1.5708 0</pose>`
- Or roll 180° then pitch: `<pose>0.05 0 0 3.14159 1.5708 0</pose>`

### If Depth Still All Black
- Increase far clip plane in URDF (currently 10m)
- Check near clip (currently 0.3m - might be too far)
- Verify sensor actually sees geometry: enable `<visualize>true</visualize>`

### If Point Cloud Inverted
- Try roll 180°: `<pose>0.05 0 0 3.14159 1.5708 0</pose>`
- Or adjust yaw: `<pose>0.05 0 0 0 1.5708 3.14159</pose>`
