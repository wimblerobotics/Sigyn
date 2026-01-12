# Phase 2 Complete: Gazebo Simulation Environment

## Summary

Phase 2 has been completed successfully. A complete Gazebo simulation environment has been created for the Can Do Challenge, including:

1. ✅ Custom world file with table and can
2. ✅ Simulated camera sensors (OAK-D and Pi Camera)
3. ✅ Launch file for simulation environment
4. ✅ Integration with existing Sigyn robot URDF

## Files Created

### 1. World File: `description/worlds/can_challenge.world`
- Simple world with ground plane and lighting
- Challenge table at (0.0, 20.0, 0.35) - dimensions 1.2m x 0.8m x 0.7m
- CokeZero can on table at (0.0, 20.0, 0.785) - cylinder with radius 0.033m, length 0.122m
- Can has black body with red label band for realistic appearance
- Can has proper physics (mass 0.015kg, inertia)

### 2. Camera Macros: `description/urdf/sim_cameras.urdf.xacro`
Contains two xacro macros for simulation cameras:

**OAK-D Simulation Camera:**
- Attached to `base_link` at elevator position
- Resolution: 1280x720 @ 30fps
- FOV: 1.2 radians (~69°)
- Range: 0.05m to 10.0m
- Topic: `/oakd/image_raw`
- Frame: `oakd_sim_optical`

**Pi Camera Simulation:**
- Attached to `parallel_gripper_base_plate` (gripper)
- Resolution: 640x480 @ 15fps
- FOV: 1.085 radians (~62°)
- Range: 0.02m to 2.0m (close-up work)
- Topic: `/pi_camera/image_raw`
- Frame: `pi_camera_sim_optical`

### 3. Launch File: `can_do_challenge/launch/can_do_sim_launch.py`
- Launches full Sigyn robot in `can_challenge.world`
- Sets `use_sim_time:=true` for all nodes
- Includes can_do_challenge behavior tree node
- Optional RViz visualization (default: true)
- Optional Groot monitoring (default: true, port 1667)

**Usage:**
```bash
ros2 launch can_do_challenge can_do_sim_launch.py

# Disable RViz:
ros2 launch can_do_challenge can_do_sim_launch.py use_rviz:=false

# Disable Groot:
ros2 launch can_do_challenge can_do_sim_launch.py use_groot:=false
```

### 4. URDF Updates: `description/urdf/sigyn.urdf.xacro`
- Added conditional inclusion of `sim_cameras.urdf.xacro` when `sim_mode:=true`
- Cameras only active in simulation (not on real robot)
- OAK-D camera attached to base_link (elevator position)
- Pi camera attached to gripper for close-up can centering

## Coordinate System

The can location in the behavior tree (0.0, 20.0, 0.75) aligns with:
- Table center: (0.0, 20.0, 0.35) with height 0.7m
- Table top at: z = 0.35 + 0.35 = 0.7m
- Can center at: z = 0.7 + 0.061 + 0.024 = 0.785m (table + half can height + small offset)

The behavior tree expects:
- `expectedCanLocation`: (0.0, 20.0, 0.75) - slightly lower for gripper approach
- Robot approaches from y = 19.5 (0.5m in front of table)
- Elevator height computed as: can_z + 0.1 = 0.85m

## Camera Topics

When simulation is running, these topics will be available:

```bash
# OAK-D camera (for long-range detection)
/oakd/image_raw          # sensor_msgs/Image
/oakd/camera_info        # sensor_msgs/CameraInfo (if configured)

# Pi Camera (for close-up centering)
/pi_camera/image_raw     # sensor_msgs/Image
/pi_camera/camera_info   # sensor_msgs/CameraInfo (if configured)
```

## Testing the Simulation

To verify the simulation environment:

```bash
# Launch simulation
ros2 launch can_do_challenge can_do_sim_launch.py

# In Gazebo GUI:
# - Verify table spawns at (0, 20, 0.35)
# - Verify can spawns on table at (0, 20, 0.785)
# - Verify Sigyn robot spawns
# - Verify cameras are visible in sensor visualization

# Check camera topics
ros2 topic list | grep image

# View camera feeds
ros2 run rqt_image_view rqt_image_view /oakd/image_raw
ros2 run rqt_image_view rqt_image_view /pi_camera/image_raw

# Monitor behavior tree in Groot2
# Connect to localhost:1667
```

## Integration Points

The simulation environment is now ready for Phase 3 vision implementation:

1. **CanDetectedByOAKD**: Subscribe to `/oakd/image_raw`, implement HSV + Canny detection
2. **CanDetectedByPiCamera**: Subscribe to `/pi_camera/image_raw`, detect can in close-up
3. **CanCenteredInPiCamera**: Analyze pi camera feed to verify can is centered
4. **CanWithinReach**: Use OAK-D depth or distance estimation

## Next Steps (Phase 3)

- [ ] Implement vision algorithms (HSV color filtering + Canny edge detection)
- [ ] Integrate Nav2 action clients for real path planning
- [ ] Connect gripper/elevator control to simulation joints
- [ ] Add camera calibration files if needed
- [ ] Implement object pose estimation from camera
- [ ] Add AprilTags or fiducials for localization testing

## Known Limitations

1. **Camera Plugins**: Using basic Gazebo camera plugin, not full OAK-D simulation with depth
2. **Can Physics**: Can is lightweight but may need tuning for realistic grasping
3. **Gripper Physics**: May need contact sensor plugins for grasp detection
4. **No Depth**: Current cameras are RGB only, depth estimation would need stereo or RGBD plugin

## Build Status

✅ All files created and building successfully
✅ Launch file structure validated
✅ URDF modifications compile without errors
✅ Ready for simulation testing

---

**Phase 2 Status**: ✅ COMPLETE

The simulation environment is ready for testing. Launch the simulation to verify all components spawn correctly before proceeding to Phase 3.
