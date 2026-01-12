# Can Do Challenge - Phase 1 Complete ✓

## Summary

Successfully created the `can_do_challenge` ROS 2 package for autonomous CokeZero can fetching.

### Package Created: `/home/ros/sigyn_ws/src/Sigyn/can_do_challenge`

## What Was Delivered

### ✅ Complete Package Structure
- **CMakeLists.txt**: Build configuration with BehaviorTree.CPP v3, Nav2 dependencies
- **package.xml**: Properly declared dependencies
- **Directory structure**: include/, src/, bt_xml/, config/, launch/

### ✅ Behavior Tree (`bt_xml/main.xml`)
- **258 lines** of complete mission logic
- **9 main subtrees** + **12 supporting subtrees**
- **40+ custom node declarations** in TreeNodesModel
- Reactive safety monitoring (battery, E-stop, tilt)
- Complete navigation, vision, and manipulation sequences
- Based on FMAB1.xml and a2.xml patterns

**Mission Flow:**
```
1. Save starting pose
2. Load can location from JSON
3. Navigate to can area (Nav2)
4. Search for can with OAK-D (rotate 360°)
5. Approach can (visual servoing)
6. Raise elevator to can height
7. Center can using Pi Camera
8. Lower, extend, grasp, retract
9. Return to start (Nav2)
```

### ✅ Custom BT Nodes (`bt_nodes.hpp` + `bt_nodes.cpp`)
- **40+ node implementations** with placeholder logic
- **Safety conditions**: Battery, E-stop, tilt monitoring
- **Vision conditions**: OAK-D and Pi Camera detection
- **Navigation actions**: Path planning, following, visual servoing
- **Gripper/elevator actions**: Full manipulation sequence
- **Custom ReactiveRepeat decorator**: For interruptible loops

All nodes:
- Properly inherit from BT base classes
- Declare input/output ports correctly
- Log their execution
- Have placeholder implementations ready for real integration

### ✅ Configuration
- **can_locations.json**: JSON database for known can locations
  - Currently has one entry: CokeZeroCan at (0.0, 20.0, 0.75)
  - Easily extendable

### ✅ Launch System
- **can_do_challenge_launch.py**: Launches behavior tree node
- **Arguments**: bt_xml_file, enable_groot, groot_port
- Ready for integration with Gazebo simulation

### ✅ Documentation
- **README.md**: Complete usage guide, architecture notes, next steps
- **Build instructions**, launch commands, configuration guide
- **TODO lists** for Phase 2 (vision) and Phase 3 (simulation)

## Build Status

```bash
✓ Package compiles successfully
✓ All dependencies resolved
✓ Ready to run (placeholder implementations)
```

## How to Build & Run

```bash
# Build
cd /home/ros/sigyn_ws
colcon build --symlink-install --packages-select can_do_challenge
source install/setup.bash

# Run
ros2 launch can_do_challenge can_do_challenge_launch.py

# Monitor with Groot (if installed)
groot2 # Connect to localhost:1667
```

## Next Steps (Answering Your Questions)

### For Simulation World (Phase 2):

**Q: How to add table at (0.0, 20.0) to simulation world?**

Clone `description/worlds/home.world` → `can_challenge_world.world`, then add:

```xml
<!-- Table model -->
<model name="kitchen_table">
  <static>true</static>
  <pose>0.0 20.0 0.4 0 0 0</pose> <!-- 0.4m height (table surface at 0.75m including thickness) -->
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1.2 0.8 0.7</size> <!-- Standard table dimensions -->
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1.2 0.8 0.7</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.3 0.2 1</ambient> <!-- Wood color -->
      </material>
    </visual>
  </link>
</model>

<!-- CokeZero can -->
<model name="coke_zero_can">
  <static>true</static>
  <pose>0.0 20.0 0.785 0 0 0</pose> <!-- On table surface (0.75 + 0.035 for can bottom) -->
  <link name="link">
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.033</radius> <!-- 66mm diameter -->
          <length>0.122</length>  <!-- Standard can height -->
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.033</radius>
          <length>0.122</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.8 0.0 0.0 1</ambient> <!-- Red color for detection -->
        <diffuse>0.8 0.0 0.0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**To change table location later**: Just modify the `<pose>X Y Z roll pitch yaw</pose>` values.

### For Camera Configuration (Phase 2):

**Q: How to configure OAK-D and Pi Camera in Gazebo?**

Find your URDF files (check `description/urdf/`), then add Gazebo camera plugins for each camera link.

**For OAK-D** (assuming link name is `oakd_rgb_camera_optical_frame` or similar):
```xml
<gazebo reference="oakd_rgb_camera_optical_frame">
  <sensor name="oakd_camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.20428</horizontal_fov> <!-- ~69 degrees -->
      <image>
        <width>1280</width>
        <height>720</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>oakd</namespace>
        <remapping>image_raw:=rgb/image_raw</remapping>
        <remapping>camera_info:=rgb/camera_info</remapping>
      </ros>
      <camera_name>oakd</camera_name>
      <frame_name>oakd_rgb_camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**For Pi Camera** (assuming link name `pi_camera_optical_frame`):
```xml
<gazebo reference="pi_camera_optical_frame">
  <sensor name="pi_camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.085</horizontal_fov> <!-- ~62 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>5</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>pi_camera</namespace>
        <remapping>image_raw:=image_raw</remapping>
        <remapping>camera_info:=camera_info</remapping>
      </ros>
      <camera_name>pi_camera</camera_name>
      <frame_name>pi_camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

This will publish to:
- `/oakd/rgb/image_raw` (sensor_msgs/Image)
- `/pi_camera/image_raw` (sensor_msgs/Image)

### For Object Recognition (Phase 2/3):

**Q: How to implement simple HSV + Canny detection?**

Create vision nodes (can be separate package or add to `can_do_challenge`):

```cpp
// Example for CanDetectedByOAKD implementation
BT::NodeStatus CanDetectedByOAKD::tick()
{
  // Subscribe to /oakd/rgb/image_raw
  // Convert to OpenCV with cv_bridge
  // HSV color segmentation for red
  // Apply Canny edge detection
  // Find contours, filter by area and circularity
  // If found, write bounding box to blackboard
  // Return SUCCESS if detected, FAILURE otherwise
}
```

HSV ranges for CokeZero red:
```cpp
cv::Scalar lower_red1(0, 100, 100);
cv::Scalar upper_red1(10, 255, 255);
cv::Scalar lower_red2(160, 100, 100);
cv::Scalar upper_red2(179, 255, 255);
```

## Files Created

```
can_do_challenge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── bt_xml/
│   └── main.xml                    # Behavior tree (258 lines)
├── config/
│   └── can_locations.json          # Can location database
├── include/can_do_challenge/
│   └── bt_nodes.hpp                # Node declarations (500+ lines)
├── launch/
│   └── can_do_challenge_launch.py  # Launch file
└── src/
    ├── bt_nodes.cpp                # Node implementations (600+ lines)
    └── can_do_challenge_node.cpp   # Main node (180 lines)
```

**Total**: ~1700+ lines of code

## Phase 1 Complete! 🎉

✅ Package structure created
✅ Behavior tree designed and implemented
✅ 40+ custom BT nodes with placeholders
✅ Configuration system (JSON)
✅ Launch system
✅ Documentation
✅ **Builds successfully**

**Ready for Phase 2**: Simulation world + camera configuration
**Ready for Phase 3**: Real implementations (vision, gripper, Nav2)
