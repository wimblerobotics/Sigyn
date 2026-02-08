## yolo_oakd_test

Debug/validation package for OAK-D spatial can detection on Sigyn.

This package runs a DepthAI spatial YOLO pipeline, publishes annotated RGB images, and publishes a 3D can point. It is intended for validating OAK-D detections, TF alignment, and the can position in base_link.

### What it publishes
- /oakd_top/annotated_image (sensor_msgs/Image)
- /oakd_top/rgb_preview (sensor_msgs/Image)
- /oakd_top/depth_image (sensor_msgs/Image)
- /oakd_top/can_point_camera (geometry_msgs/PointStamped)
- /oakd/object_detector_heartbeat (vision_msgs/Detection2DArray) heartbeat

### Primary launch
Use the package launch with the description TF tree:

ros2 launch yolo_oakd_test test_can_detection.launch.py

### Key parameters
The node supports parameters used for debugging and alignment:

- blob_path: Path to the DepthAI blob
- camera_frame: Frame to publish camera points in (default: oak_rgb_camera_optical_frame)
- spatial_axis_map: Mapping of DepthAI spatial coords (x,y,z) into the camera frame
- expected_target_base: Expected can position in base_link, for auto-suggesting axis map
- suggest_axis_map: Enable map suggestion log output

### Axis mapping fix (important)
DepthAI spatial coordinates are not guaranteed to match ROS optical frame conventions. In this setup, the correct mapping was:

spatial_axis_map: -z,-x,y

Without this, the base_link transform placed the can at the wrong height/direction even though the URDF and TF tree were correct. The mapping fix aligns the raw DepthAI spatial vector to the ROS optical frame before TF, yielding the correct base_link position.

You can auto-suggest the mapping by running:

ros2 launch yolo_oakd_test test_can_detection.launch.py expected_target_base:="[0.65, 0.0, 0.6]" suggest_axis_map:=true

The node will log a suggested spatial_axis_map value based on the expected can position.

### Build
From the workspace root:

colcon build --symlink-install --allow-overriding teleop_twist_keyboard

### Notes
- The RViz config is in config/can_detection.rviz and is loaded by the launch file.
- The detector logs the raw DepthAI spatial point, mapped point, and the base_link transform for traceability.
