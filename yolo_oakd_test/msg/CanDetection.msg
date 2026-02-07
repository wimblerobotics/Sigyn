# Custom message for can detection with extensive debugging info
std_msgs/Header header

# Detection metadata
string class_name
float32 confidence

# Bounding box in original image coordinates (1080p: 1920x1080)
int32 bbox_xmin
int32 bbox_ymin
int32 bbox_xmax
int32 bbox_ymax

# Bounding box center in original image
int32 bbox_center_x
int32 bbox_center_y

# 3D spatial coordinates in camera optical frame (meters)
geometry_msgs/Point spatial_camera

# 3D spatial coordinates in base_link frame (meters)
geometry_msgs/Point spatial_base_link

# Distance from camera (meters)
float32 distance_from_camera

# Diagnostic information
string[] debug_log
