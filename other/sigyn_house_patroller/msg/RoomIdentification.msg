# RoomIdentification.msg
# Information about the robot's current room

# Header for timestamp
std_msgs/Header header

# Room identification
string room_name
float64 confidence  # 0.0 to 1.0
geometry_msgs/Point room_center
geometry_msgs/Polygon room_bounds

# Supporting evidence
string[] detected_features  # e.g., "kitchen_counter", "living_room_couch"
float64[] feature_confidences

# Room characteristics
float64 room_area  # square meters
uint32 wall_count
string[] adjacent_rooms

# Environmental data
float64 ambient_temperature
float64 ambient_light_level
bool motion_detected
