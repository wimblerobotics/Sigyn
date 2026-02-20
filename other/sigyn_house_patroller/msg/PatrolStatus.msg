# PatrolStatus.msg
# Current status of the patrol system

# Header for timestamp
std_msgs/Header header

# Current patrol mode
string patrol_mode  # "idle", "patrolling", "investigating", "charging", "emergency"

# Current location and navigation status
geometry_msgs/PoseStamped current_pose
string current_room
geometry_msgs/PoseStamped target_waypoint
string target_room
bool is_navigating
float64 navigation_progress  # 0.0 to 1.0

# System health indicators
float64 battery_percentage
float64 current_temperature
bool[] sensor_health  # Array of sensor status flags
string[] sensor_names  # Corresponding sensor names

# Patrol statistics
uint32 waypoints_completed
uint32 threats_detected
uint32 alerts_sent
builtin_interfaces/Time patrol_start_time
builtin_interfaces/Duration total_patrol_time

# Current waypoint queue
string[] queued_waypoints
uint8[] waypoint_priorities

# Last known threats
ThreatAlert[] recent_threats

# Constants for patrol modes
string MODE_IDLE = "idle"
string MODE_PATROLLING = "patrolling"
string MODE_INVESTIGATING = "investigating"
string MODE_CHARGING = "charging"
string MODE_EMERGENCY = "emergency"
