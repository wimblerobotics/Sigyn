# ThreatAlert.msg
# Represents a detected threat or anomaly

# Header for timestamp and frame
std_msgs/Header header

# Threat identification
string threat_id
string threat_type  # "temperature", "door_open", "intruder", "battery_low", etc.
uint8 severity_level  # 1=INFO, 2=WARNING, 3=CRITICAL, 4=EMERGENCY

# Location information
geometry_msgs/Point location
string room_name
float64 confidence  # 0.0 to 1.0

# Sensor data that triggered the alert
string sensor_data_json

# Description for human consumption
string description

# Recommended actions
string[] recommended_actions

# Constants for severity levels
uint8 SEVERITY_INFO = 1
uint8 SEVERITY_WARNING = 2
uint8 SEVERITY_CRITICAL = 3
uint8 SEVERITY_EMERGENCY = 4
