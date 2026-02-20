# SystemHealth.msg
# Overall system health status

# Header for timestamp
std_msgs/Header header

# Overall system status
uint8 overall_health  # 0=UNKNOWN, 1=HEALTHY, 2=DEGRADED, 3=UNHEALTHY, 4=FAILED
string system_status_description

# Individual component health
string[] component_names
uint8[] component_health_status
string[] component_descriptions
builtin_interfaces/Time[] last_update_times

# Resource usage
float64 cpu_usage_percent
float64 memory_usage_percent
float64 disk_usage_percent
float64 network_usage_mbps

# Environmental factors
float64 internal_temperature
float64 ambient_temperature
float64 humidity_percent
float64 battery_voltage

# Performance metrics
float64 localization_accuracy
float64 navigation_success_rate
uint32 recent_failures_count
string[] recent_error_messages

# Predictive indicators
bool maintenance_required
builtin_interfaces/Time next_maintenance_due
string[] maintenance_recommendations

# Constants for health status
uint8 HEALTH_UNKNOWN = 0
uint8 HEALTH_HEALTHY = 1
uint8 HEALTH_DEGRADED = 2
uint8 HEALTH_UNHEALTHY = 3
uint8 HEALTH_FAILED = 4
