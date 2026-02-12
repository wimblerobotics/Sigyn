# Current status of the gripper assembly (elevator and extender)
# Published by teensy_bridge based on STEPPERSTAT3 messages from Board 3

std_msgs/Header header

# Current positions (meters from home position)
float32 elevator_position     # Current elevator position (m), 0.0 = home/lower limit
float32 extender_position     # Current extender position (m), 0.0 = retracted

# Limit switch states
string elevator_limit_state   # "none", "upper", "lower", "both"
string extender_limit_state   # "none", "upper", "lower", "both"

# Hardware travel limits (measured from physical hardware)
float32 elevator_max_travel   # Maximum elevator travel (0.8999m)
float32 extender_max_travel   # Maximum extender travel (0.3418m)

# Status flags
bool is_homed                 # True if homing sequence completed successfully
bool is_moving                # True if motors currently in motion (based on recent commands)
