# Command to move gripper assembly to absolute positions
# Consumed by teensy_bridge, converted to STEPPOS command for Board 3

# Target positions (meters from home position)
# Use -1.0 to indicate "no change" for that axis
float32 elevator_position  # Target elevator position (m), valid range: 0.0 to 0.8999m, -1 = no change
float32 extender_position  # Target extender position (m), valid range: 0.0 to 0.3418m, -1 = no change
