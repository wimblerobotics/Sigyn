# Teensy to Sigyn Communication Bridge

This project implements bidirectional communication between a Teensy 4.1/4.2 microcontroller and a ROS2 robot system.

## Directory Structure

```
~/teensy_to_sigyn/              # Teensy Arduino code
├── teensy_to_sigyn.ino         # Main Arduino sketch
├── roboclaw.h                  # Roboclaw module header
├── roboclaw.cpp                # Roboclaw module implementation
├── battery.h                   # Battery module header
└── battery.cpp                 # Battery module implementation

~/sigyn_to_teensy_ws/           # ROS2 workspace
└── src/sigyn_to_teensy/        # ROS2 package
    ├── CMakeLists.txt          # Build configuration
    ├── package.xml             # Package metadata
    └── src/
        └── teensy_bridge.cpp   # ROS2 bridge node
```

## Teensy Code (Arduino)

### Features
- **Roboclaw Module**: Receives Twist messages and sends status updates
- **Battery Module**: Sends battery state at configurable intervals
- **Modular Design**: Easy to add new modules

### Message Protocol
- Incoming: `TWIST:linear_x,angular_z`
- Outgoing: `STATUS:message_content` and `BATTERY:voltage,current,charge`

## ROS2 Code

### Features
- Subscribes to `/cmd_vel` topic
- Publishes to `/roboclaw_diagnostics` and `/main_battery_state`
- Serial communication at 115200 baud
- Automatic message parsing and conversion

### Building and Running

```bash
cd ~/sigyn_to_teensy_ws
colcon build
source install/setup.bash
ros2 run sigyn_to_teensy teensy_bridge
```

### Topics
- **Subscribed**: `/cmd_vel` (geometry_msgs/Twist)
- **Published**: 
  - `/roboclaw_diagnostics` (std_msgs/String)
  - `/main_battery_state` (sensor_msgs/BatteryState)

## Usage

1. Upload the Arduino code to your Teensy
2. Connect Teensy to your robot computer via USB
3. Build and run the ROS2 bridge node
4. The system will automatically handle message routing

## Customization

- Modify timing intervals in the module constructors
- Add new message types in both Arduino and ROS2 code
- Extend the protocol by adding new modules following the existing pattern
