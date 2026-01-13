# Manual Stepping Mode for Can Do Challenge

This document describes how to use the manual stepping mode to debug and develop the behavior tree for the Can Do Challenge.

## Overview

The manual stepping mode allows you to:
- Execute the behavior tree one tick at a time
- Control simulated sensor values (battery, IMU tilt, E-stop)
- Observe behavior tree state changes in Groot
- Test reactive behaviors by changing sensor conditions between ticks

## Prerequisites

1. Groot (not Groot2) installed for visualization
2. Running simulation environment

## Quick Start

### Terminal 1: Start Simulation
```bash
cd ~/sigyn_ws
source install/setup.bash
ros2 launch can_do_challenge can_do_sim_launch.py
```

This launches:
- Gazebo with can_challenge.world
- Sigyn robot with simulated cameras
- Nav2 navigation stack
- RViz for visualization

### Terminal 2: Start Manual Controller
```bash
cd ~/sigyn_ws
source install/setup.bash
ros2 launch can_do_challenge manual_sim_launch.py
```

This launches:
- Manual Controller GUI (Tkinter window)
- can_do_challenge_node in manual stepping mode
- Groot monitoring on port 1667

### Terminal 3: Start Groot
```bash
groot
```

In Groot:
1. Click "Monitor" tab
2. Connect to localhost:1667
3. You'll see the behavior tree structure and current state

## Using the Manual Controller GUI

### Main Window Components

1. **TICK Button**
   - Click to advance the behavior tree by one tick
   - Watch Groot update to see which nodes are executed
   - Status is logged to console

2. **Battery Voltage Slider (20V - 42V)**
   - Adjust battery voltage
   - Default: 36V
   - Threshold values:
     - Critical: 30V (robot will shut down)
     - Charging: 35V (robot should charge)
   - Try setting below 30V to see emergency shutdown behavior

3. **IMU Tilt Sliders**
   - **Roll** (-45° to +45°): Tilt side-to-side
   - **Pitch** (-45° to +45°): Tilt forward-backward
   - Default: 0° (level)
   - Threshold values:
     - Warning: 20° (robot will issue warning)
     - Critical: 30° (robot will trigger safety stop)
   - Try setting >30° to see tilt safety behavior

4. **E-Stop Toggle**
   - Check to trigger emergency stop
   - Unchecked: E-stop clear
   - When triggered, behavior tree should halt and report fault

5. **Gripper Status Display (Read-Only)**
   - Shows current elevator position (0.0 - 4.0m)
   - Shows current extender position (0.0 - 0.5m)
   - Updates automatically as BT sends gripper commands

## Testing Scenarios

### Scenario 1: Normal Operation
1. Start with default values (36V battery, 0° tilt, E-stop clear)
2. Click TICK repeatedly
3. Observe normal execution in Groot

### Scenario 2: Low Battery Warning
1. Set battery to 32V (above critical, below charging)
2. Click TICK
3. Should see battery charging behavior activate

### Scenario 3: Critical Battery
1. Set battery to 28V (below critical threshold)
2. Click TICK
3. Should see system shutdown sequence

### Scenario 4: Tilt Warning
1. Set pitch to 25° (above warning threshold)
2. Click TICK
3. Should see tilt warning in logs and safety checks

### Scenario 5: Critical Tilt
1. Set roll to 35° (above critical threshold)
2. Click TICK
3. Should see emergency stop and shutdown

### Scenario 6: E-Stop Triggered
1. Check E-Stop toggle
2. Click TICK
3. Should see all operations halt immediately

### Scenario 7: Recovery Testing
1. Trigger a fault (e.g., low battery)
2. Fix the condition (raise battery voltage)
3. Click TICK to see if system recovers

## Automatic Mode (No Manual Stepping)

To run without manual stepping:

```bash
ros2 launch can_do_challenge manual_sim_launch.py step_manually:=false
```

The GUI will still run for sensor control, but the BT will execute automatically at ~10Hz.

## Topics Published by GUI

- `/sigyn/teensy_bridge/battery/status` - Battery state
- `/sigyn/teensy_bridge/imu/sensor_0` - IMU orientation
- `/sigyn/teensy_bridge/safety/estop_status` - E-stop state
- `/can_do_challenge/bt_tick` - Manual tick commands

## Topics Monitored by GUI

- `/cmd_vel_gripper` - Gripper control commands (for position simulation)

## Simulated Hardware Behavior

### Battery
- Publishes at 10Hz
- Includes voltage, percentage, and power supply status
- Used by `BatteryAboveChargingVoltage` and `BatteryAboveCriticalVoltage` conditions

### IMU
- Publishes at 10Hz
- Quaternion orientation from roll/pitch GUI settings
- Used by `RobotTiltedWarning` and `RobotTiltedCritically` conditions

### E-Stop
- Publishes at 10Hz
- Binary active/inactive state
- Used by `RobotIsEstopped` condition

### Gripper/Elevator
- Subscribes to `/cmd_vel_gripper` commands
- Integrates velocities to simulate positions
- Publishes to joint_states (TODO: not yet implemented)

## Behavior Tree Structure

The main behavior tree (`main.xml`) includes:
1. **Safety Monitoring** - Reactive checks for battery, tilt, E-stop
2. **Navigation** - Path planning and following to can location
3. **Visual Acquisition** - OAK-D and Pi Camera detection
4. **Gripper Control** - Elevator, extender, and gripper manipulation
5. **Return Home** - Navigate back to starting position

## Development Workflow

1. Make changes to `main.xml` or `bt_nodes.cpp`
2. Rebuild: `colcon build --symlink-install --packages-select can_do_challenge`
3. Source: `source install/setup.bash`
4. Launch manual controller
5. Step through BT to test changes
6. Observe in Groot and console logs
7. Iterate

## Troubleshooting

### Groot Can't Connect
- Ensure port 1667 is not blocked
- Check that `enable_groot_monitoring:=true` (default)
- Verify can_do_challenge_node is running: `ros2 node list`

### GUI Doesn't Start
- Check Python dependencies: tkinter should be installed
- Verify: `python3 -c "import tkinter"`
- Install if needed: `sudo apt-get install python3-tk`

### Sensor Values Not Updating in BT
- Check topic is publishing: `ros2 topic echo /sigyn/teensy_bridge/battery/status`
- Verify timestamps are recent
- Ensure `use_sim_time:=true` in launch files

### BT Not Responding to Ticks
- Verify manual mode: Check launch output for "Manual stepping mode enabled"
- Check tick topic: `ros2 topic echo /can_do_challenge/bt_tick`
- Try clicking TICK button multiple times

## Advanced Usage

### Custom Sensor Scenarios

You can create Python scripts to automate sensor changes:

```python
import rclpy
from sensor_msgs.msg import BatteryState

# Simulate battery discharge
battery_pub = node.create_publisher(BatteryState, '/sigyn/teensy_bridge/battery/status', 10)
for voltage in range(40, 28, -1):
    msg = BatteryState()
    msg.voltage = float(voltage)
    battery_pub.publish(msg)
    time.sleep(0.5)
```

### Logging BT Execution

Enable detailed logging:

```bash
ros2 launch can_do_challenge manual_sim_launch.py --log-level can_do_challenge:=DEBUG
```

## Next Steps

1. Implement real camera detection nodes
2. Add gripper force sensing simulation
3. Implement Nav2 action clients
4. Test full autonomous sequence
5. Deploy to real robot (set `use_sim_time:=false`)

## Related Files

- `can_do_challenge/launch/manual_sim_launch.py` - Launch file
- `can_do_challenge/src/manual_controller_gui.py` - GUI implementation
- `can_do_challenge/src/can_do_challenge_node.cpp` - BT node with manual stepping
- `can_do_challenge/src/bt_nodes.cpp` - BT node implementations
- `can_do_challenge/bt_xml/main.xml` - Behavior tree definition
