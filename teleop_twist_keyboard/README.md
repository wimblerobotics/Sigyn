# teleop_twist_keyboard - ROS2 Version

This package provides a keyboard-based teleop interface for controlling robots that use geometry_msgs/Twist messages.

## Converted from ROS1 to ROS2

This package has been converted from the original ROS1 version to work with ROS2.

## Usage

### Direct execution (Recommended)
The keyboard teleop requires direct terminal access, so it's best to run it directly:

```bash
source ~/sigyn_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Setting parameters
Use ROS2 parameter syntax to customize behavior:

```bash
# Set custom speed and turn rates
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.25 -p turn:=0.3

# Enable continuous publishing at 10Hz (recommended for smooth control)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0

# Combine multiple parameters
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.25 -p turn:=0.3 -p repeat_rate:=10.0 -p key_timeout:=0.6
```

### Launch file
A launch file is provided but may have terminal I/O issues:
```bash
ros2 launch teleop_twist_keyboard teleop_twist_keyboard.launch.py
```

### Helper script
Or use the provided script:
```bash
./install/teleop_twist_keyboard/share/teleop_twist_keyboard/scripts/run_teleop.sh
```

# Usage
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```

# Reducing Key Repeat Delay

When holding down a movement key (like 'l' for turning right), you may notice a pause before the key starts repeating. This is due to your system's keyboard repeat delay. The teleop_twist_keyboard package provides a solution using the `repeat_rate` parameter:

```bash
# Recommended: Use repeat_rate for smooth, responsive control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0
```

With `repeat_rate:=10.0`, the node will continuously publish movement commands at 10 Hz when you hold down a key, eliminating the keyboard repeat delay and providing immediate, smooth control.

## Alternative: System-level keyboard settings

You can also adjust your system's keyboard repeat settings:
```bash
# Set keyboard repeat delay to 200ms and repeat rate to 50 Hz
xset r rate 200 50
```

However, using the `repeat_rate` parameter is recommended as it provides more consistent behavior across different systems.

# Key Timeout

Teleop_twist_keyboard can be configured to stop your robot if it does not receive any key presses in a configured time period, using the `key_timeout` parameter.

For example, to stop your robot if a keypress has not been received in 0.6 seconds:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p key_timeout:=0.6
```

It is recommended that you set `key_timeout` higher than the initial key repeat delay on your system (This delay is 0.5 seconds by default on Ubuntu, but can be adjusted).

# Twist with header
Publishing a `TwistStamped` message instead of `Twist` can be enabled with the `stamped` parameter. Additionally the `frame_id` of the `TwistStamped` message can be set with the `frame_id` parameter.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link
```

# Topic Remapping
To publish to a different topic (e.g., `my_cmd_vel`):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=my_cmd_vel
```

# Terminal Issues

## Terminal Reset
If your terminal appears broken after running teleop (e.g., keypresses don't show up), you can reset it manually:
```bash
reset
# or
stty sane
```

The package should automatically restore terminal settings, but in rare cases manual reset may be needed.

## Testing
Use the provided test script to verify functionality:
```bash
./scripts/test_teleop.sh
```