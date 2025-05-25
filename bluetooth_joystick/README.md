# Bluetooth Joystick ROS 2 Node

This package connects to a Bluetooth joystick (tested with Nimbus SteelSeries) and publishes ROS 2 messages for robot control.

## Features
- Publishes a custom `BluetoothJoystick` message with all control surface states.
- Publishes `cmd_vel` (base movement) from the left joystick only when moved.
- Publishes `cmd_vel_gripper` (or configurable topic) from the right joystick only when moved.
- Handles all buttons, left stick, right stick, and D-Pad (cross control).

## D-Pad (Cross Control)
The D-Pad (upper left cross) is a digital 4-way control, but is reported as axes (not buttons) by the Linux joystick driver. It is not analog.

## Configuration
Edit `config/bluetooth_joystick.yaml`:
```yaml
bluetooth_joystick:
  ros__parameters:
    device_name: "/dev/input/nimbus_steelseries"
    cmdvel_topic: "cmd_vel_joystick"
    gripper_topic: "cmd_vel_gripper"
    deadzone_percent: 5.0
    message_rate: 4
    scale_x: 0.25
    scale_z: 0.0625
```

## Launch
```
ros2 launch bluetooth_joystick bluetooth_joystick.launch.py
```

## Notes
- The rightmost joystick is mapped to axes 2 (left/right) and 3 (up/down).
- The D-Pad is mapped as axes 4 and 5 (digital, values -32767, 0, 32767).
- The node publishes the custom message on any control change, and only publishes `cmd_vel` or `gripper_topic` if the respective stick is moved.

## Udev Rules
See `udev_rules/` for example rules to create a stable device symlink.