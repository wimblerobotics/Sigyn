# Bluetooth Joystick ROS 2 Node

This ROS 2 package provides a node for interfacing with a Bluetooth joystick (tested with Nimbus SteelSeries) and publishing control messages for robot operation.

## Features
- Publishes a custom `BluetoothJoystick` message with all joystick and button states.
- Publishes `cmd_vel` (base movement) from the left joystick at a configurable rate, only when moved.
- Publishes `cmd_vel_gripper` (or a configurable topic) from the right joystick at a configurable rate, only when moved.
- All button events are published.
- Status messages (`BluetoothJoystick`) are published at a configurable rate.

## Limitations
- The MENU and D-Pad buttons are not handled by this code. The D-Pad is not published by the Linux Bluetooth driver for this device, and MENU is not handled.

## Configuration
Edit `config/bluetooth_joystick.yaml` to set device and topic names, deadzone, scaling, and message rates:

```yaml
bluetooth_joystick:
  ros__parameters:
    device_name: "/dev/input/nimbus_steelseries"
    cmdvel_topic: "cmd_vel_joystick"
    gripper_topic: "cmd_vel_gripper"
    deadzone_percent: 5.0
    scale_x: 0.25
    scale_z: 0.0625
    cmdvel_message_rate: 30         # Hz, for base movement
    gripper_message_rate: 600       # Hz, for gripper control
    joystick_message_rate: 100      # Hz, for status messages
```

## Launch
To start the node with your configuration:
```bash
ros2 launch bluetooth_joystick bluetooth_joystick.launch.py
```

## Notes
- The left joystick is mapped to axes 0 (left/right) and 1 (up/down) for base movement.
- The right joystick is mapped to axes 2 (left/right) and 3 (up/down) for gripper control (check your device mapping).
- The node publishes the custom `BluetoothJoystick` message at the configured rate, and only publishes `cmd_vel` or `gripper_topic` if the respective stick is moved.
- See `udev_rules/` for example rules to create a stable device symlink for your joystick.

## Troubleshooting
- Use `jstest /dev/input/nimbus_steelseries` to verify joystick axes and button mappings.
- If the D-Pad or MENU button do not generate events, they are not supported by the Linux driver for this device.

## License
MIT