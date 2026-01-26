# Bluetooth Joystick ROS 2 Node

This ROS 2 package provides a node for interfacing with a Bluetooth joystick (tested with Nimbus SteelSeries and 8BitDo SN 30 Pro) and publishing control messages for robot operation.

## Features
- Publishes a custom `BluetoothJoystick` message with all joystick and button states.
- Publishes `cmd_vel_joystick` (base movement) from the left joystick at a configurable rate, when the deadman switch (L1 switch, top front left of joystick controller) is pressed.
- When the deadman switch (left joystick) is released, a zeroed `cmd_vel_joystick` message is sent to stop the robot.
- Publishes `cmd_vel_gripper` (or a configurable topic) from the right joystick at a configurable rate, when the deadman switch is pressed.
- Publishes to a configurable `cmd_vel_testicle_twister` topic when R1 or R2 is pressed (with L1 deadman), with configurable values for gripper control.
- X button triggers OAK-D camera picture capture (publishes boolean true to `/sigyn/take_oakd_picture`).
- B button triggers gripper camera picture capture (publishes boolean true to `/sigyn/take_gripper_picture`).
- All button events are published to `bluetoothJoystick` at a configurable rate.
- All rates and topic names are set in the YAML config.
- Thread-safe event handling for all control surfaces and buttons.

## Limitations
- The MENU and D-Pad buttons are not handled by this code. The D-Pad is not published by the Linux Bluetooth driver for this device, and MENU is not handled.

## Configuration
Edit `config/bluetooth_joystick.yaml` to set device and topic names, deadzone, scaling, message rates, and twister values:

```yaml
bluetooth_joystick:
  ros__parameters:
    device_name: "/dev/input/nimbus_steelseries"
    cmdvel_topic: "cmd_vel_joystick"
    gripper_topic: "cmd_vel_gripper"
    deadzone_percent: 5.0
    scale_x: 0.25
    scale_z: 0.25
    cmdvel_message_rate: 30         # Hz, for base movement
    gripper_message_rate: 600       # Hz, for gripper control
    joystick_message_rate: 100      # Hz, for status messages
    cmdvel_twister_topic: "cmd_vel_testicle_twister"  # Topic for R1/R2 events (with L1 deadman)
    gripper_open_value: 1000        # Value sent when R1 is pressed (with L1 deadman)
    gripper_close_value: -1000      # Value sent when R2 is pressed (with L1 deadman)
    oakd_picture_topic: "/sigyn/take_oakd_picture"      # Topic for X button camera trigger
    gripper_picture_topic: "/sigyn/take_gripper_picture" # Topic for B button camera trigger
```

## Launch
To start the node with your configuration:
```bash
ros2 launch bluetooth_joystick bluetooth_joystick.launch.py
```

## Notes
- The left joystick is mapped to axes 0 (left/right) and 1 (up/down) for base movement.
- The right joystick is mapped to axes 2 (left/right) and 3 (up/down) for gripper control (check your device mapping).
- The node publishes the custom `bluetoothJoystick` message at the configured rate, and only publishes `cmd_vel_joystick` or `gripper_topic` if the deadman switch (L1) is pressed.
- When the deadman switch is released, a zeroed `cmd_vel_joystick` message is sent to stop the robot.
- R1 and R2 button presses (with L1 deadman) are published to the `cmd_vel_twister_topic` with configurable values for gripper open/close.
- X button triggers OAK-D camera capture (no deadman required).
- B button triggers gripper camera capture (no deadman required).
- See `udev_rules/` for example rules to create a stable device symlink for your joystick.

## Button Mapping (8BitDo SN30 Pro)
- **L1** (left shoulder): Deadman switch for base movement and gripper control
- **Left Stick**: Base movement (linear/angular velocity)
- **Right Stick**: Gripper control
- **R1** (right shoulder, with L1): Open gripper
- **R2** (right trigger, with L1): Close gripper
- **X**: Take OAK-D picture
- **B**: Take gripper camera picture

## Troubleshooting
- Use `jstest /dev/input/nimbus_steelseries` to verify joystick axes and button mappings.
- If the D-Pad or MENU button do not generate events, they are not supported by the Linux driver for this device.
- If the device path is incorrect, update `device_name` in the YAML config.
- For ROS 2 best practices and further details, see the comments in the YAML and source code.

## License
MIT