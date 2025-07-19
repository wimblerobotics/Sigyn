# Migration to sigyn_to_sensor_v2

This document outlines the migration from the legacy `sigyn_to_teensy` and `sigyn_to_sensor` packages to the new, unified `sigyn_to_sensor_v2` package. This migration was undertaken to modernize the communication architecture with the Teensy microcontrollers, improve performance, and simplify the overall system design.

## 1. Key Improvements

- **Unified Communication:** A single ROS 2 package now manages communication with all Teensy boards, including the main sensor board, the secondary sensor board, and the gripper controller.
- **Standardized Messaging:** A flexible and extensible messaging protocol has been implemented, allowing for more complex and robust data exchange between the host and the embedded systems.
- **Improved Performance:** The new architecture leverages modern C++ features and ROS 2 best practices to enhance performance and reduce latency.
- **Simplified Configuration:** All Teensy-related configurations are now centralized in a single launch file and YAML configuration file, making it easier to manage and modify system parameters.
- **Enhanced Diagnostics:** The `sigyn_to_sensor_v2` package includes comprehensive diagnostic reporting, providing detailed insights into the status and performance of the embedded systems.

## 2. Architectural Changes

The `sigyn_to_sensor_v2` package introduces a modular and scalable architecture designed to accommodate future expansion. The core components of this new architecture are:

- **`teensy_bridge_node`:** The central communication hub that interfaces directly with the Teensy boards via serial communication. It is responsible for reading, parsing, and distributing messages to the appropriate ROS 2 topics.
- **Specialized Nodes:** Dedicated nodes for `battery_monitor`, `performance_monitor`, and `safety_coordinator` process the data received from the `teensy_bridge_node` and publish relevant information to the rest of the ROS 2 system.
- **Composable Nodes:** For improved performance, the system can be launched with composable nodes, which run within a single process to minimize inter-process communication overhead.

## 3. Configuration

The `sigyn_to_sensor_v2` package is configured through the `teensy_bridge.launch.py` file and the `teensy_v2_config.yaml` file. The launch file defines the nodes to be launched and allows for the configuration of serial ports and other parameters. The YAML file contains detailed settings for each of the specialized nodes.

### udev Rules

To ensure stable and predictable communication with the Teensy boards, new `udev` rules have been implemented. These rules create symbolic links (`/dev/teensy_sensor`, `/dev/teensy_sensor2`, `/dev/teensy_gripper`) for each Teensy device based on its unique serial number. This approach eliminates the need to rely on unpredictable device names like `/dev/ttyACMX`.

The updated `udev` rules can be found in the `Sigyn/udev/00-teensy.rules` file. These rules must be copied to `/etc/udev/rules.d/` and the `udev` service must be reloaded for the changes to take effect.

## 4. How to Use

To launch the new Teensy communication system, use the following command:

```bash
ros2 launch sigyn_to_sensor_v2 teensy_bridge.launch.py
```

This will start all the necessary nodes and establish communication with the Teensy boards. The launch file can be customized to enable or disable features such as diagnostics and node composition.

This migration represents a significant step forward in the evolution of the Sigyn platform. By consolidating and modernizing the Teensy communication system, we have created a more robust, scalable, and maintainable foundation for future development.
