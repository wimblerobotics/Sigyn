# Copilot Agent Productivity Guide for Sigyn House Patroller

This guide enables AI agents to work productively in the Sigyn workspace. It summarizes architecture, workflows, conventions, and integration points unique to this codebase.

## 1. Colcon build instructions
Always include the command-line option '--symlink-install' when building the package to ensure that the agent can access the latest code changes without needing to rebuild the entire workspace. This is crucial for rapid development and testing.

```bash
colcon build --symlink-install
```

## 1. Big Picture Architecture
- **base**
  Base package for the Sigyn robot, providing core functionality and integration with ROS2. The main launch for the robot is `sigyn.launch.py`, which brings up all the components needed for the robot to operate, including navigation, perception, and control systems. The command line option `use_sim_time:=true` is used to bring up the robot in simulation mode, allowing for testing and development without requiring the physical robot.

- **bluetooth_joystick**
  Provides Bluetooth joystick control for the Sigyn robot. The main launch file is `bluetooth_joystick.launch.py`, which sets up the joystick interface and connects it to the robot's control systems.

- **description**
  Contains the robot's URDF files and gazebo simulation models.

- **Designs**
  Contains design artifacts, such as Fusion 360 files, that are used to create the robot's physical components, EasyEDA files for electronic components, and other design-related documents, KiCad files for PCB designs, and other design-related documents, and other design-related documents.

- **Docker**
  Contains Dockerfiles and related configurations for building and running the Sigyn robot in containerized environments.

- **Documentation**
  Contains documentation for the Sigyn robot, including architecture diagrams, design documents, and other related materials. The `AI` subdirectory contains architecture and design documents specifically for AI-related components.

- **experiments**
  Contains experimental code and configurations for testing new features or components of the Sigyn robot.

- **gripper**
  Contains the Teensy 4.1 code for the gripper assembly.

- **ldlidar**
  Contains a clone of the LIDAR driver code from the `ldlidar` package, which is used to interface with the LIDAR sensor on the Sigyn robot. I made customizations to the code to work with the specific LIDAR model used on the Sigyn robot.

- Media
  Contains media files related to the Sigyn robot, such as images, videos, and other media assets.

- **min_max_curr_rviz_overlay**
  Contains the RViz overlay for visualizing the minimum and maximum current readings from the robot's sensors. This is useful for monitoring the robot's power consumption and ensuring that it operates within safe limits.

- **oakd_detector**
  Contains the OAK-D detector code, which is used to detect objects and people in the robot's environment using the OAK-D camera.

- **perimeter_roamer**
  Contains the code for the perimeter roamer, which controls the robot to patrol the perimeter of the house.imeter roamer and its associated components.

- **pi_servo1**
  Contains the Teensy 4.1 code for the Pi Servo 1, which is used to control the robot's gripper and other components.
- **rviz**
  Contains RViz configuration files and related resources for visualizing the robot's state and environment.

- **scripts**
  Contains various scripts.

- **sigyn_behavior_trees**
  Contains the behavior tree implementation for the Sigyn robot, which is used to control the robot's behavior and decision-making processes.

- **sigyn_house_patroller**
  Contains the main code for the Sigyn house patroller, which is responsible for navigating the house, detecting changes, and sending notifications. This package includes the core logic for patrolling, navigation, and threat detection. It integrates with the ROS2 navigation stack and uses behavior trees for high-level decision making. The package is structured to allow for easy configuration and extension, with a focus on modularity and reusability.

- **sigyn_interfaces**
  Contains the action and service interfaces for the Sigyn robot, which define the communication protocols between different components of the robot.

- ** sigyn_nav_goals**
  Contains the navigation goals for the Sigyn robot, which define the specific locations and tasks that the robot should perform during its patrols.

- **sigyn_to_elevator**
  Contains the code for the Sigyn robot to interface with the elevator, 

- ** sigyn_to_sensor**
  Contains the code for the Sigyn robot to interface with various sensors, including temperature sensors, IMUs, and the RoboClaw.

- **Teensy**
  Design documents for the various Teensy 4.1 boards I designed.

- **teensy_monitor**
  Older code for controlling an earlier Teensy 4.1 board.

- **teensy_to_sigyn**
  Teensy 4.1 code for the main board.

- **TeensySensorBoard**
  Documentation for one of my custom Teensy 4.1 sensor boards.

- **teleop_twist_keyboard**
  Clone of the teleop_twist_keyboard package, which provides keyboard control for the Sigyn robot. I brought back code which was dropped.

- **this_to_that**
  Converts ROS2 messages to a different format.

- **twist_multiplexer**
  Clone of the original twist_multiplexer package, which is used to combine multiple twist messages into a single message for controlling the robot's movement. I made customizations to the code to work with the specific requirements of the Sigyn robot.

- **udev**
  Contains the udev rules for the Sigyn robot, which define how the robot's devices are recognized and configured by the operating system.

- **wall_finder**
  Contains the wall finder code, which is used to detect walls and obstacles in the robot's environment using LIDAR data. This is crucial for navigation and mapping tasks.

-- **wifi_logger_visualizer**
  Contains the Wi-Fi logger visualizer code, which is used to visualize the Wi-Fi signal strength and other related metrics in the robot's environment. This is useful for monitoring the robot's connectivity and ensuring that it can communicate effectively with other devices.

