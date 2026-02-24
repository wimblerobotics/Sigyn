# The repository for the Sigyn robot.
## Motivation
Worldwide, the number of elderly people (65 years and older) surpassed the number of children under 5 years of age in 2019. It is likely, in my lifetime, that there will be more people needing assistance in their lives than there will people who can provide it. I don’t expect to be able to afford especially extraordinary care, nor do I expect not to need it as I age. My alternative is to create the needed technology so it will be there when I need it.
<div style="text-align: center;">
  <img src="Media/Pictures/historic-and-un-pop-projections-by-age.webp" alt="alt text">
  <p>Source: <a href="https://ourworldindata.org/population-aged-65-outnumber-children">https://ourworldindata.org/population-aged-65-outnumber-children</a></p>
</div>

***Wimble Robotics*** is a one-person effort to create an assistive robot for my personal use.


## Organization
The various directories contain:

### Design and Infrastructure
* **Designs**<br/>
    Holds various design documents (Fusion 360, LightBurn, KiCad, EasyEDA).
* **Docker**<br/>
    For creating docker images to run the code.
* **Documentation**<br/>
    Documentation artifacts, notes, wireviz documentation, and technical guides.
* **Media**<br/>
    Pictures, movies, and other media assets.

### Core ROS2 Packages
* **base**<br/>
    Main launch files, navigation configuration, maps, and system integration.
* **description**<br/>
    Robot URDF models and Gazebo simulation worlds.
* **sigyn_interfaces**<br/>
    Custom message, action, and service definitions for ROS2.

### Navigation and Behavior
* **sigyn_behavior_trees**<br/>
    Behavior tree implementations for autonomous operation.
* **sigyn_house_patroller**<br/>
    House patrol logic and threat detection.
* **sigyn_nav_goals**<br/>
    Navigation goal management and waypoint definitions.
* **perimeter_roamer** / **perimeter_roamer_v3**<br/>
    Perimeter patrolling behaviors and algorithms.

### Sensors and Hardware Interfaces
* **ldlidar**<br/>
    Forked LIDAR driver with fixes and customizations.
* **oakd_detector**<br/>
    OAK-D camera integration for object detection.
* **sigyn_to_sensor_v2**<br/>
    ROS2 bridge to Teensy microcontroller for sensor data.
* **sigyn_to_elevator**<br/>
    Elevator control interface.
* **TeensyV2**<br/>
    Teensy 4.1 firmware for sensor management and motor control.

### Control and Input
* **bluetooth_joystick**<br/>
    Bluetooth game controller integration for manual control.
* **teleop_twist_keyboard**<br/>
    Keyboard teleoperation interface (forked with customizations).
* **twist_multiplexer**<br/>
    Command multiplexer for multiple control sources (forked with fixes).

### Utilities and Tools
* **scripts** (py_scripts package)<br/>
    Utility scripts including battery overlay publisher, WiFi monitoring, and diagnostic tools.
* **rviz**<br/>
    RViz2 configuration files.
* **udev**<br/>
    udev rules for hardware device management and symbolic links.

### Web and Monitoring
* **sigyn_web_client**<br/>
    Web-based monitoring and control interface.
* **sigyn_websocket**<br/>
    WebSocket server for real-time robot communication.

### Experimental and Development
* **experiments**<br/>
    Ongoing experiments and proof-of-concept code.
* **msgs**<br/>
    Internal message definitions (deprecated - use sigyn_interfaces).

## Building the Workspace

This is a standard ROS2 package. Build it with colcon and source the setup file.

### Install Dependencies

Use rosdep to install all required dependencies:

```bash
cd ~/sigyn_ws
rosdep install --from-paths src --ignore-src -r -y
```

Key dependencies include:
- **Nav2**: Navigation stack (`ros-jazzy-nav2-bringup`)
- **Robot Localization**: Sensor fusion (`ros-jazzy-robot-localization`)
- **RViz Overlays**: Battery and status displays (`ros-jazzy-rviz-2d-overlay-plugins`)
- **OAK-D Camera**: DepthAI ROS driver (`ros-jazzy-depthai-ros`)
- **LIDAR**: LD LIDAR drivers (included in repository)

### Build the Workspace

```bash
cd ~/sigyn_ws
colcon build --symlink-install
source install/setup.bash
```

The `--symlink-install` flag allows you to edit Python scripts and launch files without rebuilding.

## Running the Robot

### Prerequisites

Make sure udev rules are installed for hardware device access:
```bash
sudo cp udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Launch the Robot

**Option 1: Complete System (Recommended)**

Run everything in one launch file:
```bash
ros2 launch sigyn_bringup sigyn.launch.py use_sim_time:=false do_rviz:=true
```

This automatically launches:
- Robot state publisher
- Navigation stack (Nav2)
- Sensor bridges (Teensy, LIDAR, cameras)
- Robot localization (EKF)
- Battery overlay display
- RViz visualization

**Option 2: Manual Launch**

If you need more control, launch components separately:

1. **Main System** (without RViz):
   ```bash
   ros2 launch sigyn_bringup sigyn.launch.py use_sim_time:=false do_rviz:=false
   ```

2. **RViz** (separate window):
   ```bash
   rviz2 -d ~/sigyn_ws/src/Sigyn/rviz/config/config.rviz
   ```

Note: The Teensy sensor bridge (`sigyn_to_sensor_v2`) launches automatically with the main system.

### Using the Robot

1. **Set Initial Pose**: Use RViz "2D Pose Estimate" tool to set the robot's starting position
2. **Test Localization**: Use teleoperation to move the robot and verify localization is working
3. **Set Goal**: Use RViz "Nav2 Goal" tool to send navigation goals

**Teleoperation**:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Monitoring

The robot displays real-time information in RViz:
- **Battery Status**: Top-left overlay shows voltage and percentage
- **Sensor Data**: LIDAR scans, camera images, IMU orientation
- **Navigation**: Costmaps, paths, and local/global plans

## Running the Robot in Simulation
The robot can be run in simulation by running the following command:
``` bash
ros2 launch sigyn_bringup sigyn.launch.py use_sim_time:=true
```

## Recent Updates

### Battery Overlay Visualization (October 2025)
- **RViz Battery Display**: Real-time battery status overlay in RViz viewport
- **Multi-Battery Support**: Filters specific battery (36VLIPO) from multi-battery status topic
- **Color-Coded Display**: Green (>50%), Yellow (20-50%), Red (<20%) battery levels
- **Standard ROS2 Plugin**: Uses `rviz_2d_overlay_plugins` for reliable rendering
- **Automatic Launch**: Battery overlay launches automatically with `sigyn.launch.py`
- **Implementation**: Battery publisher located at `scripts/py_scripts/battery_overlay_publisher.py`

### Localization and Sensor Fusion (October 2025)
- **Multi-Height Lidar Strategy**: Upper lidar (178cm) for clean localization, lower lidar (30cm) for obstacle detection
- **Dual IMU Integration**: BNO055 sensors with explicit unit configuration and mount-aware fusion
- **EKF Optimization**: Wheel odometry + IMU fusion for stable, drift-free navigation
- **AMCL Tuning**: Optimized for dual-lidar system with dynamic environment handling

For comprehensive documentation on localization, sensor fusion, and configuration decisions, see:
- [`Documentation/LocalizationAndSensorFusion.md`](Documentation/LocalizationAndSensorFusion.md) - Complete guide with lessons learned

### TeensyV2 System Enhancements
- **Modular Embedded Architecture**: Updated TeensyV2 system with improved real-time performance
- **Enhanced Battery Monitoring**: Battery messages now include location information for multi-battery setups
- **Streamlined ROS2 Bridge**: Consolidated monitoring into a single efficient bridge node
- **Updated Message Protocol**: Comprehensive documentation of communication protocol with examples

For detailed information about the TeensyV2 system and message protocol, see:
- [`TeensyV2/README.md`](TeensyV2/README.md) - Complete embedded system documentation
- [`TeensyV2/docs/MessageProtocol.md`](TeensyV2/docs/MessageProtocol.md) - Communication protocol specification
- [`sigyn_to_sensor_v2/README.md`](sigyn_to_sensor_v2/README.md) - ROS2 bridge documentation
