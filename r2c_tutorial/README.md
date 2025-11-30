# ROS2 Control Tutorial Package

A comprehensive, hands-on tutorial for learning ros2_control from basics to advanced topics, designed to prepare you for migrating the Sigyn robot to a ros2_control-based architecture.

## 📚 What You'll Learn

- **Fundamentals:** ros2_control architecture, hardware interfaces, controllers, and the controller manager
- **Practical Skills:** Configure differential drive robots, position-controlled arms, and sensor integration
- **Sensor Testing:** Inject test values to validate safety systems in simulation
- **Real-World Application:** Step-by-step guide to migrate Sigyn from custom control to ros2_control
- **Best Practices:** Real-time performance, error handling, testing, and debugging

## 🤖 Tutorial Robot

The tutorial uses a simplified robot with:
- **Differential drive base:** 2 driven wheels + 1 caster wheel
- **1-DOF arm:** Revolute joint for learning position control
- **Multiple sensors:** Temperature, battery (voltage/current), range sensor
- **Gazebo simulation:** Full ros2_control integration with test command injection
- **Dual mode:** Same controllers work in both simulation and real hardware

## 📖 Documentation Structure

### 1. **[TUTORIAL.md](docs/TUTORIAL.md)** - Start Here!
Complete tutorial from basics to advanced:
- **Phase 1:** Understanding the Basics (URDF, ros2_control, controllers)
- **Phase 2:** Exploring Controllers (diff_drive, joint_trajectory, switching)
- **Phase 3:** Creating Custom Hardware Interfaces
- **Phase 4:** Integration with Sigyn (planning and strategy)

### 2. **[SENSOR_TESTING_GUIDE.md](docs/SENSOR_TESTING_GUIDE.md)** - Testing Safety Systems
Complete guide for testing safety systems:
- **Architecture:** How sensor command injection works
- **Python Examples:** Ready-to-use test scripts
- **C++ Examples:** Production-quality test code
- **Test Scenarios:** Thermal, battery, overcurrent, collision tests
- **Best Practices:** Test automation and validation

### 3. **[ADVANCED_TOPICS.md](docs/ADVANCED_TOPICS.md)**
Deep dives into:
- Real-time performance optimization
- Custom controller development
- Sensor integration patterns
- Multi-interface hardware
- Error handling and recovery
- Performance tuning

### 4. **[MIGRATION_GUIDE.md](docs/MIGRATION_GUIDE.md)**
Sigyn-specific migration plan:
- Current system analysis
- Target architecture
- Phase-by-phase migration steps
- Testing strategy
- Rollback plan

## 🚀 Quick Start

### Build the Package

```bash
cd ~/sigyn_ws
colcon build --packages-select r2c_tutorial --symlink-install
source install/setup.bash
```

### Run the Simulation

```bash
# Terminal 1: Launch Gazebo simulation with robot and controllers
ros2 launch r2c_tutorial gazebo_foxglove.launch.py

# Terminal 2: Monitor sensor data
ros2 run r2c_tutorial sensor_reader.py

# Terminal 3: Drive the robot with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Test safety systems by injecting sensor values
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=93.0
```

### Test Safety Systems

```bash
# Inject high temperature to test thermal protection
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=93.0

# Test low battery protection
ros2 run r2c_tutorial sensor_writer.py --ros-args -p voltage_cmd:=9.5

# Test overcurrent protection
ros2 run r2c_tutorial sensor_writer.py --ros-args -p current_cmd:=10.0

# Reset to normal simulation
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=0.0

# See SENSOR_TESTING_GUIDE.md for complete testing documentation
```

### Expected Output

- **Gazebo:** Robot model moving in empty world
- **Sensor Reader:** Live sensor data with safety warnings
- **RViz (via Foxglove):** Visualization with TF frames and odometry trail
- **Terminal:** Real-time joint states, odometry, and sensor readings

## 📁 Package Structure

```
r2c_tutorial/
├── config/
│   ├── controllers.yaml         # Controller configurations
│   └── r2c_tutorial.rviz        # RViz visualization config
├── docs/
│   ├── TUTORIAL.md              # Main tutorial (START HERE)
│   ├── SENSOR_TESTING_GUIDE.md  # Testing safety systems
│   ├── ADVANCED_TOPICS.md       # Advanced concepts
│   ├── MIGRATION_GUIDE.md       # Sigyn migration plan
│   └── SENSOR_INTEGRATION_GUIDE.md  # Complete sensor architecture
├── include/r2c_tutorial/
│   ├── sensor_hardware_interface.hpp       # Real hardware interface
│   ├── sensor_hardware_interface_sim.hpp   # Simulation interface
│   ├── sensor_state_broadcaster.hpp        # Reads & publishes sensors
│   └── sensor_command_controller.hpp       # Injects test values
├── launch/
│   ├── gazebo_foxglove.launch.py    # Full simulation with Foxglove
│   └── manual_control.launch.py     # URDF testing without Gazebo
├── scripts/
│   ├── sensor_reader.py         # Monitor sensor data
│   ├── sensor_writer.py         # Inject test values
│   └── monitor_robot.py         # Real-time monitoring tool
├── src/
│   ├── sensor_hardware_interface.cpp       # Real hardware implementation
│   ├── sensor_hardware_interface_sim.cpp   # Simulation implementation
│   ├── sensor_state_broadcaster.cpp        # State publisher
│   └── sensor_command_controller.cpp       # Command subscriber
├── urdf/
│   ├── r2c_test.xacro           # Robot description
│   ├── r2c_test.ros2_control.xacro  # ros2_control interfaces
│   ├── r2c_test.gazebo.xacro    # Gazebo-specific config
│   └── inertial_macros.xacro    # Inertia calculations
├── worlds/
│   └── empty.world              # Simple Gazebo world
├── CMakeLists.txt
├── package.xml
└── README.md                    # This file
```

## 🎓 Learning Path

### Beginner (1-2 days)

1. **Read [TUTORIAL.md](docs/TUTORIAL.md) Phase 1**
   - Understand URDF structure
   - Learn ros2_control interfaces
   - Configure basic controllers

2. **Run the simulation**
   ```bash
   ros2 launch r2c_tutorial gazebo_sim.launch.py
   ```

3. **Complete exercises in TUTORIAL.md**
   - Modify robot parameters
   - Test controller configurations
   - Observe behavior changes

### Intermediate (2-3 days)

1. **Read [TUTORIAL.md](docs/TUTORIAL.md) Phase 2**
   - Deep dive into diff_drive_controller
   - Understand trajectory control
   - Learn controller switching

2. **Read [ADVANCED_TOPICS.md](docs/ADVANCED_TOPICS.md)**
   - Real-time performance
   - Sensor integration patterns

3. **Experiment with custom parameters**
   - Tune controller gains
   - Test different update rates
   - Add a new sensor to the URDF

### Advanced (3-5 days)

1. **Read [TUTORIAL.md](docs/TUTORIAL.md) Phase 3**
   - Study hardware interface structure
   - Understand read/write loops
   - Examine Sigyn's hardware interface

2. **Read [MIGRATION_GUIDE.md](docs/MIGRATION_GUIDE.md)**
   - Study Sigyn's current architecture
   - Understand target architecture
   - Plan migration phases

3. **Begin Sigyn migration**
   - Follow phase-by-phase guide
   - Test incrementally
   - Document results

## 🧪 Testing and Validation

### Check Controllers

```bash
# List all controllers
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces

# View controller configuration
ros2 param dump /controller_manager
```

### Monitor Performance

```bash
# Monitor joint states
ros2 topic echo /joint_states

# Monitor odometry
ros2 topic echo /diff_drive_controller/odom

# Monitor temperature sensor
ros2 topic echo /temperature_sensor/raw

# Use custom monitor
ros2 run r2c_tutorial monitor_robot.py
```

### Drive the Robot

```bash
# Keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" --rate 10
```

### Control the Arm

```bash
# Move to 90 degrees
ros2 topic pub /arm_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_joint'], points: [{positions: [1.57], time_from_start: {sec: 2}}]}" \
  --once

# Return to home
ros2 topic pub /arm_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_joint'], points: [{positions: [0.0], time_from_start: {sec: 2}}]}" \
  --once
```

## 🔧 Common Issues and Solutions

### Issue: Gazebo doesn't start

**Solution:**
```bash
# Check if gz_ros2_control is installed
apt list --installed | grep gz-ros2-control

# Install if missing
sudo apt install ros-jazzy-gz-ros2-control
```

### Issue: Controllers don't load

**Solution:**
```bash
# Check controller_manager is running
ros2 node list | grep controller_manager

# Check for error messages
ros2 launch r2c_tutorial gazebo_sim.launch.py 2>&1 | grep -i error
```

### Issue: Robot doesn't move

**Solution:**
```bash
# Verify controllers are active
ros2 control list_controllers

# Manually activate if needed
ros2 control set_controller_state diff_drive_controller active

# Check cmd_vel is being published
ros2 topic hz /cmd_vel
```

### Issue: Odometry drifts

**Solution:**
```yaml
# Tune multipliers in config/controllers.yaml
wheel_separation_multiplier: 1.02  # If turning too much
left_wheel_radius_multiplier: 0.98  # If left wheel overcounts
```

## 📚 Additional Resources

### Official Documentation
- [ros2_control Documentation](https://control.ros.org/)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [Gazebo ROS2 Control](https://github.com/ros-controls/gz_ros2_control)

### Related Sigyn Packages
- `sigyn_hardware_interface/` - Real hardware interface implementation
- `base/` - Main Sigyn launch and configuration
- `TeensyV2/` - Embedded firmware

### Tutorials and Examples
- [DiffBot Example](https://github.com/ros-controls/ros2_control_demos)
- [ros2_control Demos](https://control.ros.org/master/doc/ros2_control_demos/doc/index.html)

## 🤝 Contributing

This tutorial is part of the Sigyn project. Improvements and suggestions are welcome!

## 📝 License

MIT License - See Sigyn project LICENSE file

## 🙏 Acknowledgments

- **BasicMicro RoboClaw:** High-performance motor controllers
- **Teensy 4.1:** Real-time embedded processing
- **ros2_control community:** Excellent framework and documentation

---

## 🎯 Next Steps

1. **Complete the tutorial:** Work through [TUTORIAL.md](docs/TUTORIAL.md) systematically
2. **Understand advanced concepts:** Read [ADVANCED_TOPICS.md](docs/ADVANCED_TOPICS.md)
3. **Plan your migration:** Review [MIGRATION_GUIDE.md](docs/MIGRATION_GUIDE.md)
4. **Ask questions:** Document issues and solutions as you learn

**Remember:** The goal is to deeply understand ros2_control so you can confidently migrate Sigyn and troubleshoot issues. Take your time, experiment, and learn by doing!

**Happy learning! 🚀**
