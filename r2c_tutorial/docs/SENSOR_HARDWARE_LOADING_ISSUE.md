# Sensor Hardware Interface Loading Issue

## Problem

The `sensor_system` hardware interface (defined in `r2c_test.ros2_control.xacro`) **cannot be loaded by Gazebo's gz_ros2_control plugin** because:

1. Gazebo's plugin expects all hardware interfaces to inherit from `gz_ros2_control::GazeboSimSystemInterface`
2. Our `SensorHardwareInterface` inherits from `hardware_interface::SystemInterface`
3. Gazebo tries to load BOTH `r2c_test_system` and `sensor_system` but fails on the latter

Error message:
```
[gz_ros_control.GZResourceManager]: The plugin failed to load... 
class r2c_tutorial/SensorHardwareInterface with base class type 
gz_ros2_control::GazeboSimSystemInterface does not exist.
```

Result: **NO sensor state/command interfaces are available**, so sensor controllers fail to activate.

## Solutions

### Option 1: Separate Sensor URDF (Cleanest)

Create a separate URDF for sensors that's loaded by a standalone controller_manager instance.

**Pros**: Clean separation, follows ros2_control architecture
**Cons**: More complex launch configuration

### Option 2: Remove Sensors from Simulation URDF (Simplest)

Comment out `<ros2_control name="sensor_system">` from the simulation URDF. Controllers read directly from `/sensors/*` topics.

**Pros**: Simple, works immediately
**Cons**: Sensors don't go through ros2_control in simulation

### Option 3: Make SensorHardwareInterface Inherit from GazeboSimSystemInterface

Modify the C++ class to inherit from Gazebo's base class.

**Pros**: Single hardware interface works everywhere
**Cons**: Couples sensor code to Gazebo, more complex

### Option 4: Use Mock Hardware for Sensors in Simulation

Use `mock_components/GenericSystem` for sensors in sim mode.

**Pros**: Standard ros2_control approach
**Cons**: Requires mock hardware configuration

## Recommended Quick Fix

For your immediate needs, **Option 2** is fastest. Here's what to do:

1. Comment out the sensor_system from the URDF in simulation mode
2. Have sensor controllers read directly from topics (not ros2_control)
3. Use ros2_control for sensors only on real hardware

This way:
- Simulation works immediately (controllers read `/sensors/*` topics)
- Real hardware uses proper ros2_control integration
- You learn both approaches

## Implementation

I'll create a modified version that uses Option 2 for now, but sets you up to easily switch to Option 1 (separate URDF) when you're ready for production.
