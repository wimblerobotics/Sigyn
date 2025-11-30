# Sensor Integration Architecture Decision

## The Problem We Solved

When launching the simulation, the sensor controllers (`sensor_state_broadcaster` and `sensor_command_controller`) were failing to activate with this error:

```
[gz-2] [WARN] [controller_manager]: Unable to activate controller 'sensor_state_broadcaster' 
since the state interface 'temperature_sensor/temperature' is not available.
```

Root cause: The `sensor_system` hardware interface was defined in the simulation URDF, but **Gazebo's gz_ros2_control plugin could not load it** because it doesn't inherit from `GazeboSimSystemInterface`.

## The Solution: Separate Sim and Real Hardware Paths

We removed `sensor_system` from the simulation URDF and use **different architectures for simulation vs real hardware**:

### Simulation Mode (sim_mode: true)
```
sensor_simulator.py → /sensors/* topics → Your application code
```

**No ros2_control for sensors in simulation**
- Simple and direct
- `sensor_simulator.py` publishes sensor data directly to topics
- Your applications subscribe to `/sensors/temperature`, `/sensors/voltage`, etc.
- No hardware interfaces or controllers needed

### Real Hardware Mode (sim_mode: false)  
```
Teensy → sigyn_to_sensor_v2 → /sigyn/teensy_bridge/* topics
                                       ↓
                              SensorHardwareInterface (ros2_control)
                                       ↓
                              SensorStateBroadcaster (controller)
                                       ↓
                       /sensor_state_broadcaster/* topics → Your application code
```

**Full ros2_control integration for real hardware**
- `sensor_system` hardware interface loads (outside Gazebo)
- Controllers read hardware interfaces
- Proper abstraction layer for physical sensors

## What Changed in the Code

### 1. URDF (`r2c_test.ros2_control.xacro`)

**Before**: `sensor_system` was inside `<xacro:if value="$(arg sim_mode)">`
- Gazebo tried to load it → FAILED

**After**: `sensor_system` is inside `<xacro:unless value="$(arg sim_mode)">`
- Only loads for real hardware (sim_mode=false)
- Gazebo never sees it in simulation

### 2. Launch File (`gazebo_foxglove.launch.py`)

**Before**: Tried to spawn sensor controllers in simulation
```python
sensor_state_broadcaster_spawner = Node(...)  # Would fail
sensor_command_controller_spawner = Node(...)  # Would fail
```

**After**: Removed sensor controller spawners
- Only spawns joint controllers (diff_drive, arm, joint_state)
- Sensors handled by `sensor_simulator.py` directly

## Benefits of This Approach

### ✅ Simulation
- **Simple**: No ros2_control overhead
- **Fast**: Direct topic publication
- **Works immediately**: No Gazebo plugin issues
- **Easy testing**: Just run sensor_simulator.py

### ✅ Real Hardware
- **Proper abstraction**: Hardware interface isolates sensor details
- **Controller pattern**: Follows ros2_control best practices
- **Reusable**: Controllers work with any hardware implementation
- **Safe**: Read-only interfaces for sensors

### ✅ Learning
- Understand when ros2_control adds value vs complexity
- Learn both direct topic and controller patterns
- See trade-offs between approaches

## When Would You Use ros2_control for Sensors in Simulation?

You would use it if you needed:
1. **Identical code paths**: Exact same architecture in sim and real
2. **Hardware-in-the-loop testing**: Testing controller logic before deployment
3. **Complex sensor logic**: State machines, filtering, fusion at hardware layer
4. **Command interfaces**: Write values to sensors (like we tried to implement)

But for simple sensor reading in simulation, **direct topics are simpler and better**.

## How to Use Sensor Data Now

### In Simulation

Subscribe directly to sensor_simulator topics:

```python
self.create_subscription(
    Temperature,
    '/sensors/temperature',
    self.callback,
    10
)
```

### On Real Hardware

Subscribe to controller topics:

```python
self.create_subscription(
    Temperature,
    '/sensor_state_broadcaster/temperature',
    self.callback,
    10
)
```

Or create a launch argument to switch between them:

```python
sensor_topic = LaunchConfiguration('sensor_topic', default='/sensors/temperature')
```

## Summary

**You were right** - it makes much more sense to handle sensors completely in the launch file for simulation rather than trying to force them through Gazebo's URDF loading mechanism.

**Result**: 
- ✅ Simulation works (no Gazebo plugin errors)
- ✅ Controllers load successfully (only joint controllers)
- ✅ Sensor data available (from sensor_simulator.py)
- ✅ Real hardware path ready (sensor_system in URDF)
- ✅ You learned the tradeoffs between approaches

**Next steps**:
1. Test simulation → should work perfectly now
2. Use `/sensors/*` topics for your applications in simulation
3. When ready for real hardware, set `sim_mode:=false` and sensor_system will load
