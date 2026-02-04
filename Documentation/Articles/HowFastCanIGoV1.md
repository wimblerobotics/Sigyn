# How Fast Can I Go?
## Understanding Speed Limits for Mobile Robots
### Version 1, February 3, 2026
### (c) 2025 by Michael Wimble, ALL RIGHTS RESERVED

**Target Audience:** Hobbyist roboticists with basic to moderate ROS2 experience  
**Author:** Michael Wimble  
**Date:** February 2026  
**Robot Context:** Lessons from building Sigyn, an autonomous house patrol robot

---
## Caveat
Some of this article is based upon measurements I've gathered from my robot, "Sigyn". Some of the data is more speculative but I've tried to come up with reasonable, ball park figures. It isn't a matter of "your mileage may vary", it's very much a matter of using this as a starting point and coming up with your own data from your own measurements and research. This article was written with the help of AI.

## Introduction

If you're building a mobile robot, one of the most natural questions is: "How fast can I make it go?" You might think the answer depends on your motors, or your battery, or how much current your motor driver can handle. While those factors matter, **the real answer is much more interesting—and much more about timing than torque.**

The fundamental constraint isn't "how fast can your robot move?" It's **"how fast can your robot move without crashing into things?"** And that question leads us down a fascinating rabbit hole of sensor latencies, control loop frequencies, CPU performance, and the surprising impact of Linux's task scheduler.

This article will help you understand:
- Why your robot can't go as fast as you think it should
- Which factors actually limit your safe speed (hint: it's not your motors)
- How to calculate safe velocity for your specific robot
- The dramatic differences between a Raspberry Pi and a more powerful computer
- Why microcontrollers might be your best friend

Let's start with a simple thought experiment.

---

## The Fundamental Problem: Don't Hit Things

Imagine you're driving a car at highway speed when suddenly a deer jumps in front of you. Several things have to happen before you can avoid hitting it:

1. **Light from the deer reaches your eyes** (speed of light—essentially instant)
2. **Your brain processes what you're seeing** (~200ms for human reaction time)
3. **Your foot moves to the brake pedal** (~100ms)
4. **Brake pressure builds** (~50-100ms)
5. **Your car actually slows down** (depends on speed, weight, road conditions)

The total time from "deer appears" to "car starts slowing" might be 350-450ms. During that time, traveling at 60 mph (27 m/s), you've moved **about 30 feet (9 meters)** before you even start decelerating!

Your robot faces the exact same problem, except instead of deer, it's chairs, walls, pets, and people. Let's break down the robotic equivalent of each step.

---

## The Robot's Detection-to-Action Pipeline

For a typical ROS2-based mobile robot, the pipeline looks like this:

### 1. **Sensor Captures Data**

Let's say you're using a LIDAR (like the popular LD06):
- The LIDAR spins at 10 Hz (10 rotations per second)
- This means one complete scan takes **100 milliseconds**
- When an obstacle first appears, it might be at the angle the LIDAR just passed
- Worst case: you wait **100ms** before detecting it

**Key insight:** The spinning LIDAR creates "data age" from 0ms to 100ms depending on where in the scan the obstacle appears.

### 2. **Sensor Data Travels to Your Computer**

- USB communication: typically 1-3ms for LIDAR data
- ROS2 message serialization and transport: 1-5ms
- **Total: ~5ms** (usually negligible)

### 3. **Costmap Updates**

Your ROS2 Nav2 costmap combines sensor data into a map of obstacles:
- Typical update frequency: 10-15 Hz
- That's an update every **67-100ms**
- Worst case: obstacle appears right after an update, wait for next one

**Running total:** 100ms (LIDAR) + 5ms (transport) + 100ms (costmap wait) = **205ms**

### 4. **Controller Runs**

Nav2's controller (the thing that calculates motor commands) runs at a fixed frequency:
- Default in many tutorials: 20 Hz (every 50ms)
- More aggressive: 50 Hz (every 20ms)
- If an obstacle shows up right after the controller ran, you wait for the next cycle

**Running total:** 205ms + 50ms (controller wait) = **255ms**

### 5. **Computer Processes Everything**

This is where things get interesting and platform-dependent:
- **Fast desktop (AMD Ryzen 7900X):** 10-20ms to compute control commands
- **Raspberry Pi 4:** 40-80ms (the CPU is working hard!)
- **Raspberry Pi 5:** 20-40ms (better, but still significant)

Let's use a Pi 4 for this example: **+60ms**

**Running total:** 255ms + 60ms = **315ms**

### 6. **Linux Scheduler Jitter** (The Hidden Killer)

Here's something most tutorials don't mention: Linux is not a real-time operating system. When your controller node should run at exactly 20 Hz, Linux might decide to:
- Swap log files (5-50ms delay)
- Handle network interrupts
- Service other processes
- Perform memory management

On a Raspberry Pi 4 running standard Ubuntu, you can see scheduling delays of:
- Typical: 5-20ms
- Occasional spikes: 50-100ms
- During disk operations: up to 200ms!

Let's add a conservative **20ms** for typical jitter.

**Running total:** 315ms + 20ms = **335ms**

### 7. **Motor Commands Reach the Motors**

- Serial communication to motor controller: 1-5ms
- Motor controller processes command: 5-10ms
- PWM signal changes: <1ms
- Motor actually responds (overcome inertia): 10-50ms

For a good motor controller like a RoboClaw: **~20ms total**  
For a basic H-bridge: **~50ms total**

Let's use 30ms as middle ground.

**Running total:** 335ms + 30ms = **365ms**

### 8. **The Robot Actually Stops**

Finally, your robot has to physically decelerate. This depends on:
- Robot mass
- Wheel traction
- Motor torque
- Deceleration limit in your config

For a ~22kg (48 lb) robot decelerating at 1.0 m/s² (a conservative rate):
- From 0.5 m/s: **0.5 seconds** to stop
- From 1.0 m/s: **1.0 seconds** to stop

---

## The Math: How Fast Can You Go?

Now we can do the actual calculation. The formula is:

```
Safe Velocity (V_safe) = Clearance Distance / (Reaction Time + Stopping Time + Safety Margin)
```

Let's work through an example:

### Example 1: Raspberry Pi 4 Based Robot

**Reaction time breakdown:**
- Sensor latency: 100ms
- Costmap lag: 100ms
- Controller wait: 50ms
- Computation: 60ms
- Linux jitter: 20ms
- Motor response: 30ms
- **Total reaction time: 360ms = 0.36 seconds**

**Stopping characteristics:**
- Deceleration: 1.0 m/s²
- At 0.5 m/s: stops in 0.5 seconds
- Stopping distance: 0.125 meters

**Safety margin:** Add 50% buffer for unexpected situations
- 0.5 seconds additional

**Total time budget:**
```
T_total = 0.36s (reaction) + 0.5s (stopping) + 0.5s (safety) = 1.36 seconds
```

**If you need 1.0 meter clearance:**
```
V_safe = 1.0m / 1.36s = 0.735 m/s ≈ 1.6 mph
```

But wait! On a Pi 4, you also need to account for:
- Occasional 50-100ms jitter spikes
- CPU throttling when hot
- Multiple ROS2 nodes competing for CPU

**Realistic safe speed for Pi 4: 0.4-0.5 m/s (0.9-1.1 mph)**

### Example 2: High-Performance Desktop (Sigyn's AMD Ryzen 7900X)

**Reaction time breakdown:**
- Sensor latency: 100ms (same LIDAR)
- Costmap lag: 67ms (15 Hz updates)
- Controller wait: 50ms (20 Hz)
- Computation: 15ms (fast CPU)
- Linux jitter: 5ms (beefy CPU, less contention)
- Motor response: 20ms (RoboClaw)
- **Total reaction time: 257ms = 0.26 seconds**

**With same stopping and safety margins:**
```
T_total = 0.26s + 0.5s + 0.5s = 1.26 seconds
V_safe = 1.0m / 1.26s = 0.79 m/s ≈ 1.8 mph
```

**Realistic safe speed for high-end system: 0.6-0.8 m/s (1.3-1.8 mph)**

---

## Real-World Example: Sigyn's Configuration

My robot Sigyn is configured with these actual parameters (from `navigation_sim.yaml`):

```yaml
controller_frequency: 20.0  # Hz
vx_max: 0.5                 # m/s (1.1 mph)
ax_max: 1.0                 # m/s² deceleration
robot_radius: 0.28          # m
inflation_radius: 0.15      # m (safety buffer around obstacles)
```

Running on an AMD Ryzen 7900X with three Teensy 4.1 microcontrollers handling sensors, this gives me:
- Reaction time: ~230ms
- 1.0m obstacle clearance
- Safe velocity: **0.5 m/s configured** (conservative, could go up to ~0.7 m/s)

The conservative limit works well for navigating narrow doorways (26 inches / 66cm minimum in my house).

---

## The Factors That Actually Matter (Ranked by Impact)

Based on the analysis above, here's what **really** limits your speed, in order:

### 1. **Stopping Distance/Time** ⭐⭐⭐⭐⭐ (DOMINATES EVERYTHING)

This is the big one. Physics doesn't negotiate:
```
Stopping distance = (velocity²) / (2 × deceleration)

At 0.5 m/s with 1.0 m/s² deceleration: 0.125m
At 1.0 m/s with 1.0 m/s² deceleration: 0.5m
At 2.0 m/s with 1.0 m/s² deceleration: 2.0m!
```

Speed increases linearly, but stopping distance increases **quadratically**. Double your speed, quadruple your stopping distance!

**What you can do:**
- Higher deceleration (needs better motors/grip)
- Larger safety margins (more conservative navigation)
- Better prediction (advanced planners)

### 2. **Sensor Update Rate** ⭐⭐⭐⭐

Your LIDAR spinning at 10 Hz creates a fundamental 100ms delay. This is **not negotiable** with that sensor.

**Distance traveled during one LIDAR rotation:**
- At 0.5 m/s: 5cm (2 inches)
- At 1.0 m/s: 10cm (4 inches)
- At 2.0 m/s: 20cm (8 inches)

**What you can do:**
- Use faster LIDAR (some spin at 20-30 Hz, but cost more)
- Add complementary sensors (cameras, sonar, time-of-flight)
- Use sensor fusion to predict between scans

### 3. **Control Loop Frequency** ⭐⭐⭐⭐

Running your controller at 20 Hz means 50ms between reactions. At 50 Hz it's 20ms.

```yaml
# In your navigation config
controller_frequency: 20.0  # vs 50.0 makes a difference!
```

**Distance traveled during one control cycle:**
- At 20 Hz, going 0.5 m/s: 2.5cm
- At 50 Hz, going 0.5 m/s: 1.0cm

**What you can do:**
- Increase controller frequency (if your CPU can handle it)
- Use predictive controllers (MPPI, DWB)
- Tune costmap update frequency to match

### 4. **CPU Computation Time** ⭐⭐⭐ (Platform-Dependent!)

This is where Raspberry Pi vs. desktop makes a **massive** difference:

| Platform | Nav2 Cycle Time | Impact at 0.5 m/s |
|----------|----------------|-------------------|
| Raspberry Pi 4 | 40-80ms | 2-4cm traveled |
| Raspberry Pi 5 | 20-40ms | 1-2cm traveled |
| AMD Ryzen 7900X | 10-20ms | 0.5-1cm traveled |

**What you can do:**
- Use a more powerful computer
- Offload sensor processing to microcontrollers (like Sigyn's Teensy boards)
- Disable unnecessary ROS2 nodes
- Optimize costmap resolution and size
- Use simpler controllers

### 5. **Linux Scheduler Jitter** ⭐⭐⭐ (The Hidden Problem)

Standard Linux (Ubuntu, Raspbian) is **not real-time**. Your 20 Hz control loop might run at:
- 20.0 Hz most of the time
- 18 Hz occasionally (missed a cycle)
- 10 Hz during disk operations
- Completely frozen for 100-200ms during swapping

**On Raspberry Pi 4, I've measured:**
- Typical jitter: 5-20ms
- 95th percentile: 50ms
- Worst case: 100-200ms

**This is why you can't just rely on your average loop time!**

**What you can do:**
- Install PREEMPT_RT kernel patch (reduces jitter to <1ms)
- Use microcontrollers for time-critical tasks
- Increase process priority (`nice -20`)
- Minimize background tasks
- Add more safety margin to account for jitter

### 6. **Communication Latency** ⭐

Modern USB and serial links are fast enough that this is rarely the bottleneck:
- USB 2.0 (480 Mbps): <1ms for typical sensor data
- Serial at 230400 baud: <1ms for motor commands
- ROS2 message passing: 1-3ms typically

**What you can do:**
- Not much needed; this is rarely a problem
- Use USB 3.0 for high-bandwidth sensors (cameras)
- Optimize ROS2 QoS settings if seeing issues

### 7. **Motor Response Time** ⭐⭐

Modern motor controllers respond quickly:
- RoboClaw with velocity PID: 10-20ms to change speed
- Simple H-bridge: 30-100ms
- Cheap H-bridge without feedback: unpredictable!

**What you can do:**
- Use quality motor controllers with velocity PID
- Tune PID parameters
- Ensure encoders are working properly

### 8. **Wheel Slip/Traction** ⭐ (Usually Not a Concern)

At typical indoor robot speeds (<1 m/s), wheel slip is rarely the limiting factor:

**Maximum deceleration before slip:**
```
a_max = μ × g
where μ = coefficient of friction (0.7-0.8 for rubber on smooth floors)
      g = 9.81 m/s²

a_max ≈ 7 m/s² (WAY higher than typical 1.0 m/s² config)
```

Traction only becomes a concern if:
- Going >1.5 m/s
- Operating on slippery floors
- Making sharp turns at speed
- Emergency stops

**What you can do:**
- Use wider tires with good tread
- Reduce speed on slippery surfaces
- Add weight to increase normal force (but increases momentum!)

---

## Microcontrollers: The Secret Weapon

Here's something most tutorials don't tell you: **offloading sensors and motors to microcontrollers can dramatically improve your safe speed.**

### The Problem with "Linux Does Everything"

When your Raspberry Pi has to:
- Read LIDAR data
- Process 8 sonar sensors
- Read 2 IMUs
- Communicate with motor controllers
- Update costmaps
- Run navigation stack
- Handle ROS2 message passing
- Deal with logging, networking, and OS tasks

...things get **slow and unpredictable**.

### The Microcontroller Solution (Sigyn's Architecture)

Sigyn uses three Teensy 4.1 microcontrollers:

**Board 1 (Navigation/Safety):**
- Reads motor encoders at 70 Hz
- Computes odometry
- Monitors motor safety
- Handles emergency stop
- Publishes to ROS2 via USB
- **Guaranteed timing: <15ms from wheel movement to odometry message**

**Board 2 (Power/Sensors):**
- Monitors battery voltage and current
- Reads temperature sensors
- Manages 8× VL53L0X range sensors
- Reads 2× BNO055 IMUs
- All with deterministic timing

**Board 3 (Elevator/Gripper):**
- Controls stepper motors
- Handles limit switches
- Manages gripper actuators

**Why this matters:**

| Metric | Pi 4 Direct | Pi 4 + Teensy | AMD + Teensy |
|--------|-------------|---------------|--------------|
| Odometry rate | 10-20 Hz | 70 Hz | 70 Hz |
| Sensor jitter | 10-50ms | <1ms | <1ms |
| CPU load | 80-100% | 40-60% | 10-20% |
| Worst-case latency | 200ms+ | 100ms | 80ms |
| Safe velocity | 0.3 m/s | 0.5 m/s | 0.7 m/s |

The microcontrollers handle time-critical tasks with **deterministic, guaranteed timing**, while the main computer focuses on high-level planning.

---

## Platform Comparison: What Speed Can You Actually Achieve?

Let's compare three realistic scenarios:

### Scenario 1: Budget DIY (Pi 4 + Basic H-Bridge)

**Hardware:**
- Raspberry Pi 4 (4GB)
- L298N or similar H-bridge
- Encoders connected directly to Pi GPIO
- LD06 LIDAR via USB
- Total cost: ~$200

**Performance characteristics:**
- Odometry: 10-15 Hz
- CPU load: 80-95% during navigation
- Scheduler jitter: 20-50ms typical, 100ms spikes
- Motor response: 50-100ms

**Latency budget:**
```
Sensor: 100ms
Costmap: 100ms (10 Hz updates)
Controller: 100ms (10 Hz)
Computation: 80ms
Jitter: 40ms
Motor: 60ms
Total: 480ms

With 0.5s stop time, 1.0m clearance:
V_safe = 1.0 / (0.48 + 0.5 + 0.5) ≈ 0.3 m/s (0.67 mph)
```

**Realistic max speed: 0.25-0.35 m/s**

### Scenario 2: iRobot Create 3 Base

**Hardware:**
- Create 3 robot base (~$330)
- Built-in Raspberry Pi 4-equivalent
- Built-in sensors and encoders
- Well-tuned firmware

**Performance characteristics:**
- Odometry: 30-60 Hz (firmware-optimized)
- Integrated motor control
- Lower latency than DIY Pi 4

**Latency budget:**
```
Sensor: 100ms
Internal: 150ms (combined costmap/controller/motors)
Total: 250ms

V_safe = 1.0 / (0.25 + 0.5 + 0.5) ≈ 0.5 m/s (1.1 mph)
```

**Realistic max speed: 0.4-0.5 m/s**

### Scenario 3: High-End (Desktop + Microcontrollers)

**Hardware:**
- AMD Ryzen 7900X or similar (12+ cores)
- Teensy 4.1 or STM32 for motor control
- Additional microcontrollers for sensors
- Quality motor controller (RoboClaw)
- Total cost: ~$1500-2000

**Performance characteristics:**
- Odometry: 60-70 Hz (microcontroller)
- CPU load: <20% during navigation
- Minimal jitter: 1-5ms
- Motor response: 10-20ms

**Latency budget:**
```
Sensor: 100ms
Costmap: 67ms (15 Hz)
Controller: 50ms (20 Hz)
Computation: 15ms
Jitter: 5ms
Motor: 15ms
Total: 252ms

V_safe = 1.0 / (0.25 + 0.5 + 0.5) ≈ 0.8 m/s (1.8 mph)
```

**Realistic max speed: 0.6-0.8 m/s**

### The Cost-Performance Curve

| Investment | Platform | Safe Speed | Speed/Dollar |
|------------|----------|------------|--------------|
| $200 | DIY Pi 4 | 0.3 m/s | 0.0015 m/s/$ |
| $330 | Create 3 | 0.5 m/s | 0.0015 m/s/$ |
| $1500 | Desktop + MCU | 0.7 m/s | 0.0005 m/s/$ |

**Key insight:** There's a point of diminishing returns. Going from $200 to $330 gets you 67% more speed. Going from $330 to $1500 only gets you 40% more speed.

---

## Practical Configuration Examples

### Conservative (Beginner-Friendly)

For your first robot or one operating in cluttered spaces:

```yaml
# navigation.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.3              # 0.67 mph
      min_vel_x: -0.1             # Minimal reverse
      max_vel_theta: 0.8          # rad/s
      acc_lim_x: 0.5              # Smooth acceleration
      acc_lim_theta: 1.0
      
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0      # Match realistic Pi 4 capability
      width: 3
      height: 3
      resolution: 0.05            # 5cm cells
      robot_radius: 0.25          # 10" radius + buffer
      inflation_radius: 0.3       # 1 foot safety zone
```

**Expected performance:** Safe indoor navigation, minimal risk of crashes, good for beginners.

### Moderate (Experienced Users)

For robots with decent hardware and tuned systems:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5              # 1.1 mph
      min_vel_x: -0.2
      max_vel_theta: 1.2
      acc_lim_x: 0.8
      acc_lim_theta: 1.5
      
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 15.0
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      inflation_radius: 0.25
```

**Expected performance:** Good speed/safety balance, requires careful testing.

### Aggressive (High-Performance Systems)

For robots like Sigyn with powerful computers and microcontroller offload:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"  # Predictive controller
      max_vel_x: 0.7              # 1.6 mph
      min_vel_x: -0.3
      max_vel_theta: 1.5
      acc_lim_x: 1.0
      acc_lim_theta: 2.0
      time_steps: 56              # Longer prediction horizon
      
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 15.0
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.20
      inflation_radius: 0.20
```

**Expected performance:** Fast navigation, requires extensive testing and safety systems.

---

## How to Find YOUR Safe Speed

Here's a step-by-step process:

### Step 1: Measure Your Latencies

Create a simple test to measure actual timing:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class LatencyTester(Node):
    def __init__(self):
        super().__init__('latency_tester')
        
        self.scan_times = []
        self.cmd_times = []
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_scan_time = None
        
    def scan_callback(self, msg):
        now = time.time()
        if self.last_scan_time:
            dt = now - self.last_scan_time
            self.scan_times.append(dt)
            if len(self.scan_times) % 10 == 0:
                avg = sum(self.scan_times[-10:]) / 10
                self.get_logger().info(f'Avg scan interval: {avg*1000:.1f}ms')
        self.last_scan_time = now
        
    def timer_callback(self):
        # Measure time to publish
        start = time.time()
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_pub.publish(twist)
        dt = time.time() - start
        self.cmd_times.append(dt)
        
        if len(self.cmd_times) % 10 == 0:
            avg = sum(self.cmd_times[-10:]) / 10
            self.get_logger().info(f'Avg cmd_vel publish: {avg*1000:.1f}ms')

def main():
    rclpy.init()
    node = LatencyTester()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run this for 30 seconds and note:
- Average LIDAR scan interval
- Maximum LIDAR scan interval
- CPU usage during operation

### Step 2: Test Stopping Distance

Mark a line on the floor. Drive your robot toward it and send a stop command. Measure where it actually stops. Repeat 10 times at different speeds:

```python
# Simple stopping test
def test_stopping_distance():
    # Accelerate to target speed
    for i in range(20):
        cmd.linear.x = 0.3  # Target: 0.3 m/s
        pub.publish(cmd)
        time.sleep(0.1)
    
    # Mark starting position (manually or with camera)
    start_position = get_robot_pose()
    
    # Emergency stop
    cmd.linear.x = 0.0
    pub.publish(cmd)
    
    # Wait for complete stop
    time.sleep(2.0)
    
    end_position = get_robot_pose()
    distance = calculate_distance(start_position, end_position)
    print(f"Stopped in {distance:.3f} meters")
```

### Step 3: Calculate Your Latency Budget

Add up your measured values:

```
My Measured Latencies:
- LIDAR scan interval: _____ ms
- Costmap update (1/frequency): _____ ms  
- Controller cycle (1/frequency): _____ ms
- Observed computation time: _____ ms
- Worst-case jitter observed: _____ ms
- Motor command to motion: _____ ms

Total Latency: _____ ms = _____ seconds
```

### Step 4: Factor in Stopping

From your stopping tests:
```
At 0.3 m/s, stopped in: _____ m, time: _____ s
At 0.5 m/s, stopped in: _____ m, time: _____ s

Calculate deceleration:
a = v² / (2 × d)
a = (0.3)² / (2 × _____) = _____ m/s²
```

### Step 5: Calculate Safe Velocity

```
V_safe = Clearance / (Latency + Stop_Time + Safety_Margin)

With 1.0m clearance:
V_safe = 1.0 / (_____ + _____ + 0.5)
V_safe = _____ m/s = _____ mph
```

### Step 6: Test Conservatively

Set your `max_vel_x` to 70% of your calculated safe velocity initially:

```yaml
max_vel_x: 0.7 × V_safe = _____
```

Then gradually increase while testing in a safe environment.

---

## Advanced Techniques to Go Faster

Once you've optimized the basics, here are advanced approaches:

### 1. Install PREEMPT_RT Kernel

This dramatically reduces Linux scheduler jitter:

```bash
# On Ubuntu for Raspberry Pi
sudo apt install linux-image-rt-generic
# Reboot and verify
uname -a  # Should show "PREEMPT RT"
```

**Expected improvement:** Jitter from 20-50ms → 1-5ms

### 2. Use Predictive Controllers

MPPI (Model Predictive Path Integral) controller predicts future states:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56          # Look ahead 2.8 seconds
      model_dt: 0.05          # 50ms steps
      vx_std: 0.3             # Exploration noise
      temperature: 0.3        # Lower = more aggressive
```

**Expected improvement:** Can handle 20-30% higher speeds with same safety margin.

### 3. Add Complementary Sensors

LIDAR alone updates slowly. Add faster sensors:

- **Depth cameras (30 Hz):** Intel RealSense, OAK-D
- **ToF sensors (50+ Hz):** VL53L0X arrays
- **Bump sensors (instant):** Mechanical switches

Fuse them in your costmap:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      observation_sources: scan depth tof
      
      scan:  # LIDAR - authoritative
        topic: /scan
        data_type: LaserScan
        
      depth:  # Camera - supplement
        topic: /camera/depth/points
        data_type: PointCloud2
        
      tof:  # Close-range - override
        topic: /tof_points
        data_type: PointCloud2
```

### 4. Offload to Microcontrollers

Move time-critical tasks to dedicated hardware:

**What to offload:**
- Encoder reading → 70+ Hz odometry
- IMU fusion → 100 Hz orientation
- Motor control → <10ms response
- Sonar reading → Deterministic timing
- Emergency stop → Hardware-level safety

**Example: Teensy 4.1 Setup**

```cpp
// On Teensy 4.1
#include <micro_ros_arduino.h>

void setup() {
  // Configure encoders for high-speed reading
  attachInterrupt(ENCODER_A_PIN, encoder_isr, CHANGE);
  
  // Initialize micro-ROS
  set_microros_transports();
  
  // Create publishers
  rcl_publisher_init(&odom_publisher, ...);
}

void loop() {
  // Read encoders at maximum rate
  update_odometry();  // ~70 Hz
  
  // Publish via micro-ROS
  rcl_publish(&odom_publisher, &odom_msg, NULL);
  
  // Tight loop ensures consistent timing
}
```

### 5. Tune Your Controller Aggressively

Once hardware is solid, tune software:

```yaml
# For DWB controller
sim_time: 2.0                   # Longer prediction
vx_samples: 20                  # More velocity options
vy_samples: 5
vtheta_samples: 20

# Prefer forward motion
path_distance_bias: 32.0
goal_distance_bias: 24.0
occdist_scale: 0.1              # Less obstacle aversion

# For MPPI controller
batch_size: 2000                # More trajectories
iteration_count: 3              # Multiple iterations
critics:
  - ConstraintCritic
  - ObstaclesCritic
  - GoalCritic
  - PreferForwardCritic          # New in ROS2 Humble+
```

---

## Common Pitfalls and Solutions

### Pitfall 1: "My motors can go 5 m/s, why limit to 0.5 m/s?"

**Reality:** Your motors might achieve 5 m/s, but your sensors and computer can't keep up. Going faster than your reaction time allows = guaranteed crashes.

**Solution:** Do the latency budget math shown in this article.

### Pitfall 2: "It works fine in simulation at 2 m/s!"

**Reality:** Simulation has perfect sensors (no noise, infinite update rate) and perfect computation (no CPU limits, no jitter).

**Solution:** Test on real hardware. Use the simulator to develop algorithms, but **never trust simulation speeds** for the real robot.

### Pitfall 3: "I set max_vel_x to 0.5 but it only goes 0.3 m/s"

**Reality:** Many other parameters limit speed:
- Costmap inflation radius
- Controller trajectory scoring
- Acceleration limits
- Obstacle proximity

**Solution:** Check logs for which component is limiting speed:

```bash
ros2 topic echo /local_costmap/costmap --once
ros2 param get /controller_server max_vel_x
```

### Pitfall 4: "It crashes occasionally at 0.5 m/s but usually works"

**Reality:** "Usually works" isn't safe enough. Those occasions when it doesn't work are precisely when obstacles appear in worst-case timing.

**Solution:** Your safe speed is what works in the **worst case**, not the average case. Test extensively with obstacles appearing at unexpected times.

### Pitfall 5: "I added a faster CPU but speed didn't increase"

**Reality:** If sensor latency dominates (that 100ms LIDAR), CPU speed doesn't help much.

**Solution:** Prioritize sensor upgrade (faster LIDAR or add complementary sensors) before CPU upgrade.

---

## Measuring Success: Key Metrics

Track these metrics to know if your tuning is working:

### Safety Metrics (Most Important)

```
Crashes per Hour: _____ (Target: 0.0)
Near-misses per Hour: _____ (Target: <0.5)
Average Clearance to Obstacles: _____ (Target: >0.5m)
Minimum Clearance Observed: _____ (Target: >0.2m)
```

### Performance Metrics

```
Average Velocity: _____ m/s
Time to Complete Standard Route: _____ seconds
CPU Usage Average: _____ %
CPU Usage Peak: _____ %
Controller Frequency Actual: _____ Hz (vs configured)
Costmap Update Frequency Actual: _____ Hz (vs configured)
```

### Diagnostic Metrics

```
Worst-Case Sensor Latency: _____ ms
Worst-Case Scheduler Jitter: _____ ms
Odometry Frequency: _____ Hz
Navigation Warnings per Hour: _____
Recovery Behaviors Triggered per Hour: _____
```

Use ROS2 tools to monitor:

```bash
# Check actual frequencies
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 topic hz /odom

# Monitor CPU
htop

# Check for warnings
ros2 topic echo /diagnostics

# Log metrics
ros2 bag record -o metrics_run1 /scan /cmd_vel /odom /diagnostics
```

---

## Case Study: Sigyn's Journey to 0.5 m/s

Let me share how I arrived at Sigyn's configuration through real testing:

### Initial Attempt (Failure)

**Configuration:**
- Raspberry Pi 4 handling everything
- Generic motor controller
- max_vel_x: 0.8 m/s (optimistic!)

**Results:**
- Frequent crashes into chair legs
- Oscillatory motion in doorways
- CPU at 100%, frequent lag spikes
- **Failed:** Unsafe to operate

### Second Attempt (Better)

**Configuration:**
- Added AMD Ryzen 7900X computer
- RoboClaw motor controller
- max_vel_x: 0.6 m/s

**Results:**
- Fewer crashes but still occasional
- Better motion but jerky
- CPU comfortable at 30%
- **Partial success:** Safer but not reliable

### Final Configuration (Success)

**Configuration:**
- AMD Ryzen 7900X
- 3× Teensy 4.1 microcontrollers
- RoboClaw with PID tuning
- max_vel_x: 0.5 m/s
- MPPI controller with careful tuning
- Dual IMUs, 8× ToF sensors, LIDAR

**Results:**
- Zero crashes in 50+ hours of testing
- Smooth motion through narrow doorways (26" / 66cm)
- Consistent 20 Hz control loop
- CPU at 15-20%
- **Success:** Reliable autonomous navigation

**Key lessons:**
1. Microcontrollers for deterministic timing were game-changing
2. Conservative velocity with good sensors beats aggressive velocity
3. 0.5 m/s feels slow to humans but is plenty fast for a home robot
4. Extra CPU headroom eliminates jitter problems

---

## Conclusion: The Speed You Need vs. The Speed You Want

Here's the uncomfortable truth: **Most hobby mobile robots should operate at 0.3-0.5 m/s (0.7-1.1 mph).** This feels painfully slow when you're watching it, especially after investing hundreds of hours building it. But it's the right speed for safety given typical hobbyist hardware.

### The Speed Hierarchy

```
🐌 0.2-0.3 m/s: Budget DIY robots (Pi 4 + basic hardware)
   - Safe for cluttered indoor environments
   - Allows cheap hardware to keep up
   
🚶 0.3-0.5 m/s: Typical hobby robots (Create 3, mid-range builds)
   - Good balance of speed and safety
   - Human walking pace feels natural
   
🏃 0.5-0.7 m/s: High-performance hobby robots (good hardware + tuning)
   - Requires dedicated microcontrollers
   - Needs careful testing and tuning
   
🚗 0.7-1.0 m/s: Research/commercial grade (multiple computers, pro sensors)
   - Requires significant investment
   - Professional-grade safety systems needed
   
⚡ >1.0 m/s: Specialized applications only
   - Outdoor robots with long sight lines
   - Warehouse robots in controlled environments
   - Requires predictive planning and redundant safety
```

### The Right Question

Instead of "How fast can I go?", ask:

**"What speed gives me the best combination of:**
- **Safe operation** (no crashes)
- **Useful functionality** (completes tasks in reasonable time)
- **Reliable performance** (works consistently, not just sometimes)
- **Affordable hardware** (within my budget)"

For most home robots, that answer is **0.4-0.5 m/s**, which happens to be a comfortable human walking pace. Your robot will:
- Navigate safely through doorways
- Avoid pets and furniture reliably
- Complete household tasks in reasonable time
- Operate predictably for years

### Final Thoughts

Building a mobile robot is an exercise in managing tradeoffs. Speed is exciting, but **safety and reliability are what make a robot actually useful.** 

Start conservative. Measure everything. Tune gradually. Add hardware selectively. Test extensively.

And remember: a robot that operates reliably at 0.4 m/s is infinitely more valuable than one that theoretically can go 1.0 m/s but crashes into things.

---

## Further Reading

**ROS2 Navigation Documentation:**
- [Nav2 Controller Tuning Guide](https://navigation.ros.org/tuning/index.html)
- [MPPI Controller Configuration](https://navigation.ros.org/configuration/packages/configuring-mppic.html)
- [DWB Controller Configuration](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)

**Real-Time Linux:**
- [PREEMPT_RT Patch](https://wiki.linuxfoundation.org/realtime/start)
- [ROS2 Real-Time Working Group](https://ros-realtime.github.io/)

**Sigyn Project Resources:**
- [WhySigynMovesSoSmoothly.md](WhySigynMovesSoSmoothly.md) - Technical deep dive
- [TeensyV2 Architecture](../../TeensyV2/docs/ARCHITECTURE.md) - Microcontroller design

---

**Document Version:** 1.0  
**Last Updated:** February 3, 2026  
**Author:** Michael Wimble  
**License:** CC BY-SA 4.0
