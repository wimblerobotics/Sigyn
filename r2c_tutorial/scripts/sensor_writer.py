#!/usr/bin/env python3
"""
Example: Write Sensor Commands (Simulation Mode Only)

This script demonstrates how to inject test values into sensor hardware interfaces
through the sensor_command_controller in simulation mode. This allows testing
safety systems by injecting fault conditions without damaging real hardware.

The sensor_command_controller subscribes to command topics and writes values
to the hardware interface command interfaces:
  - temperature_sensor/temperature
  - battery_sensor/voltage
  - battery_sensor/current
  - range_sensor/range

Use cases:
  - Test thermal protection by injecting high temperature
  - Test low battery protection by simulating voltage drop
  - Test overcurrent protection by simulating high current
  - Test collision avoidance by simulating close obstacles

Usage with ROS parameters:
    # Inject high temperature (test thermal shutdown)
    ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=85.0
    
    # Inject low voltage (test battery protection)
    ros2 run r2c_tutorial sensor_writer.py --ros-args -p voltage_cmd:=9.5
    
    # Inject high current (test overcurrent protection)
    ros2 run r2c_tutorial sensor_writer.py --ros-args -p current_cmd:=9.0
    
    # Inject close obstacle (test collision avoidance)
    ros2 run r2c_tutorial sensor_writer.py --ros-args -p range_cmd:=0.05
    
    # Combined test scenario
    ros2 run r2c_tutorial sensor_writer.py --ros-args \\
        -p temperature_cmd:=85.0 \\
        -p voltage_cmd:=9.5 \\
        -p current_cmd:=8.5
    
    # Reset to simulation (send 0.0 to any command)
    ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=0.0

Alternative: Direct topic publishing:
    ros2 topic pub /sensor_command_controller/temperature_cmd std_msgs/msg/Float64 "{data: 93.0}" --once
    ros2 topic pub /sensor_command_controller/voltage_cmd std_msgs/msg/Float64 "{data: 9.5}" --once
    ros2 topic pub /sensor_command_controller/current_cmd std_msgs/msg/Float64 "{data: 10.0}" --once
    ros2 topic pub /sensor_command_controller/range_cmd std_msgs/msg/Float64 "{data: 0.05}" --once

Data Flow:
    1. This script publishes to /sensor_command_controller/*_cmd topics
    2. SensorCommandController subscribes to command topics
    3. Controller writes commands to hardware interface command interfaces
    4. Hardware interface read() applies commands to state interfaces
    5. SensorStateBroadcaster reads updated state interfaces
    6. Your safety system reacts to sensor values

Note: This ONLY works in simulation mode. In real hardware mode, sensor
values come from the Teensy and command interfaces are not exposed.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SensorCommandWriter(Node):
    """
    Example client that publishes sensor commands to test safety systems.
    
    This demonstrates testing safety systems in simulation by injecting
    fault conditions through the sensor_command_controller.
    
    Command value behavior:
    - Non-zero value: Hardware uses this fixed value (test mode)
    - Zero (0.0): Hardware returns to normal simulation (reset)
    """
    
    def __init__(self):
        super().__init__('sensor_command_writer')
        
        # Declare parameters for command values
        self.declare_parameter('temperature_cmd', 0.0)
        self.declare_parameter('voltage_cmd', 0.0)
        self.declare_parameter('current_cmd', 0.0)
        self.declare_parameter('range_cmd', 0.0)
        
        # Get parameter values
        temperature = self.get_parameter('temperature_cmd').value
        voltage = self.get_parameter('voltage_cmd').value
        current = self.get_parameter('current_cmd').value
        range_val = self.get_parameter('range_cmd').value
        
        # Create publishers
        self.temp_pub = self.create_publisher(
            Float64, '/sensor_command_controller/temperature_cmd', 10)
        self.voltage_pub = self.create_publisher(
            Float64, '/sensor_command_controller/voltage_cmd', 10)
        self.current_pub = self.create_publisher(
            Float64, '/sensor_command_controller/current_cmd', 10)
        self.range_pub = self.create_publisher(
            Float64, '/sensor_command_controller/range_cmd', 10)
        
        # Wait for subscribers
        self.get_logger().info('Waiting for sensor_command_controller...')
        import time
        time.sleep(0.5)
        
        self.get_logger().info('Sensor Command Writer (Simulation Mode Only)')
        self.get_logger().info('=' * 70)
        
        # Publish commands
        commands_sent = False
        
        if temperature != 0.0:
            msg = Float64()
            msg.data = temperature
            self.temp_pub.publish(msg)
            self.get_logger().info(f'✓ Temperature command: {temperature:.1f} °C')
            if temperature > 80.0:
                self.get_logger().warn('  ⚠️  HIGH TEMPERATURE - Testing thermal protection!')
            commands_sent = True
        
        if voltage != 0.0:
            msg = Float64()
            msg.data = voltage
            self.voltage_pub.publish(msg)
            self.get_logger().info(f'✓ Voltage command: {voltage:.2f} V')
            if voltage < 10.0:
                self.get_logger().warn('  ⚠️  LOW VOLTAGE - Testing battery protection!')
            commands_sent = True
        
        if current != 0.0:
            msg = Float64()
            msg.data = current
            self.current_pub.publish(msg)
            self.get_logger().info(f'✓ Current command: {current:.2f} A')
            if current > 8.0:
                self.get_logger().warn('  ⚠️  HIGH CURRENT - Testing overcurrent protection!')
            commands_sent = True
        
        if range_val != 0.0:
            msg = Float64()
            msg.data = range_val
            self.range_pub.publish(msg)
            self.get_logger().info(f'✓ Range command: {range_val:.3f} m')
            if range_val < 0.1:
                self.get_logger().warn('  ⚠️  CLOSE OBSTACLE - Testing collision avoidance!')
            commands_sent = True
        
        if not commands_sent:
            self.get_logger().info('No commands specified. Usage:')
            self.get_logger().info('  ros2 run r2c_tutorial sensor_writer.py --ros-args \\')
            self.get_logger().info('    -p temperature_cmd:=85.0 \\')
            self.get_logger().info('    -p voltage_cmd:=9.5 \\')
            self.get_logger().info('    -p current_cmd:=8.5 \\')
            self.get_logger().info('    -p range_cmd:=0.05')
            self.get_logger().info('')
            self.get_logger().info('Or use direct topic publishing:')
            self.get_logger().info('  ros2 topic pub /sensor_command_controller/temperature_cmd \\')
            self.get_logger().info('    std_msgs/msg/Float64 "{data: 93.0}" --once')
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('Commands published successfully!')
        self.get_logger().info('Monitor with: ros2 run r2c_tutorial sensor_reader.py')
        self.get_logger().info('')
        self.get_logger().info('To reset to normal simulation, send 0.0:')
        self.get_logger().info('  ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=0.0')


def main(args=None):
    rclpy.init(args=args)
    node = SensorCommandWriter()
    
    # Spin briefly to ensure messages are sent
    try:
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
