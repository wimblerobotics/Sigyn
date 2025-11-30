#!/usr/bin/env python3
"""
Sensor Simulator for ros2_control

This node simulates sensor values and writes them to the ros2_control
hardware interface state values. It demonstrates how to integrate custom
sensors (temperature, voltage, current, distance) into ros2_control.

The sensors are accessed through the hardware interface's state interfaces:
- temperature_sensor/temperature
- voltage_sensor/voltage
- current_sensor/current
- vl53l0x_sensor/range
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListHardwareInterfaces
import math
import random


class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        
        # Simulation parameters
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('temperature_ambient', 25.0)  # °C
        self.declare_parameter('voltage_nominal', 12.0)  # V
        self.declare_parameter('current_base', 0.5)  # A
        self.declare_parameter('range_default', 0.5)  # m
        
        update_rate = self.get_parameter('update_rate').value
        self.temp_ambient = self.get_parameter('temperature_ambient').value
        self.voltage_nominal = self.get_parameter('voltage_nominal').value
        self.current_base = self.get_parameter('current_base').value
        self.range_default = self.get_parameter('range_default').value
        
        # Simulation state
        self.time = 0.0
        self.temperature = self.temp_ambient
        self.voltage = self.voltage_nominal
        self.current = self.current_base
        self.range_distance = self.range_default
        
        # Publishers for sensor data (standard ROS topics)
        from sensor_msgs.msg import Temperature, Range
        from std_msgs.msg import Float32
        
        self.temp_pub = self.create_publisher(Temperature, '/sensors/temperature', 10)
        self.voltage_pub = self.create_publisher(Float32, '/sensors/voltage', 10)
        self.current_pub = self.create_publisher(Float32, '/sensors/current', 10)
        self.range_pub = self.create_publisher(Range, '/sensors/range', 10)
        
        # Timer for sensor updates
        self.timer = self.create_timer(1.0 / update_rate, self.update_sensors)
        
        self.get_logger().info(f'Sensor simulator started at {update_rate} Hz')
        self.get_logger().info('Publishing sensor data to:')
        self.get_logger().info('  /sensors/temperature (sensor_msgs/Temperature)')
        self.get_logger().info('  /sensors/voltage (std_msgs/Float32)')
        self.get_logger().info('  /sensors/current (std_msgs/Float32)')
        self.get_logger().info('  /sensors/range (sensor_msgs/Range)')
    
    def update_sensors(self):
        """Update all sensor simulations"""
        dt = 0.1  # 10 Hz update rate
        self.time += dt
        
        # Temperature: Slow drift with noise
        # Simulates temperature rising when robot is active
        temp_target = self.temp_ambient + 5.0 * math.sin(self.time * 0.1)
        self.temperature += 0.1 * (temp_target - self.temperature)
        self.temperature += random.gauss(0, 0.2)  # Add noise
        
        # Voltage: Battery discharge simulation with noise
        # Slowly decreases over time, with load-dependent fluctuations
        discharge_rate = 0.001  # Very slow discharge
        load_variation = 0.05 * math.sin(self.time * 2.0)  # Load fluctuation
        self.voltage = self.voltage_nominal - (self.time * discharge_rate) + load_variation
        self.voltage += random.gauss(0, 0.05)
        self.voltage = max(10.0, min(14.0, self.voltage))  # Clamp to realistic range
        
        # Current: Load-dependent with motion
        # Higher current when motors are active
        base_current = self.current_base
        motor_current = 2.0 * abs(math.sin(self.time * 0.5))  # Simulate motor load
        self.current = base_current + motor_current
        self.current += random.gauss(0, 0.1)
        self.current = max(0.0, self.current)
        
        # Range: Simulate distance sensor seeing objects
        # Oscillates to simulate robot moving or objects in view
        self.range_distance = 0.5 + 0.3 * math.sin(self.time * 0.3)
        self.range_distance += random.gauss(0, 0.02)  # Sensor noise
        self.range_distance = max(0.03, min(2.0, self.range_distance))
        
        # Publish to ROS topics
        self.publish_temperature()
        self.publish_voltage()
        self.publish_current()
        self.publish_range()
    
    def publish_temperature(self):
        """Publish temperature as sensor_msgs/Temperature"""
        from sensor_msgs.msg import Temperature
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temperature_sensor'
        msg.temperature = self.temperature
        msg.variance = 0.04  # 0.2°C standard deviation
        self.temp_pub.publish(msg)
    
    def publish_voltage(self):
        """Publish voltage as Float32"""
        from std_msgs.msg import Float32
        msg = Float32()
        msg.data = self.voltage
        self.voltage_pub.publish(msg)
    
    def publish_current(self):
        """Publish current as Float32"""
        from std_msgs.msg import Float32
        msg = Float32()
        msg.data = self.current
        self.current_pub.publish(msg)
    
    def publish_range(self):
        """Publish range as sensor_msgs/Range"""
        from sensor_msgs.msg import Range
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temperature_sensor'  # VL53L0X on arm tip
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.44  # 25 degrees in radians
        msg.min_range = 0.03  # 30mm
        msg.max_range = 2.0   # 2 meters
        msg.range = self.range_distance
        self.range_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
