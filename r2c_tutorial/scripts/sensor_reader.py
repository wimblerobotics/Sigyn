#!/usr/bin/env python3
"""
Example: Read Sensor Data from ROS2 Control

This script demonstrates how to subscribe to sensor data published by
the sensor_state_broadcaster controller.

The sensor_state_broadcaster reads sensor state interfaces from the
hardware interface and publishes them as standard ROS messages:
  - ~/temperature -> sensor_msgs/msg/Temperature
  - ~/battery -> sensor_msgs/msg/BatteryState
  - ~/range -> sensor_msgs/msg/Range

Usage:
    ros2 run r2c_tutorial sensor_reader.py

Topics subscribed:
    /sensor_state_broadcaster/temperature (sensor_msgs/msg/Temperature)
    /sensor_state_broadcaster/battery (sensor_msgs/msg/BatteryState)
    /sensor_state_broadcaster/range (sensor_msgs/msg/Range)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, BatteryState, Range


class SensorReader(Node):
    """
    Example client that reads sensor data from ros2_control controllers.
    
    This demonstrates the complete data flow:
    1. Hardware Interface exports state interfaces
    2. Controller reads state interfaces
    3. Controller publishes to topics
    4. Client subscribes to topics
    """
    
    def __init__(self):
        super().__init__('sensor_reader')
        
        # Declare parameter to switch between simulation and real hardware topics
        # Default to False since we now use ros2_control in both modes
        self.declare_parameter('use_sim_topics', False)
        use_sim_topics = self.get_parameter('use_sim_topics').value
        
        # Choose topic prefix based on mode
        if use_sim_topics:
            # Legacy mode: direct topic publishing (deprecated)
            temp_topic = '/sensors/temperature'
            voltage_topic = '/sensors/voltage'
            current_topic = '/sensors/current'
            range_topic = '/sensors/range'
            self.get_logger().info('Using legacy simulation topics (/sensors/*)')
        else:
            # Standard mode: ros2_control topics (both sim and real hardware)
            temp_topic = '/sensor_state_broadcaster/temperature'
            voltage_topic = None  # Will use battery topic
            current_topic = None
            range_topic = '/sensor_state_broadcaster/range'
            self.get_logger().info('Using ros2_control topics (/sensor_state_broadcaster/*)')
        
        # Subscribe to sensor topics
        self.temp_sub = self.create_subscription(
            Temperature,
            temp_topic,
            self.temperature_callback,
            10
        )
        
        if use_sim_topics:
            # Legacy simulation: separate voltage/current topics
            from std_msgs.msg import Float32
            
            self.voltage_sub = self.create_subscription(
                Float32,
                voltage_topic,
                self.voltage_callback_sim,
                10
            )
            
            self.current_sub = self.create_subscription(
                Float32,
                current_topic,
                self.current_callback_sim,
                10
            )
        else:
            # Standard mode: combined battery topic
            self.battery_sub = self.create_subscription(
                BatteryState,
                '/sensor_state_broadcaster/battery',
                self.battery_callback,
                10
            )
        
        self.range_sub = self.create_subscription(
            Range,
            range_topic,
            self.range_callback,
            10
        )
        
        # Store latest sensor values
        self.temperature = None
        self.voltage = None
        self.current = None
        self.range = None
        
        # Create timer to print status
        self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('Sensor reader started - waiting for data...')
        if use_sim_topics:
            self.get_logger().info('Topics:')
            self.get_logger().info('  /sensors/temperature')
            self.get_logger().info('  /sensors/voltage')
            self.get_logger().info('  /sensors/current')
            self.get_logger().info('  /sensors/range')
        else:
            self.get_logger().info('Topics:')
            self.get_logger().info('  /sensor_state_broadcaster/temperature')
            self.get_logger().info('  /sensor_state_broadcaster/battery')
            self.get_logger().info('  /sensor_state_broadcaster/range')
    
    def temperature_callback(self, msg):
        """Handle temperature sensor data."""
        self.temperature = msg.temperature
        self.get_logger().debug(f'Temperature: {msg.temperature:.1f} °C')
    
    def voltage_callback_sim(self, msg):
        """Handle simulation voltage (Float32)."""
        self.voltage = msg.data
    
    def current_callback_sim(self, msg):
        """Handle simulation current (Float32)."""
        self.current = msg.data
    
    def battery_callback(self, msg):
        """Handle battery/power sensor data (voltage + current)."""
        self.voltage = msg.voltage
        self.current = msg.current
        self.get_logger().debug(
            f'Battery: {msg.voltage:.2f}V, {msg.current:.3f}A, '
            f'Power: {msg.voltage * msg.current:.2f}W'
        )
    
    def range_callback(self, msg):
        """Handle range sensor data (VL53L0X)."""
        self.range = msg.range
        self.get_logger().debug(f'Range: {msg.range:.3f} m')
    
    def print_status(self):
        """Print current sensor values periodically."""
        if self.temperature is None:
            self.get_logger().warn('No sensor data received yet...')
            return
        
        # Calculate power
        power = self.voltage * self.current if (self.voltage and self.current) else 0.0
        
        self.get_logger().info(
            f'Sensors: Temp={self.temperature:.1f}°C  '
            f'Voltage={self.voltage:.2f}V  '
            f'Current={self.current:.3f}A  '
            f'Power={power:.2f}W  '
            f'Range={self.range:.3f}m'
        )
        
        # Example: Check for safety limits (this is where you'd implement safety logic)
        if self.temperature > 80.0:
            self.get_logger().error('⚠️  TEMPERATURE TOO HIGH! Thermal protection needed!')
        
        if self.voltage < 10.0:
            self.get_logger().warn('⚠️  Low battery warning!')
        
        if self.current > 8.0:
            self.get_logger().warn('⚠️  High current draw!')
        
        if self.range < 0.1:
            self.get_logger().warn('⚠️  Obstacle detected nearby!')


def main(args=None):
    rclpy.init(args=args)
    node = SensorReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
