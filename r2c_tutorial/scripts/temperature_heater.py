#!/usr/bin/env python3
"""
Temperature Heater Script

This script simulates heating the temperature sensor by publishing
commands to a Gazebo topic or ros2_control interface.

Usage:
    ros2 run r2c_tutorial temperature_heater --temperature 50.0
    
Arguments:
    --temperature TEMP    Target temperature in Celsius (default: 50.0)
    --rate RATE          Publishing rate in Hz (default: 10.0)
    --ramp              Gradually ramp up temperature instead of step
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import argparse
import math


class TemperatureHeater(Node):
    def __init__(self, target_temp, rate, use_ramp):
        super().__init__('temperature_heater')
        
        self.target_temp = target_temp
        self.use_ramp = use_ramp
        self.current_temp = 25.0  # Room temperature starting point
        self.ramp_rate = 1.0  # Degrees per second
        
        # Publisher to temperature control topic
        # Note: This is a simulation helper - in real hardware, you'd
        # interface with actual heating elements through ros2_control
        self.temp_publisher = self.create_publisher(
            Float64,
            '/temperature_control/command',
            10
        )
        
        # Timer for publishing
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(
            f'Temperature heater started. Target: {target_temp}°C, '
            f'Ramp: {use_ramp}, Rate: {rate} Hz'
        )
    
    def timer_callback(self):
        """Publish temperature command"""
        msg = Float64()
        
        if self.use_ramp:
            # Gradually ramp to target
            if abs(self.current_temp - self.target_temp) > 0.1:
                step = self.ramp_rate * (1.0 / 10.0)  # Assuming 10 Hz
                if self.current_temp < self.target_temp:
                    self.current_temp = min(self.current_temp + step, self.target_temp)
                else:
                    self.current_temp = max(self.current_temp - step, self.target_temp)
            
            msg.data = self.current_temp
            self.get_logger().info(f'Ramping temperature: {self.current_temp:.1f}°C', 
                                   throttle_duration_sec=1.0)
        else:
            # Step to target immediately
            msg.data = self.target_temp
            self.get_logger().info(f'Setting temperature: {self.target_temp:.1f}°C',
                                   throttle_duration_sec=1.0)
        
        self.temp_publisher.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='Temperature sensor heater control')
    parser.add_argument('--temperature', type=float, default=50.0,
                      help='Target temperature in Celsius')
    parser.add_argument('--rate', type=float, default=10.0,
                      help='Publishing rate in Hz')
    parser.add_argument('--ramp', action='store_true',
                      help='Gradually ramp temperature instead of step change')
    
    parsed_args, remaining = parser.parse_known_args()
    
    rclpy.init(args=remaining)
    
    node = TemperatureHeater(
        target_temp=parsed_args.temperature,
        rate=parsed_args.rate,
        use_ramp=parsed_args.ramp
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
