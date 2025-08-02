#!/usr/bin/env python3

"""
Demo script for Perimeter Roamer V2
This script shows how to use the perimeter roamer package programmatically.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time

class PerimeterRoamerDemo(Node):
    """Demo class to show how to interact with the perimeter roamer"""
    
    def __init__(self):
        super().__init__('perimeter_roamer_demo')
        
        # Subscribe to roamer topics
        self.state_sub = self.create_subscription(
            String,
            '/roamer_state',
            self.state_callback,
            10
        )
        
        self.space_type_sub = self.create_subscription(
            String,
            '/space_type',
            self.space_type_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Initialize state tracking
        self.current_state = "UNKNOWN"
        self.current_space_type = "UNKNOWN"
        self.last_cmd_vel = None
        
        # Create timer for status updates
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info("Perimeter Roamer Demo started")
        self.get_logger().info("Monitoring roamer state and space classification...")
    
    def state_callback(self, msg):
        """Callback for robot state changes"""
        self.current_state = msg.data
    
    def space_type_callback(self, msg):
        """Callback for space type changes"""
        self.current_space_type = msg.data
    
    def cmd_vel_callback(self, msg):
        """Callback for velocity commands"""
        self.last_cmd_vel = msg
    
    def print_status(self):
        """Print current status"""
        status_msg = f"State: {self.current_state}, Space: {self.current_space_type}"
        
        if self.last_cmd_vel:
            linear_speed = self.last_cmd_vel.linear.x
            angular_speed = self.last_cmd_vel.angular.z
            status_msg += f", Speed: {linear_speed:.2f} m/s, Turn: {angular_speed:.2f} rad/s"
        
        self.get_logger().info(status_msg)

def main(args=None):
    rclpy.init(args=args)
    demo = PerimeterRoamerDemo()
    
    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info("Demo interrupted by user")
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 