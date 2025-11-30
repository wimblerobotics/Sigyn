#!/usr/bin/env python3
"""
Robot Monitor Script

Displays real-time information about the robot's state including:
- Wheel positions and velocities
- Arm joint position
- Temperature sensor reading
- Odometry data

Usage:
    ros2 run r2c_tutorial monitor_robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Temperature
from nav_msgs.msg import Odometry
import sys
import os


class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        
        # Storage for latest data
        self.joint_states = None
        self.temperature = None
        self.odometry = None
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        self.temp_sub = self.create_subscription(
            Temperature,
            '/temperature_sensor/raw',
            self.temp_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )
        
        # Timer for display update
        self.create_timer(0.5, self.display_status)
        
        self.get_logger().info('Robot monitor started')
    
    def joint_callback(self, msg):
        """Store joint state data"""
        self.joint_states = msg
    
    def temp_callback(self, msg):
        """Store temperature data"""
        self.temperature = msg
    
    def odom_callback(self, msg):
        """Store odometry data"""
        self.odometry = msg
    
    def display_status(self):
        """Display current robot status"""
        # Clear screen (works on Unix-like systems)
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("=" * 70)
        print("R2C TUTORIAL ROBOT MONITOR")
        print("=" * 70)
        print()
        
        # Joint States
        print("JOINT STATES:")
        print("-" * 70)
        if self.joint_states:
            for i, name in enumerate(self.joint_states.name):
                pos = self.joint_states.position[i] if i < len(self.joint_states.position) else 0.0
                vel = self.joint_states.velocity[i] if i < len(self.joint_states.velocity) else 0.0
                print(f"  {name:20s}: pos={pos:8.3f} rad, vel={vel:8.3f} rad/s")
        else:
            print("  Waiting for joint state data...")
        print()
        
        # Temperature Sensor
        print("TEMPERATURE SENSOR:")
        print("-" * 70)
        if self.temperature:
            print(f"  Temperature: {self.temperature.temperature:.2f} °C")
        else:
            print("  Waiting for temperature data...")
        print()
        
        # Odometry
        print("ODOMETRY:")
        print("-" * 70)
        if self.odometry:
            pos = self.odometry.pose.pose.position
            ori = self.odometry.pose.pose.orientation
            lin = self.odometry.twist.twist.linear
            ang = self.odometry.twist.twist.angular
            
            print(f"  Position: x={pos.x:.3f} m, y={pos.y:.3f} m, z={pos.z:.3f} m")
            print(f"  Orientation: x={ori.x:.3f}, y={ori.y:.3f}, z={ori.z:.3f}, w={ori.w:.3f}")
            print(f"  Linear vel: x={lin.x:.3f} m/s, y={lin.y:.3f} m/s")
            print(f"  Angular vel: z={ang.z:.3f} rad/s")
        else:
            print("  Waiting for odometry data...")
        print()
        
        print("=" * 70)
        print("Press Ctrl+C to exit")


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
