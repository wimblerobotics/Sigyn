#!/usr/bin/env python3

"""
IMU Monitor for OAK-D Camera
Displays IMU data to help determine camera orientation for URDF adjustments.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuMonitor(Node):
    def __init__(self):
        super().__init__('imu_monitor')
        
        # Subscribe to OAK-D IMU topic
        self.imu_sub = self.create_subscription(
            Imu,
            '/oakd_top/imu/data',  # Adjust topic name if needed
            self.imu_callback,
            10
        )
        
        self.get_logger().info("IMU Monitor started. Listening for IMU data...")
        self.get_logger().info("Tilt the camera to see orientation changes.")
        
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)  
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def imu_callback(self, msg):
        # Extract quaternion from IMU message
        orientation = msg.orientation
        
        # Convert to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        # Extract angular velocity
        angular_vel = msg.angular_velocity
        
        # Extract linear acceleration  
        linear_accel = msg.linear_acceleration
        
        # Clear screen and display current data
        print("\033[2J\033[H")  # Clear screen and move cursor to top
        print("=" * 60)
        print("OAK-D Camera IMU Data Monitor")
        print("=" * 60)
        print(f"Frame ID: {msg.header.frame_id}")
        print()
        
        print("ORIENTATION (Degrees):")
        print(f"  Roll  (X): {roll:8.2f}°")
        print(f"  Pitch (Y): {pitch:8.2f}°") 
        print(f"  Yaw   (Z): {yaw:8.2f}°")
        print()
        
        print("ANGULAR VELOCITY (rad/s):")
        print(f"  X: {angular_vel.x:8.4f}")
        print(f"  Y: {angular_vel.y:8.4f}")
        print(f"  Z: {angular_vel.z:8.4f}")
        print()
        
        print("LINEAR ACCELERATION (m/s²):")
        print(f"  X: {linear_accel.x:8.4f}")
        print(f"  Y: {linear_accel.y:8.4f}")
        print(f"  Z: {linear_accel.z:8.4f}")
        print()
        
        print("URDF ORIENTATION GUIDE:")
        print("=" * 40)
        print("For camera mount in URDF, use these approximate values:")
        print(f"  <origin xyz=\"...\" rpy=\"{math.radians(roll):.4f} {math.radians(pitch):.4f} {math.radians(yaw):.4f}\"/>")
        print(f"  (Roll={roll:.1f}° Pitch={pitch:.1f}° Yaw={yaw:.1f}°)")
        print()
        print("Press Ctrl+C to exit")

def main(args=None):
    rclpy.init(args=args)
    
    imu_monitor = ImuMonitor()
    
    try:
        rclpy.spin(imu_monitor)
    except KeyboardInterrupt:
        print("\nIMU Monitor stopped.")
    
    imu_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()