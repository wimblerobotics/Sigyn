#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

"""
Diagnostic script to capture and analyze localization data.

This script:
1. Records critical topics during a forward + rotation maneuver
2. Saves data to a pickle file for repeated analysis
3. Analyzes the data to identify sign inversions and other issues

Expected behavior:
- Forward motion: wheel_odom.vx > 0, odom moves forward, amcl stays aligned
- CCW rotation: wheel_odom.wz > 0, odom rotates CCW, amcl stays aligned

If CCW rotation causes backward motion, angular velocity sign is inverted.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import pickle
import math
from collections import defaultdict
import numpy as np

class LocalizationDiagnostic(Node):
    def __init__(self):
        super().__init__('localization_diagnostic')
        
        # Data storage
        self.data = {
            'cmd_vel': [],
            'wheel_odom': [],
            'ekf_odom': [],
            'amcl_pose': [],
            'imu_sensor_0': [],
            'imu_sensor_1': [],
            'timestamps': []
        }
        
        self.recording = False
        self.start_time = None
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/sigyn/wheel_odom', self.wheel_odom_callback, 10)
        self.create_subscription(Odometry, '/odom', self.ekf_odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(Imu, '/sigyn/teensy_bridge/imu/sensor_0', self.imu0_callback, 10)
        self.create_subscription(Imu, '/sigyn/teensy_bridge/imu/sensor_1', self.imu1_callback, 10)
        
        # Publisher for commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Localization diagnostic node started')
    
    def cmd_vel_callback(self, msg):
        if self.recording:
            self.data['cmd_vel'].append({
                'time': time.time() - self.start_time,
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            })
    
    def wheel_odom_callback(self, msg):
        if self.recording:
            # Extract quaternion and convert to yaw
            qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
                             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            self.data['wheel_odom'].append({
                'time': time.time() - self.start_time,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': yaw,
                'vx': msg.twist.twist.linear.x,
                'vy': msg.twist.twist.linear.y,
                'wz': msg.twist.twist.angular.z
            })
    
    def ekf_odom_callback(self, msg):
        if self.recording:
            qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
                             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            self.data['ekf_odom'].append({
                'time': time.time() - self.start_time,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': yaw,
                'vx': msg.twist.twist.linear.x,
                'vy': msg.twist.twist.linear.y,
                'wz': msg.twist.twist.angular.z
            })
    
    def amcl_callback(self, msg):
        if self.recording:
            qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
                             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            self.data['amcl_pose'].append({
                'time': time.time() - self.start_time,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': yaw
            })
    
    def imu0_callback(self, msg):
        if self.recording:
            self.data['imu_sensor_0'].append({
                'time': time.time() - self.start_time,
                'gz': msg.angular_velocity.z
            })
    
    def imu1_callback(self, msg):
        if self.recording:
            self.data['imu_sensor_1'].append({
                'time': time.time() - self.start_time,
                'gz': msg.angular_velocity.z
            })
    
    def start_recording(self):
        self.recording = True
        self.start_time = time.time()
        self.get_logger().info('🔴 Recording started')
    
    def stop_recording(self):
        self.recording = False
        self.get_logger().info('⏹️  Recording stopped')
    
    def send_command(self, linear_x, angular_z, duration):
        """Send velocity command for specified duration"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        start = time.time()
        
        while time.time() - start < duration:
            self.cmd_pub.publish(msg)
            time.sleep(0.033)  # ~30 Hz
        
        # Stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        time.sleep(0.1)  # Ensure stop command is sent
    
    def run_test_sequence(self):
        """Execute the test maneuver"""
        self.get_logger().info('🚀 Starting test sequence in 3 seconds...')
        time.sleep(3)
        
        self.start_recording()
        
        # Phase 1: Move forward 0.5 meters
        # At 0.15 m/s, this takes about 3.3 seconds
        self.get_logger().info('📏 Phase 1: Moving forward 0.5m')
        self.send_command(0.15, 0.0, 3.5)
        time.sleep(1.0)  # Settling time
        
        # Phase 2: Rotate CCW 45 degrees (π/4 radians)
        # At 0.3 rad/s, this takes about 2.6 seconds
        self.get_logger().info('🔄 Phase 2: Rotating CCW 45 degrees')
        self.send_command(0.0, 0.3, 2.8)
        time.sleep(1.0)  # Settling time
        
        self.stop_recording()
        
        # Save data
        filename = f'localization_data_{int(time.time())}.pkl'
        with open(filename, 'wb') as f:
            pickle.dump(self.data, f)
        
        self.get_logger().info(f'💾 Data saved to {filename}')
        return filename


def analyze_data(filename):
    """Analyze recorded data for sign inversions and other issues"""
    with open(filename, 'rb') as f:
        data = pickle.load(f)
    
    print("\n" + "="*80)
    print("LOCALIZATION DIAGNOSTIC ANALYSIS")
    print("="*80)
    
    # Find forward motion phase (cmd_vel.x > 0)
    forward_phase = [d for d in data['cmd_vel'] if d['linear_x'] > 0.05]
    rotation_phase = [d for d in data['cmd_vel'] if abs(d['angular_z']) > 0.1]
    
    if forward_phase:
        forward_start = forward_phase[0]['time']
        forward_end = forward_phase[-1]['time']
        print(f"\n📏 FORWARD MOTION PHASE: {forward_start:.2f}s - {forward_end:.2f}s")
        
        # Get wheel odom during forward motion
        forward_odom = [d for d in data['wheel_odom'] 
                       if forward_start <= d['time'] <= forward_end]
        
        if forward_odom:
            avg_vx = np.mean([d['vx'] for d in forward_odom])
            avg_cmd = np.mean([d['linear_x'] for d in forward_phase])
            
            print(f"  Commanded vx: {avg_cmd:.3f} m/s")
            print(f"  Wheel odom vx: {avg_vx:.3f} m/s")
            
            if avg_vx * avg_cmd < 0:
                print("  ⚠️  SIGN INVERSION DETECTED! Linear velocity is inverted!")
            elif abs(avg_vx - avg_cmd) > 0.05:
                print(f"  ⚠️  Velocity mismatch: {abs(avg_vx - avg_cmd):.3f} m/s")
            else:
                print("  ✅ Linear velocity signs match")
            
            # Check position change
            delta_x = forward_odom[-1]['x'] - forward_odom[0]['x']
            delta_y = forward_odom[-1]['y'] - forward_odom[0]['y']
            distance = math.sqrt(delta_x**2 + delta_y**2)
            print(f"  Distance traveled: {distance:.3f} m (expected ~0.5m)")
    
    if rotation_phase:
        rot_start = rotation_phase[0]['time']
        rot_end = rotation_phase[-1]['time']
        print(f"\n🔄 ROTATION PHASE: {rot_start:.2f}s - {rot_end:.2f}s")
        
        # Get wheel odom during rotation
        rot_odom = [d for d in data['wheel_odom'] 
                   if rot_start <= d['time'] <= rot_end]
        
        if rot_odom:
            avg_wz = np.mean([d['wz'] for d in rot_odom])
            avg_cmd_wz = np.mean([d['angular_z'] for d in rotation_phase])
            
            print(f"  Commanded wz: {avg_cmd_wz:.3f} rad/s")
            print(f"  Wheel odom wz: {avg_wz:.3f} rad/s")
            
            if avg_wz * avg_cmd_wz < 0:
                print("  ⚠️  SIGN INVERSION DETECTED! Angular velocity is inverted!")
                print("  🔧 FIX: In roboclaw_monitor.cpp line 795, change to:")
                print("      float delta_theta = (dist_m1 - dist_m2) / WHEEL_BASE_M;")
            elif abs(avg_wz - avg_cmd_wz) > 0.1:
                print(f"  ⚠️  Angular velocity mismatch: {abs(avg_wz - avg_cmd_wz):.3f} rad/s")
            else:
                print("  ✅ Angular velocity signs match")
            
            # Check if position moved during rotation (should be minimal)
            delta_x = rot_odom[-1]['x'] - rot_odom[0]['x']
            delta_y = rot_odom[-1]['y'] - rot_odom[0]['y']
            linear_drift = math.sqrt(delta_x**2 + delta_y**2)
            
            if linear_drift > 0.1:
                print(f"  ⚠️  Position drifted {linear_drift:.3f}m during in-place rotation!")
                print(f"     This suggests wheel odometry calculation error")
            else:
                print(f"  ✅ Minimal position drift: {linear_drift:.3f}m")
            
            # Check yaw change
            delta_yaw = rot_odom[-1]['yaw'] - rot_odom[0]['yaw']
            print(f"  Yaw change: {math.degrees(delta_yaw):.1f}° (expected ~45°)")
    
    # Compare EKF output with wheel odom
    if data['ekf_odom'] and data['wheel_odom']:
        print("\n🔀 EKF vs WHEEL ODOM COMPARISON")
        
        # Sample at similar times
        for phase_name, start, end in [("Forward", forward_start, forward_end),
                                        ("Rotation", rot_start, rot_end)]:
            wheel = [d for d in data['wheel_odom'] if start <= d['time'] <= end]
            ekf = [d for d in data['ekf_odom'] if start <= d['time'] <= end]
            
            if wheel and ekf:
                wheel_vx = np.mean([d['vx'] for d in wheel])
                ekf_vx = np.mean([d['vx'] for d in ekf])
                wheel_wz = np.mean([d['wz'] for d in wheel])
                ekf_wz = np.mean([d['wz'] for d in ekf])
                
                print(f"\n  {phase_name} phase:")
                print(f"    Wheel vx: {wheel_vx:.3f}, EKF vx: {ekf_vx:.3f}")
                print(f"    Wheel wz: {wheel_wz:.3f}, EKF wz: {ekf_wz:.3f}")
    
    print("\n" + "="*80)


def main():
    rclpy.init()
    
    node = LocalizationDiagnostic()
    
    # Create executor for spinning in background
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Start spinning in background thread
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        # Run test sequence
        filename = node.run_test_sequence()
        
        # Give time for all messages to arrive
        time.sleep(2)
        
        # Analyze
        analyze_data(filename)
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
