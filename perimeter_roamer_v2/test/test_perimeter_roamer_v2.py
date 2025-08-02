#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time

class TestPerimeterRoamerV2(unittest.TestCase):
    
    def setUp(self):
        rclpy.init()
        self.node = Node('test_perimeter_roamer_v2')
        
        # Create test publishers
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        
        # Create test subscribers
        self.cmd_vel_received = False
        self.cmd_vel_msg = None
        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.state_received = False
        self.state_msg = None
        self.state_sub = self.node.create_subscription(
            String,
            '/roamer_state',
            self.state_callback,
            10
        )
        
        self.space_type_received = False
        self.space_type_msg = None
        self.space_type_sub = self.node.create_subscription(
            String,
            '/space_type',
            self.space_type_callback,
            10
        )
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def cmd_vel_callback(self, msg):
        self.cmd_vel_received = True
        self.cmd_vel_msg = msg
    
    def state_callback(self, msg):
        self.state_received = True
        self.state_msg = msg
    
    def space_type_callback(self, msg):
        self.space_type_received = True
        self.space_type_msg = msg
    
    def create_test_scan(self, front_dist=2.0, left_dist=1.5, right_dist=1.5):
        """Create a test laser scan message"""
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.angle_min = -1.5708  # -90 degrees
        scan.angle_max = 1.5708   # 90 degrees
        scan.angle_increment = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Create ranges array
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [10.0] * num_readings
        
        # Set specific distances
        # Front (0 degrees)
        front_index = int((0 - scan.angle_min) / scan.angle_increment)
        if 0 <= front_index < len(scan.ranges):
            scan.ranges[front_index] = front_dist
        
        # Left (-90 degrees)
        left_index = int((-1.5708 - scan.angle_min) / scan.angle_increment)
        if 0 <= left_index < len(scan.ranges):
            scan.ranges[left_index] = left_dist
        
        # Right (90 degrees)
        right_index = int((1.5708 - scan.angle_min) / scan.angle_increment)
        if 0 <= right_index < len(scan.ranges):
            scan.ranges[right_index] = right_dist
        
        return scan
    
    def create_test_odom(self):
        """Create a test odometry message"""
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        return odom
    
    def test_package_import(self):
        """Test that the package can be imported"""
        try:
            from perimeter_roamer_v2.perimeter_roamer import PerimeterRoamer
            self.assertTrue(True, "Package imported successfully")
        except ImportError as e:
            self.fail(f"Failed to import package: {e}")
    
    def test_basic_functionality(self):
        """Test basic functionality with mock data"""
        # This test would require running the actual node
        # For now, just test that we can create the test messages
        scan = self.create_test_scan()
        odom = self.create_test_odom()
        
        self.assertIsNotNone(scan)
        self.assertIsNotNone(odom)
        self.assertEqual(scan.header.frame_id, 'base_link')
        self.assertEqual(odom.header.frame_id, 'odom')

if __name__ == '__main__':
    unittest.main() 