#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import sqlite3
import os
import math


class WaypointCapture(Node):
    def __init__(self):
        super().__init__('waypoint_capture')
        
        # Initialize database
        self.db_path = os.path.expanduser('~/sigyn_ws/patrol_waypoints.db')
        self.init_database()
        
        # Subscribe to waypoints topic
        self.subscription = self.create_subscription(
            MarkerArray,
            '/waypoints',
            self.waypoint_callback,
            10
        )
        
        self.get_logger().info(f'Waypoint capture node started. Database: {self.db_path}')
        
    def init_database(self):
        """Initialize or clear the SQLite database"""
        try:
            # Remove existing database file if it exists
            if os.path.exists(self.db_path):
                os.remove(self.db_path)
                self.get_logger().info(f'Cleared existing database: {self.db_path}')
            
            # Create new database and table
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute('''
                CREATE TABLE waypoints (
                    id INTEGER PRIMARY KEY,
                    x_pose REAL NOT NULL,
                    y_pose REAL NOT NULL,
                    z_pose REAL NOT NULL,
                    x_orientation REAL NOT NULL,
                    y_orientation REAL NOT NULL,
                    z_orientation REAL NOT NULL,
                    w_orientation REAL NOT NULL,
                    euler_orientation REAL NOT NULL,
                    text TEXT,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            
            conn.commit()
            conn.close()
            
            self.get_logger().info('Database initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize database: {str(e)}')
    
    def quaternion_to_euler_yaw(self, x, y, z, w):
        """Convert quaternion to euler yaw angle in radians using manual calculation"""
        try:
            # Calculate yaw (rotation around z-axis) from quaternion
            # Formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            return yaw
        except Exception as e:
            self.get_logger().warning(f'Failed to convert quaternion to euler: {str(e)}')
            return 0.0
    
    def waypoint_callback(self, msg):
        """Process incoming MarkerArray messages"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            waypoints_processed = 0
            
            for marker in msg.markers:
                # Only process markers with type 9 (TEXT_VIEW_FACING)
                if marker.type == 9:
                    # Extract position
                    x_pose = marker.pose.position.x
                    y_pose = marker.pose.position.y
                    z_pose = marker.pose.position.z
                    
                    # Extract orientation
                    x_orientation = marker.pose.orientation.x
                    y_orientation = marker.pose.orientation.y
                    z_orientation = marker.pose.orientation.z
                    w_orientation = marker.pose.orientation.w
                    
                    # Calculate euler yaw angle
                    euler_yaw = self.quaternion_to_euler_yaw(
                        x_orientation, y_orientation, z_orientation, w_orientation
                    )
                    
                    # Extract text
                    text = marker.text if marker.text else None
                    
                    # Insert or replace waypoint in database
                    cursor.execute('''
                        INSERT OR REPLACE INTO waypoints 
                        (id, x_pose, y_pose, z_pose, x_orientation, y_orientation, 
                         z_orientation, w_orientation, euler_orientation, text)
                        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ''', (
                        marker.id, x_pose, y_pose, z_pose,
                        x_orientation, y_orientation, z_orientation, w_orientation,
                        euler_yaw, text
                    ))
                    
                    waypoints_processed += 1
                    
                    self.get_logger().debug(
                        f'Stored waypoint ID {marker.id}: '
                        f'pos=({x_pose:.3f}, {y_pose:.3f}, {z_pose:.3f}), '
                        f'yaw={math.degrees(euler_yaw):.1f}°, text="{text}"'
                    )
            
            conn.commit()
            conn.close()
            
            if waypoints_processed > 0:
                self.get_logger().info(f'Processed {waypoints_processed} waypoints')
                
        except Exception as e:
            self.get_logger().error(f'Error processing waypoints: {str(e)}')
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Waypoint capture node shutting down')


def main(args=None):
    rclpy.init(args=args)
    
    waypoint_capture = WaypointCapture()
    
    try:
        rclpy.spin(waypoint_capture)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_capture.shutdown()
        waypoint_capture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
