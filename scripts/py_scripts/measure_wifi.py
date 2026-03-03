#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import rclpy
from rclpy.node import Node
import subprocess
import re
import sqlite3
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
import pprint  # Add this import for pretty-printing

class WifiDataCollector(Node):
    def __init__(self):
        super().__init__('wifi_data_collector')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.db_path = '/home/ros/sigyn_ws/src/Sigyn/wifi_data.db'  # Database path
        self.wifi_interface = self.get_wifi_interface()
        if not self.wifi_interface:
            self.get_logger().error("Could not determine WiFi interface. Exiting.")
            rclpy.shutdown()
            exit()
        self.create_table()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_pose = None
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def get_wifi_interface(self):
        try:
            output = subprocess.check_output(["iwconfig"]).decode("utf-8")
            interface_match = re.search(r"^(?P<interface>wlp\d+s\d*)", output, re.MULTILINE)
            if interface_match:
                return interface_match.group("interface")
            else:
                self.get_logger().warn("Could not find wifi interface")
                return None
        except subprocess.CalledProcessError:
            self.get_logger().warn("Could not retrieve wifi interface")
            return None

    def create_table(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS wifi_data (
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                    x REAL,
                    y REAL,
                    bit_rate REAL,
                    link_quality REAL,
                    signal_level REAL,
                    PRIMARY KEY (x, y)
                )
            """)
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            self.get_logger().error(f"Error creating table: {e}")

    def get_wifi_data(self):
        try:
            output = subprocess.check_output(["iwconfig", self.wifi_interface]).decode("utf-8")

            # Extract bit rate
            bit_rate_match = re.search(r"Bit Rate[:=](?P<bit_rate>\d+\.?\d*) (Mb/s|Gb/s)", output)
            if bit_rate_match:
                bit_rate = float(bit_rate_match.group("bit_rate"))
                if bit_rate_match.group(2) == "Gb/s":
                    bit_rate *= 1000  # Convert Gb/s to Mb/s
            else:
                bit_rate = None

            # Extract link quality
            link_quality_match = re.search(r"Link Quality=(?P<link_quality>\d+/\d+)", output)
            if link_quality_match:
                link_quality_str = link_quality_match.group("link_quality")
                link_quality = float(link_quality_str.split('/')[0]) / float(link_quality_str.split('/')[1])
            else:
                link_quality = None

            # Extract signal level
            signal_level_match = re.search(r"Signal level[:=](?P<signal_level>-?\d+) dBm", output)
            signal_level = float(signal_level_match.group("signal_level")) if signal_level_match else None

            return bit_rate, link_quality, signal_level

        except subprocess.CalledProcessError:
            self.get_logger().warn("Could not retrieve wifi data")
            return None, None, None

    def odom_callback(self, msg):
        try:
            # Create a PoseStamped message from the odometry data
            odom_pose = PoseStamped()
            odom_pose.header = msg.header
            odom_pose.pose = msg.pose.pose

            # Lookup the transform from odom to map
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                'odom',  # Source frame
                rclpy.time.Time(),  # Get the latest available transform
                timeout=rclpy.duration.Duration(seconds=5.0)
            )

            # Pretty-print the odom_pose value
            print(f"Odom Pose:\n{pprint.pformat(odom_pose)}")

            # Transform the pose to the map frame
            transformed_pose = do_transform_pose(odom_pose.pose, transform)
            print(f"transformed_pose:\n{pprint.pformat(transformed_pose)}")
            self.current_pose = (transformed_pose.position.x, transformed_pose.position.y)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Transform to map frame failed: {e}")
            self.current_pose = None

    def timer_callback(self):
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available, skipping data insertion")
            return

        self.x, self.y = self.current_pose
        bit_rate, link_quality, signal_level = self.get_wifi_data()
        if bit_rate is not None and link_quality is not None and signal_level is not None:
            self.insert_data(bit_rate, link_quality, signal_level)
            self.get_logger().info(f"X: {self.x}, Y: {self.y}, Bit Rate: {bit_rate}, Link Quality: {link_quality}, Signal Level: {signal_level}")
        else:
            self.get_logger().warn("Could not retrieve all wifi data, skipping insertion")

    def insert_data(self, bit_rate, link_quality, signal_level):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO wifi_data (x, y, bit_rate, link_quality, signal_level)
                VALUES (?, ?, ?, ?, ?)
                ON CONFLICT(x, y) DO UPDATE SET
                    timestamp = CURRENT_TIMESTAMP,
                    bit_rate = excluded.bit_rate,
                    link_quality = excluded.link_quality,
                    signal_level = excluded.signal_level
            """, (self.x, self.y, bit_rate, link_quality, signal_level))
            conn.commit()
            conn.close()
            print(f"Inserted/Updated data: X: {self.x}, Y: {self.y}, Bit Rate: {bit_rate}, Link Quality: {link_quality}, Signal Level: {signal_level}")
        except sqlite3.Error as e:
            self.get_logger().error(f"Error inserting or updating data: {e}")

def main(args=None):
    rclpy.init(args=args)
    wifi_data_collector = WifiDataCollector()
    rclpy.spin(wifi_data_collector)
    rclpy.shutdown()
