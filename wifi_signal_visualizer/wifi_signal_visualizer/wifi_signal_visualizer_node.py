#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sqlite3
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from scipy.interpolate import Rbf  # Import Radial Basis Function interpolator

class WifiSignalVisualizerNode(Node):
    def __init__(self):
        super().__init__('wifi_signal_visualizer_node')
        topic_name = '/global_costmap/costmap'
        topic_type = OccupancyGrid
        self.get_logger().info(f'waiting for topic {topic_name} to be published...')
        while not self.topic_is_available(topic_name):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'topic {topic_name} is now available.')

        msg = self.wait_for_message(topic_type, topic_name)
        # print(f'msg: {msg}')
        self.costmap_resolution = msg.info.resolution
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height

        self.publisher = self.create_publisher(OccupancyGrid, 'wifi_signal_strength_costmap', 10)
        self.resolution = self.costmap_resolution
        self.costmap = np.full((self.costmap_width, self.costmap_height), -1, dtype=np.int8)
        self.db_path = '/home/ros/sigyn_ws/src/Sigyn/wifi_data.db'
        self.declare_parameter('enable_interpolation', True)
        self.declare_parameter('max_interpolation_distance', 2.0)  # in meters
        self.max_interpolation_distance = self.get_parameter('max_interpolation_distance').value
        self.declare_parameter('generate_new_data', False)
        self.generate_new_data = self.get_parameter('generate_new_data').value

        self.wifi_data = []  # List to store wifi data (x, y, signal_strength)
        self.create_table()

        if self.generate_new_data:
            self.clear_wifi_data()
            self.generate_wifi_data()
        else:
            self.load_wifi_data()
            
        if self.wifi_data:
            signal_levels = [signal_level for _, _, signal_level in self.wifi_data]
            self.min_signal_level = min(signal_levels)
            self.max_signal_level = max(signal_levels)
            self.get_logger().info(f"Min signal level: {self.min_signal_level}, Max signal level: {self.max_signal_level}")
        else:
            self.get_logger().info("No WiFi data available to determine min and max signal levels.")

    def topic_is_available(self, topic_name: str) -> bool:
        """
        Check if a ROS topic is available.
        """
        topic_names_and_types = self.get_topic_names_and_types()
        return any(topic_name == name for name, _ in topic_names_and_types)

    def wait_for_message(self, message_type, topic_name, timeout_sec=20.0):
        """
        Wait for a message on the given topic.
        """
        msg = None
        received = False

        def callback(msg_data):
            nonlocal msg, received
            msg = msg_data
            received = True

        sub = self.create_subscription(message_type, topic_name, callback, 1)

        start_time = self.get_clock().now()
        while not received and (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)

        if received:
            return msg
        else:
            print(f'wait_for_message msg failed')
            return None

    def publish_costmap(self):
        """
        Publishes the costmap with interpolated WiFi signal strengths.
        """
        occupancy_grid = OccupancyGrid()
        occupancy_grid.data = [-1] * (self.costmap_width * self.costmap_height)
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.costmap_resolution
        occupancy_grid.info.width = self.costmap_width
        occupancy_grid.info.height = self.costmap_height

        # Extract data points
        points_x = np.array([x / self.costmap_resolution for x, _, _ in self.wifi_data])
        points_y = np.array([y / self.costmap_resolution for _, y, _ in self.wifi_data])
        values = np.array([signal_strength for _, _, signal_strength in self.wifi_data])

        # Check if interpolation is enabled
        enable_interpolation = self.get_parameter('enable_interpolation').value

        if enable_interpolation:
            # Interpolate the data using Radial Basis Function (RBF)
            if len(self.wifi_data) > 0:
                rbf_interpolator = Rbf(points_x, points_y, values, function='linear')
                grid_x, grid_y = np.meshgrid(
                    np.arange(self.costmap_width),
                    np.arange(self.costmap_height)
                )
                interpolated_values = rbf_interpolator(grid_x, grid_y)

                # Replace NaN values with -1 (unknown)
                interpolated_values = np.nan_to_num(interpolated_values, nan=-1)

                # Apply maximum distance threshold
                replace_count = 0
                for i in range(self.costmap_width):
                    for j in range(self.costmap_height):
                        # Check distance to nearest data point
                        distances = np.sqrt((points_x - i)**2 + (points_y - j)**2)
                        min_distance = np.min(distances) * self.resolution  # Convert grid units to meters
                        if min_distance > self.max_interpolation_distance:
                            replace_count += 1
                            interpolated_values[j, i] = -1  # Too far, set to unknown
                print(f'Number of points replaced due to max distance: {replace_count}')

                # Convert to int8 and flatten
                occupancy_grid.data = interpolated_values.astype(np.int8).flatten().tolist()
            else:
                occupancy_grid.data = self.costmap.flatten().tolist()  # No wifi data, publish empty costmap
        else:
            # No interpolation, use raw data
            for x, y, signal_strength in self.wifi_data:
                grid_x = int(x / self.costmap_resolution)
                grid_y = int(y / self.costmap_resolution)
                if 0 <= grid_x < self.costmap_width and 0 <= grid_y < self.costmap_height:
                    occupancy_grid.data[grid_x + grid_y * self.costmap_width] = int(signal_strength)

        self.publisher.publish(occupancy_grid)

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
                    signal_level REAL
                )
            """)
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            self.get_logger().error(f"Error creating table: {e}")

    def insert_data(self, x, y, bit_rate, link_quality, signal_level):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("""
                INSERT INTO wifi_data (x, y, bit_rate, link_quality, signal_level)
                VALUES (?, ?, ?, ?, ?)
            """, (x, y, bit_rate, link_quality, signal_level))
            # print(f'inserted data at x={x}, y={y}, bit_rate={bit_rate}, link_quality={link_quality}, signal_level={signal_level}')
            conn.commit()
            conn.close()
        except sqlite3.Error as e:
            self.get_logger().error(f"Error inserting data: {e}")

    def clear_wifi_data(self):
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("DELETE FROM wifi_data")
            conn.commit()
            conn.close()
            self.get_logger().info("Cleared existing wifi data from database.")
        except sqlite3.Error as e:
            self.get_logger().error(f"Error clearing wifi data: {e}")

    def generate_wifi_data(self):
        """
        Generates wifi data and stores it in the database.
        """
        inserted_rows = 0
        max_dim = max(self.costmap_width, self.costmap_height) * self.costmap_resolution
        max_dist = np.sqrt(self.costmap_width**2 + self.costmap_height**2)
        print(f'max_dim: {max_dim}')
        for i in range(0, self.costmap_width, 20):
            for j in range(0, self.costmap_height, 20):
                # distance = np.sqrt(i**2 + j**2)
                # # signal_strength = int(distance / max_dist * 127)
                # signal_strength = int(np.sin(distance / max_dist * 2 * np.pi) * 50) + 40
                # if i == 1:
                #     print(f'i: {i}, j: {j}, distance: {distance}, signal_strength: {signal_strength}') 
                # # if (signal_strength > 127):
                # #     signal_strength = 127

                # self.costmap[i, j] = signal_strength
                # self.wifi_data.append((i * self.costmap_resolution, j * self.costmap_resolution, signal_strength))  # Store wifi data
                real_x = i * self.costmap_resolution
                real_y = j * self.costmap_resolution
                distance = np.sqrt(real_x**2 + real_y**2)
                max_dim = max(self.costmap_width, self.costmap_height) * self.costmap_resolution
                signal_strength = int(np.sin(distance / max_dim * 2 * np.pi) * 50) + 49
                if 0 <= i < self.costmap_width and 0 <= j < self.costmap_height:
                    self.costmap[i, j] = signal_strength
                    self.insert_data(real_x, real_y, 200000000, 1.0, signal_strength)
                    self.wifi_data.append((real_x, real_y, signal_strength))  # Store wifi data
                    # print(f'wifi_signal_visualizer_node: inserted data at x={real_x}, y={real_y}, i={i}, j={j}, distance={distance}, signal_strength={signal_strength}')
                    inserted_rows += 1
            if (i % 100) == 0:
                print(f'wifi_signal_visualizer_node: processed {inserted_rows} rows of data')

    def load_wifi_data(self):
        """
        Loads wifi data from the database.
        """
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            cursor.execute("SELECT x, y, signal_level FROM wifi_data")
            rows = cursor.fetchall()
            conn.close()

            for row in rows:
                x, y, signal_level = row
                self.wifi_data.append((x, y, signal_level))
            self.get_logger().info(f"Loaded {len(self.wifi_data)} wifi data points from database.")

        except sqlite3.Error as e:
            self.get_logger().error(f"Error loading wifi data: {e}")

    def main_loop(self):
        """
        Main loop to generate random data and publish the costmap.
        """
        rate = self.create_rate(5)  # 6 Hz
        while rclpy.ok():
            self.publish_costmap()
            rclpy.spin_once(self)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = WifiSignalVisualizerNode()
    node.main_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
