#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sqlite3
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter

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
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.costmap_resolution
        occupancy_grid.info.width = self.costmap_width
        occupancy_grid.info.height = self.costmap_height

        # Create grid coordinates
        grid_x, grid_y = np.mgrid[0:self.costmap_width, 0:self.costmap_height]

        # Extract data points
        points = np.array([(x, y) for x, y, _ in self.wifi_data])
        values = np.array([signal_strength for _, _, signal_strength in self.wifi_data])

        # Check if interpolation is enabled
        enable_interpolation = self.get_parameter('enable_interpolation').value

        if enable_interpolation:
            # Interpolate the data
            if len(self.wifi_data) > 0:
                interpolated_values = griddata(points, values, (grid_x, grid_y), method='cubic')  # Changed to cubic

                # Apply maximum distance threshold
                for i in range(self.costmap_width):
                    for j in range(self.costmap_height):
                        if np.isnan(interpolated_values[i, j]):  # No nearby data
                            interpolated_values[i, j] = -1  # Set to unknown
                        else:
                            # Check distance to nearest data point
                            distances = np.sqrt((points[:, 0] - i)**2 + (points[:, 1] - j)**2)
                            min_distance = np.min(distances) * self.resolution  # Convert grid units to meters
                            if min_distance > self.max_interpolation_distance:
                                interpolated_values[i, j] = -1  # Too far, set to unknown

                interpolated_values = gaussian_filter(interpolated_values, sigma=2) # Apply Gaussian smoothing

                # Convert to int8 and flatten
                occupancy_grid.data = interpolated_values.astype(np.int8).flatten().tolist()
            else:
                occupancy_grid.data = self.costmap.flatten().tolist()  # No wifi data, publish empty costmap
        else:
            # No interpolation, use raw data
            for x, y, signal_strength in self.wifi_data:
                if 0 <= x < self.costmap_width and 0 <= y < self.costmap_height:
                    self.costmap[int(x), int(y)] = signal_strength
            occupancy_grid.data = self.costmap.flatten().tolist()
        # if len(self.wifi_data) > 0:
        #     interpolated_values = griddata(points, values, (grid_x, grid_y), method='cubic')  # Changed to cubic

        #     # Apply maximum distance threshold
        #     for i in range(self.costmap_width):
        #         for j in range(self.costmap_height):
        #             if np.isnan(interpolated_values[i, j]):  # No nearby data
        #                 interpolated_values[i, j] = -1  # Set to unknown
        #             else:
        #                 # Check distance to nearest data point
        #                 distances = np.sqrt((points[:, 0] - i)**2 + (points[:, 1] - j)**2)
        #                 min_distance = np.min(distances) * self.resolution  # Convert grid units to meters
        #                 if min_distance > self.max_interpolation_distance:
        #                     interpolated_values[i, j] = -1  # Too far, set to unknown

        #     interpolated_values = gaussian_filter(interpolated_values, sigma=2) # Apply Gaussian smoothing

        #     # Convert to int8 and flatten
        #     occupancy_grid.data = interpolated_values.astype(np.int8).flatten().tolist()
        # else:
        #     occupancy_grid.data = self.costmap.flatten().tolist()  # No wifi data, publish empty costmap

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
        for i in range(0, self.costmap_width, 40):  # Increased data density
            for j in range(0, self.costmap_height, 40):  # Increased data density
                distance = np.sqrt(i**2 + j**2)
                max_dim = max(self.costmap_width, self.costmap_height)
                distance = np.sqrt(i**2 + j**2)
                signal_strength = int((np.sin(distance / max_dim * 5 * np.pi) + 1) / 2 * 100)
                if 0 <= i < self.costmap_width and 0 <= j < self.costmap_height:
                    self.costmap[i, j] = signal_strength
                    self.insert_data(i, j, signal_strength, signal_strength, signal_strength)
                    self.wifi_data.append((i, j, signal_strength))  # Store wifi data
            if (i % 100) == 0:
                print(f'wifi_signal_visualizer_node: processed {i} rows of data')

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
