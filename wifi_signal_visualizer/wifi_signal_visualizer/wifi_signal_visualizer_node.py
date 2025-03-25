#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
# from wifi_signal_visualizer.msg import LocationWifiSignal
from nav_msgs.msg import OccupancyGrid
import numpy as np

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
        print(f'msg: {msg}')
        self.costmap_resolution = msg.info.resolution
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height

        # self.get_logger().info(f'resolution: {self.costmap_resolution}, width: {self.costmap_width}, height: {self.costmap_height}')

        self.publisher = self.create_publisher(OccupancyGrid, 'wifi_signal_strength_costmap', 10)
        self.resolution = self.costmap_resolution
        # self.costmap = np.zeros((costmap_width, self.costmap_height), dtype=np.int8)
        self.costmap = np.full((self.costmap_width, self.costmap_height), -1, dtype=np.int8)

    def topic_is_available(self, topic_name: str) -> bool:
        """
        Check if a ROS topic is available.
        """
        topic_names_and_types = self.get_topic_names_and_types()
        # print(f'topic_names_and_types: {topic_names_and_types}, found: {topic_name in topic_names_and_types}')
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
            # print(f"Received message: {msg}")

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
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.costmap_resolution
        occupancy_grid.info.width = self.costmap_width
        occupancy_grid.info.height = self.costmap_height
        occupancy_grid.data = self.costmap.flatten().tolist()
        self.publisher.publish(occupancy_grid)

def main(args=None):
    rclpy.init(args=args)
    node = WifiSignalVisualizerNode()
    for i in range(100):
        x_index = int(random.randint(0, node.costmap_width - 1))
        y_index = int(random.randint(0, node.costmap_height - 1))
        signal_strength = random.randint(0, 100)
        print(f'node.costmap_width: {node.costmap_width}, node.costmap_height: {node.costmap_height}')
        print(f'x_index: {x_index}, y_index: {y_index}, signal_strength: {signal_strength}')
        if 0 <= x_index < node.costmap_width and 0 <= y_index < node.costmap_height:
            node.costmap[x_index, y_index] = signal_strength
            
    rate = node.create_rate(5)  # 6 Hz
    while rclpy.ok():
        node.publish_costmap()
        rclpy.spin_once(node)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
