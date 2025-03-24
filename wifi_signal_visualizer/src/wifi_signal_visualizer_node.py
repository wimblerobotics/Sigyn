import rclpy
from rclpy.node import Node
from wifi_signal_visualizer.msg import LocationWifiSignal
from nav_msgs.msg import OccupancyGrid
import numpy as np

class WifiSignalVisualizerNode(Node):
    def __init__(self):
        super().__init__('wifi_signal_visualizer_node')
        self.subscription = self.create_subscription(
            LocationWifiSignal,
            'wifi_signal',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(OccupancyGrid, 'costmap', 10)
        self.grid_size = 100
        self.resolution = 0.1
        self.costmap = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

    def listener_callback(self, msg):
        x_index = int(msg.x / self.resolution)
        y_index = int(msg.y / self.resolution)
        if 0 <= x_index < self.grid_size and 0 <= y_index < self.grid_size:
            self.costmap[y_index, x_index] = int(msg.signal_strength * 100)
        self.publish_costmap()

    def publish_costmap(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = self.grid_size
        occupancy_grid.info.height = self.grid_size
        occupancy_grid.data = self.costmap.flatten().tolist()
        self.publisher.publish(occupancy_grid)

def main(args=None):
    rclpy.init(args=args)
    node = WifiSignalVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
