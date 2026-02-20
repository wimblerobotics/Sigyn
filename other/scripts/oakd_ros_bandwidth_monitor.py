#!/usr/bin/env python3
"""
ROS2 node to monitor OAK-D topic bandwidth and USB performance.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from collections import deque
import time


class OAKDBandwidthMonitor(Node):
    def __init__(self):
        super().__init__('oakd_bandwidth_monitor')
        
        self.topics = {
            '/oakd_top/color/image': Image,
            '/oakd_top/color/image/compressed': CompressedImage,
            '/oakd_top/stereo/points2': PointCloud2,
        }
        
        self.stats = {}
        
        for topic, msg_type in self.topics.items():
            self.stats[topic] = {
                'count': 0,
                'bytes': 0,
                'timestamps': deque(maxlen=100)
            }
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.callback(msg, t),
                10
            )
        
        self.timer = self.create_timer(2.0, self.print_stats)
        self.get_logger().info('OAK-D Bandwidth Monitor started')
    
    def callback(self, msg, topic):
        stats = self.stats[topic]
        stats['count'] += 1
        stats['timestamps'].append(time.time())
        
        # Calculate message size
        if hasattr(msg, 'data'):
            if isinstance(msg.data, bytes):
                stats['bytes'] += len(msg.data)
            elif isinstance(msg.data, list):
                stats['bytes'] += len(msg.data)
    
    def print_stats(self):
        self.get_logger().info('=' * 70)
        self.get_logger().info(f"{'Topic':<40} {'Hz':>8} {'MB/s':>10}")
        self.get_logger().info('-' * 70)
        
        total_bw = 0.0
        for topic, stats in self.stats.items():
            if len(stats['timestamps']) > 1:
                time_span = stats['timestamps'][-1] - stats['timestamps'][0]
                if time_span > 0:
                    hz = len(stats['timestamps']) / time_span
                    mb_per_sec = stats['bytes'] / time_span / (1024 * 1024)
                    total_bw += mb_per_sec
                    
                    self.get_logger().info(
                        f"{topic:<40} {hz:>8.2f} {mb_per_sec:>10.2f}"
                    )
                    
                    # Reset byte counter after reporting
                    stats['bytes'] = 0
        
        self.get_logger().info('-' * 70)
        self.get_logger().info(f"{'Total Bandwidth:':<40} {'':<8} {total_bw:>10.2f}")
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    node = OAKDBandwidthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
