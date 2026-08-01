#!/usr/bin/env python3
"""
Simple LaserScan QoS relay node.

Subscribes to a laser scan with BEST_EFFORT and republishes with BEST_EFFORT
and appropriate depth to avoid WiFi network saturation.

This node works around the laser_filters package's hardcoded RELIABLE QoS
with depth=1000, which causes severe network congestion when new subscribers
connect over WiFi.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class LaserScanQoSRelay(Node):
    """Relay LaserScan messages with proper QoS for WiFi networks."""

    def __init__(self):
        super().__init__('laser_scan_qos_relay')
        
        # Declare parameters
        self.declare_parameter('input_topic', 'scan_filtered')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('queue_size', 5)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        queue_size = self.get_parameter('queue_size').value
        
        # Subscribe with RELIABLE to match laser_filters publisher
        # (laser_filters hardcodes RELIABLE with depth 1000)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size
        )
        
        # Publish with BEST_EFFORT for WiFi - prevents retransmission congestion
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size
        )
        
        # Subscribe with RELIABLE to match filter's RELIABLE publisher
        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            sub_qos
        )
        
        # Publish with BEST_EFFORT and low depth to prevent WiFi saturation
        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            pub_qos
        )
        
        self.get_logger().info(
            f'LaserScan QoS Relay: {input_topic} -> {output_topic} '
            f'(BEST_EFFORT, depth={queue_size})'
        )
        
    def scan_callback(self, msg: LaserScan):
        """Relay the scan message."""
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanQoSRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
