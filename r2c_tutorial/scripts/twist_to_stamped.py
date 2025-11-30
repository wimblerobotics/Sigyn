#!/usr/bin/env python3
"""
Converts geometry_msgs/Twist to geometry_msgs/TwistStamped

This bridge allows standard teleop nodes (which publish Twist) to work
with controllers that expect TwistStamped messages.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        
        # Parameters
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel')
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.twist_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            TwistStamped,
            output_topic,
            10
        )
        
        self.get_logger().info(f'Converting Twist from {input_topic} to TwistStamped on {output_topic}')
    
    def twist_callback(self, msg):
        """Convert Twist to TwistStamped and publish"""
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = msg
        
        self.publisher.publish(stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToStamped()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
