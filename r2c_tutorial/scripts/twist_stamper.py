#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        
        # Subscribe to standard Twist (teleop)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_in',
            self.listener_callback,
            10)
            
        # Publish TwistStamped (controller)
        self.publisher = self.create_publisher(
            TwistStamped, 
            'cmd_vel_out', 
            10)
            
        self.get_logger().info('Twist Stamper Node Started')
        self.get_logger().info('Converting Twist -> TwistStamped')

    def listener_callback(self, msg):
        stamped_msg = TwistStamped()
        
        # Add timestamp and frame_id
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        
        # Copy the twist data
        stamped_msg.twist = msg
        
        self.publisher.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
