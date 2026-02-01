#!/usr/bin/env python3
"""
--- Can Detection ---
Original frame: oak_rgb_camera_optical_frame
  x: -0.094m
  y: 0.266m
  z: 1.186m

Relative to base_footprint:
  x: 0.283m
  y: 0.074m
  z: 0.395m

Actual can in base_footprint frame:
  x: 0.255m Center of can
  y: 0.085m Center of can
  z: 0.05 - 0.15m center of can height, 0.565 - 0.15m = 0.415m top of can height



--- Can Detection ---
Original frame: oak_rgb_camera_optical_frame
  x: -0.092m
  y: 0.256m
  z: 1.170m

Relative to base_footprint:
  x: 0.283m
  y: 0.072m
  z: 0.414m

Relative to base_link:
  x: 0.283m
  y: 0.072m
  z: 0.342m
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class CanDetectionTransformer(Node):
    def __init__(self):
        super().__init__('can_detection_transformer')
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to can detection
        self.subscription = self.create_subscription(
            PointStamped,
            '/oakd/can_detection',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('Listening to /oakd/can_detection...')
    
    def detection_callback(self, msg):
        try:
            # Transform the point to base_footprint frame
            transform_footprint = self.tf_buffer.lookup_transform(
                'base_footprint',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            point_in_footprint = tf2_geometry_msgs.do_transform_point(msg, transform_footprint)
            
            # Transform the point to base_link frame
            transform_link = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            point_in_link = tf2_geometry_msgs.do_transform_point(msg, transform_link)
            
            # Print results
            self.get_logger().info(
                f'\n--- Can Detection ---'
                f'\nOriginal frame: {msg.header.frame_id}'
                f'\n  x: {msg.point.x:.3f}m'
                f'\n  y: {msg.point.y:.3f}m'
                f'\n  z: {msg.point.z:.3f}m'
                f'\n\nRelative to base_footprint:'
                f'\n  x: {point_in_footprint.point.x:.3f}m'
                f'\n  y: {point_in_footprint.point.y:.3f}m'
                f'\n  z: {point_in_footprint.point.z:.3f}m'
                f'\n\nRelative to base_link:'
                f'\n  x: {point_in_link.point.x:.3f}m'
                f'\n  y: {point_in_link.point.y:.3f}m'
                f'\n  z: {point_in_link.point.z:.3f}m'
            )
            
        except Exception as e:
            self.get_logger().error(f'Transform failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CanDetectionTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()