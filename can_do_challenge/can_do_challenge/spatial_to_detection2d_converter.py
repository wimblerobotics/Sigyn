#!/usr/bin/env python3
"""
Converts DepthAI Detection3DArray to vision_msgs Detection2DArray.

Subscribes to /oakd_top/oak/nn/spatial_detections (vision_msgs::Detection3DArray)
Publishes to /oakd/detections (vision_msgs::Detection2DArray)

This provides the heartbeat signal that the BT nodes expect.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection3DArray, Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import Pose2D


class SpatialToDetection2DConverter(Node):
    def __init__(self):
        super().__init__('spatial_to_detection2d_converter')
        
        # QoS: Reliable for spatial detections from DepthAI
        spatial_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # QoS: Best effort for output (matching what BT expects)
        output_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/oakd_top/oak/nn/spatial_detections',
            self.spatial_callback,
            spatial_qos)
        
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/oakd/detections',
            output_qos)
        
        self.get_logger().info('Spatial to Detection2D converter started')
        self.get_logger().info('  Input: /oakd_top/oak/nn/spatial_detections (Detection3DArray)')
        self.get_logger().info('  Output: /oakd/detections (Detection2DArray)')
    
    def spatial_callback(self, spatial_msg):
        """Convert Detection3DArray to Detection2DArray."""
        det_array = Detection2DArray()
        det_array.header = spatial_msg.header
        
        for det_3d in spatial_msg.detections:
            if not det_3d.results:
                continue
            
            det_2d = Detection2D()
            det_2d.header = det_3d.header
            
            # Use empty 2D bbox since we can't project 3D->2D without camera intrinsics
            # This is sufficient for heartbeat purposes. The crash was due to assigning
            # a BoundingBox3D to a BoundingBox2D field.
            det_2d.bbox = BoundingBox2D()
            
            # Convert results
            for result in det_3d.results:
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = result.hypothesis.class_id
                hyp.hypothesis.score = result.hypothesis.score
                det_2d.results.append(hyp)
            
            det_array.detections.append(det_2d)
        
        self.publisher.publish(det_array)
        if det_array.detections:
            self.get_logger().info(f'Published {len(det_array.detections)} detections', once=True)


def main(args=None):
    rclpy.init(args=args)
    converter = SpatialToDetection2DConverter()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
