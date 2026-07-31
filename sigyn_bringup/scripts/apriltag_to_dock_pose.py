#!/usr/bin/env python3
"""
AprilTag to Dock Pose Converter

Converts Detection3DArray from AprilTag detector to PoseStamped for docking server.
Filters for Tag ID 1 (charging dock marker) and republishes as detected_dock_pose.

Date: 2026-07-31
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState


class AprilTagToDockPose(Node):
    """Converts AprilTag detections to dock pose for Nav2 docking server."""
    
    def __init__(self):
        super().__init__('apriltag_to_dock_pose')
        
        # Parameters
        self.declare_parameter('dock_tag_id', '1')
        self.declare_parameter('min_detection_score', 50.0)
        
        self._dock_tag_id = self.get_parameter('dock_tag_id').value
        self._min_score = self.get_parameter('min_detection_score').value
        
        # Subscribers
        self._detection_sub = self.create_subscription(
            Detection3DArray,
            '/oakd_apriltag_node/detections',
            self._detection_callback,
            10
        )
        
        self._charger_sub = self.create_subscription(
            BatteryState,
            '/sigyn/power/charger',
            self._charger_callback,
            10
        )
        
        # Publishers
        self._dock_pose_pub = self.create_publisher(
            PoseStamped,
            '/detected_dock_pose',
            10
        )
        
        self._battery_pub = self.create_publisher(
            BatteryState,
            '/battery_state',
            10
        )
        
        self.get_logger().info(
            f'Converting AprilTag ID {self._dock_tag_id} to dock pose '
            f'(min score: {self._min_score})'
        )
    
    def _detection_callback(self, msg: Detection3DArray):
        """Convert AprilTag detections to dock pose."""
        if not msg.detections:
            return
        
        # Find the dock tag (ID 1)
        for detection in msg.detections:
            if not detection.results:
                continue
            
            result = detection.results[0]
            tag_id = result.hypothesis.class_id
            score = result.hypothesis.score
            
            # Check if this is the dock tag with sufficient score
            if tag_id == self._dock_tag_id and score >= self._min_score:
                # Create PoseStamped from detection
                dock_pose = PoseStamped()
                dock_pose.header = detection.header
                dock_pose.pose = result.pose.pose
                
                # Publish for docking server
                self._dock_pose_pub.publish(dock_pose)
                
                self.get_logger().debug(
                    f'Dock detected: Tag {tag_id}, score={score:.1f}, '
                    f'distance={dock_pose.pose.position.z:.3f}m'
                )
                
                # Only publish the best (first valid) detection
                return
    
    def _charger_callback(self, msg: BatteryState):
        """Forward charger status to /battery_state for docking server."""
        # Docking server expects /battery_state, but we publish /sigyn/power/charger
        # Forward the message to the expected topic
        self._battery_pub.publish(msg)
        
        if msg.current > 0.01:
            self.get_logger().debug(
                f'Charging: {msg.voltage:.2f}V, {msg.current:.2f}A'
            )


def main():
    rclpy.init()
    node = AprilTagToDockPose()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
