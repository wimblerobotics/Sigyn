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
        
        self._last_detection_time = None
        self._had_detection = False
        
        self.get_logger().info(
            f'Converting AprilTag ID {self._dock_tag_id} to dock pose '
            f'(min score: {self._min_score})'
        )
    
    def _detection_callback(self, msg: Detection3DArray):
        """Convert AprilTag detections to dock pose."""
        if not msg.detections:
            if self._had_detection:
                self.get_logger().warn('❌ DOCK TAG LOST - No detections in array')
                self._had_detection = False
            return
        
        # Find the dock tag (ID 1)
        found_dock_tag = False
        for detection in msg.detections:
            if not detection.results:
                continue
            
            result = detection.results[0]
            tag_id = result.hypothesis.class_id
            score = result.hypothesis.score
            
            # Check if this is the dock tag with sufficient score
            if tag_id == self._dock_tag_id:
                if score >= self._min_score:
                    # Create PoseStamped from detection
                    dock_pose = PoseStamped()
                    dock_pose.header = detection.header
                    dock_pose.pose = result.pose.pose
                    
                    # Publish for docking server
                    self._dock_pose_pub.publish(dock_pose)
                    
                    # Log detection state changes
                    if not self._had_detection:
                        self.get_logger().info(
                            f'🎯 DOCK TAG ACQUIRED - ID:{tag_id} '
                            f'pos=[{dock_pose.pose.position.x:.3f}, '
                            f'{dock_pose.pose.position.y:.3f}, '
                            f'{dock_pose.pose.position.z:.3f}] '
                            f'score={score:.1f}'
                        )
                        self._had_detection = True
                    else:
                        # Log continuous tracking
                        self.get_logger().info(
                            f'📍 Dock @ x={dock_pose.pose.position.x:.3f}m '
                            f'y={dock_pose.pose.position.y:.3f}m '
                            f'z={dock_pose.pose.position.z:.3f}m '
                            f'score={score:.1f}'
                        )
                    
                    self._last_detection_time = self.get_clock().now()
                    found_dock_tag = True
                else:
                    self.get_logger().warn(
                        f'⚠️  Dock tag score too low: {score:.1f} < {self._min_score}'
                    )
                
                # Only handle the first valid detection
                break
        
        if not found_dock_tag and self._had_detection:
            self.get_logger().warn('❌ DOCK TAG LOST - Not in detection array')
            self._had_detection = False
    
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
