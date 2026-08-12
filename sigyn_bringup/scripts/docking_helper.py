#!/usr/bin/env python3
"""
Docking Helper Script for Sigyn Robot

Provides utilities for testing and triggering the Nav2 docking server.
Uses AprilTag ID 1 detection and charging port current monitoring.

Date: 2026-07-31
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import DockRobot, UndockRobot
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState
from vision_msgs.msg import Detection3DArray
import math


class DockingHelper(Node):
    """Helper node for managing docking operations."""
    
    def __init__(self):
        super().__init__('docking_helper')
        
        # Action clients
        self._dock_client = ActionClient(self, DockRobot, '/dock_robot')
        self._undock_client = ActionClient(self, UndockRobot, '/undock_robot')
        
        # Subscribers for monitoring
        self._apriltag_sub = self.create_subscription(
            Detection3DArray,
            '/oakd_apriltag_node/detections',
            self._apriltag_callback,
            10
        )
        self._charger_sub = self.create_subscription(
            BatteryState,
            '/sigyn/power/charger',
            self._charger_callback,
            10
        )
        self._amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10
        )
        
        self._last_detection = None
        self._last_charger = None
        self._last_amcl_pose = None
        self._last_docking_state = None
        
        self.get_logger().info('Docking helper initialized')
    
    def _apriltag_callback(self, msg):
        """Monitor AprilTag detections."""
        prev_had_detection = self._last_detection is not None and self._last_detection.detections
        self._last_detection = msg
        
        if msg.detections:
            for detection in msg.detections:
                if detection.results:
                    tag_id = detection.results[0].hypothesis.class_id
                    score = detection.results[0].hypothesis.score
                    pos = detection.results[0].pose.pose.position
                    orient = detection.results[0].pose.pose.orientation
                    
                    # Log detection event
                    if not prev_had_detection:
                        self.get_logger().info(
                            f'🎯 APRILTAG ACQUIRED - ID:{tag_id} @ '
                            f'x={pos.x:.3f}m y={pos.y:.3f}m z={pos.z:.3f}m '
                            f'score={score:.1f}'
                        )
                    
                    # Always log current state for debugging
                    self.get_logger().info(
                        f'AprilTag ID {tag_id}: '
                        f'pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] '
                        f'orient=[{orient.x:.3f}, {orient.y:.3f}, {orient.z:.3f}, {orient.w:.3f}] '
                        f'score={score:.1f}'
                    )
        elif prev_had_detection:
            self.get_logger().warn('❌ APRILTAG LOST - No detection')
    
    def _charger_callback(self, msg):
        """Monitor charging status."""
        self._last_charger = msg
        if msg.current > 0.01:
            self.get_logger().debug(
                f'Charging: {msg.voltage:.2f}V, {msg.current:.2f}A'
            )
    
    def _amcl_pose_callback(self, msg):
        """Track robot localization."""
        self._last_amcl_pose = msg
    
    def _format_pose(self, pose_msg):
        """Format pose for readable logging."""
        pos = pose_msg.pose.pose.position
        orient = pose_msg.pose.pose.orientation
        
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return f'pos=({pos.x:.4f}, {pos.y:.4f}) yaw={yaw:.4f}rad ({math.degrees(yaw):.2f}°)'
    
    def dock_robot(self, dock_id='home_charging_dock', use_dock_id=True):
        """
        Send docking command to Nav2 docking server.
        
        Args:
            dock_id: Name of the dock to approach (when use_dock_id=True)
            use_dock_id: If True, use dock_id; if False, use current position
        """
        self.get_logger().info(f'Requesting docking to: {dock_id if use_dock_id else "current position"}')
        
        if not self._dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Docking server not available!')
            return False
        
        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = use_dock_id
        
        if use_dock_id:
            goal_msg.dock_id = dock_id
            goal_msg.navigate_to_staging_pose = True
        else:
            # Using current position - AprilTag must be visible
            goal_msg.navigate_to_staging_pose = False
            goal_msg.dock_type = 'simple_charging_dock'
        
        send_goal_future = self._dock_client.send_goal_async(
            goal_msg,
            feedback_callback=self._dock_feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Docking goal rejected!')
            return False
        
        self.get_logger().info('Docking goal accepted, waiting for result...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.success:
            self.get_logger().info('✓ Docking completed successfully!')
            if self._last_charger and self._last_charger.current > 0.01:
                self.get_logger().info(
                    f'✓ Charging confirmed: {self._last_charger.voltage:.2f}V, '
                    f'{self._last_charger.current:.2f}A'
                )
        else:
            self.get_logger().error(f'✗ Docking failed: {result.error_code}')
        
        return result.success
    
    def _dock_feedback_callback(self, feedback_msg):
        """Display docking progress."""
        feedback = feedback_msg.feedback
        state_names = {
            0: 'INITIAL',
            1: 'WAIT_FOR_VALID_DOCK',
            2: 'DOCK_APPROACH',
            3: 'DOCKING',
            4: 'WAIT_FOR_CHARGE',
        }
        state_str = state_names.get(feedback.state, f'UNKNOWN({feedback.state})')
        
        # Log state changes with robot pose
        if feedback.state != self._last_docking_state:
            self._last_docking_state = feedback.state
            pose_str = self._format_pose(self._last_amcl_pose) if self._last_amcl_pose else 'POSE UNKNOWN'
            self.get_logger().info(
                f'🔄 STATE CHANGE → {state_str} | Robot: {pose_str}'
            )
        
        self.get_logger().info(
            f'📍 DOCKING STATE: {state_str} | '
            f'Retries: {feedback.num_retries} | '
            f'AprilTag: {"VISIBLE" if self._last_detection and self._last_detection.detections else "NO DETECTION"}'
        )
        
        # Log AprilTag pose during docking
        if self._last_detection and self._last_detection.detections:
            for det in self._last_detection.detections:
                if det.results:
                    pos = det.results[0].pose.pose.position
                    self.get_logger().info(
                        f'   └─ Tag at: x={pos.x:.3f}m y={pos.y:.3f}m z={pos.z:.3f}m'
                    )
    
    def undock_robot(self):
        """Send undocking command."""
        self.get_logger().info('Requesting undock')
        
        if not self._undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Docking server not available!')
            return False
        
        goal_msg = UndockRobot.Goal()
        
        send_goal_future = self._undock_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Undocking goal rejected!')
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.success:
            self.get_logger().info('✓ Undocking completed successfully!')
        else:
            self.get_logger().error(f'✗ Undocking failed: {result.error_code}')
        
        return result.success
    
    def get_status(self):
        """Print current detection and charging status."""
        print('\n=== Docking Status ===')
        
        if self._last_detection and self._last_detection.detections:
            print('\nAprilTag Detection:')
            for det in self._last_detection.detections:
                if det.results:
                    tag_id = det.results[0].hypothesis.class_id
                    score = det.results[0].hypothesis.score
                    pos = det.results[0].pose.pose.position
                    print(f'  Tag ID: {tag_id}')
                    print(f'  Score: {score:.2f}')
                    print(f'  Distance: {pos.z:.3f}m')
                    print(f'  Offset: x={pos.x:.3f}m, y={pos.y:.3f}m')
        else:
            print('\nAprilTag: No detection')
        
        if self._last_charger:
            print('\nCharging Port:')
            print(f'  Voltage: {self._last_charger.voltage:.2f}V')
            print(f'  Current: {self._last_charger.current:.2f}A')
            print(f'  Status: {"CHARGING" if self._last_charger.current > 0.01 else "NOT CHARGING"}')
        else:
            print('\nCharging: No data')
        
        print('\n')


def main():
    rclpy.init()
    
    if len(sys.argv) < 2:
        print('Usage:')
        print('  ros2 run sigyn_bringup docking_helper.py dock [dock_id]')
        print('  ros2 run sigyn_bringup docking_helper.py undock')
        print('  ros2 run sigyn_bringup docking_helper.py status')
        sys.exit(1)
    
    helper = DockingHelper()
    
    command = sys.argv[1].lower()
    
    if command == 'dock':
        dock_id = sys.argv[2] if len(sys.argv) > 2 else 'home_charging_dock'
        helper.dock_robot(dock_id)
    elif command == 'undock':
        helper.undock_robot()
    elif command == 'status':
        # Spin briefly to collect messages
        import time
        for _ in range(10):
            rclpy.spin_once(helper, timeout_sec=0.1)
            time.sleep(0.1)
        helper.get_status()
    else:
        print(f'Unknown command: {command}')
        sys.exit(1)
    
    helper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
