#!/usr/bin/env python3
"""
Battery overlay publisher for RViz2
Subscribes to battery status and publishes as OverlayText
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from rviz_2d_overlay_msgs.msg import OverlayText


class BatteryOverlayPublisher(Node):
    def __init__(self):
        super().__init__('battery_overlay_publisher')
        
        # Parameters
        self.declare_parameter('battery_topic', '/sigyn/teensy_bridge/battery/status')
        self.declare_parameter('overlay_topic', '/battery_overlay_text')
        self.declare_parameter('min_voltage', 30.0)
        self.declare_parameter('max_voltage', 42.0)
        self.declare_parameter('filter_battery_id', '36VLIPO')  # Filter for specific battery
        
        battery_topic = self.get_parameter('battery_topic').value
        overlay_topic = self.get_parameter('overlay_topic').value
        self.min_voltage = self.get_parameter('min_voltage').value
        self.max_voltage = self.get_parameter('max_voltage').value
        self.filter_battery_id = self.get_parameter('filter_battery_id').value
        
        # Subscriber and publisher
        self.battery_sub = self.create_subscription(
            BatteryState,
            battery_topic,
            self.battery_callback,
            10
        )
        
        self.overlay_pub = self.create_publisher(
            OverlayText,
            overlay_topic,
            10
        )
        
        self.get_logger().info(f'Battery overlay publisher started')
        self.get_logger().info(f'  Battery topic: {battery_topic}')
        self.get_logger().info(f'  Overlay topic: {overlay_topic}')
        self.get_logger().info(f'  Filter battery ID: {self.filter_battery_id}')
        
    def battery_callback(self, msg: BatteryState):
        """Convert battery state to overlay text"""
        # Filter by battery ID (check header.frame_id)
        if self.filter_battery_id and msg.header.frame_id != self.filter_battery_id:
            return  # Skip this battery
        
        overlay = OverlayText()
        
        # Action: ADD = 0 (show overlay)
        overlay.action = OverlayText.ADD
        
        # Position and size
        overlay.width = 200
        overlay.height = 40
        overlay.horizontal_distance = 10
        overlay.vertical_distance = 10
        overlay.horizontal_alignment = OverlayText.LEFT
        overlay.vertical_alignment = OverlayText.TOP
        
        # Colors (RGBA 0-1 range)
        voltage = msg.voltage
        percentage = msg.percentage * 100  # Convert to percentage
        
        # Color based on battery level
        if percentage > 50:
            # Green
            overlay.fg_color.r = 0.0
            overlay.fg_color.g = 1.0
            overlay.fg_color.b = 0.0
            overlay.fg_color.a = 1.0
        elif percentage > 20:
            # Yellow
            overlay.fg_color.r = 1.0
            overlay.fg_color.g = 1.0
            overlay.fg_color.b = 0.0
            overlay.fg_color.a = 1.0
        else:
            # Red
            overlay.fg_color.r = 1.0
            overlay.fg_color.g = 0.0
            overlay.fg_color.b = 0.0
            overlay.fg_color.a = 1.0
        
        # Background (semi-transparent black)
        overlay.bg_color.r = 0.0
        overlay.bg_color.g = 0.0
        overlay.bg_color.b = 0.0
        overlay.bg_color.a = 0.5
        
        # Font size and style
        overlay.line_width = 2
        overlay.text_size = 14.0
        
        # Text content
        overlay.text = f"Battery: {voltage:.1f}V ({percentage:.0f}%)"
        
        # Publish
        self.overlay_pub.publish(overlay)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryOverlayPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
