#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import time

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        self.publisher = self.create_publisher(BatteryState, '/sigyn/teensy_bridge/battery/status', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_state)
        self.battery_level = 100.0  # Start at 100%
        self.get_logger().info('Battery simulator started')

    def publish_battery_state(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "36VLIPO"  # Match TeensyV2 battery location name
        
        # Simulate slow battery drain with correct voltage range for 36V LiFePO4 system
        self.battery_level -= 0.1  # Drain 0.1% per second
        if self.battery_level < 18.5:
            self.battery_level = 100.0  # Reset when empty
            self.get_logger().info('Battery recharged to 100%')
        
        msg.percentage = self.battery_level / 100.0  # Convert to 0-1 range
        # 36V LiFePO4 system: 34-44V range (34V warning, 32V critical, 44V full)
        msg.voltage = 32.0 + (self.battery_level / 100.0) * 12.0  # 32-44V range
        msg.current = -1.5  # Discharging at 1.5A
        msg.charge = float('nan')  # Not available
        msg.capacity = float('nan')  # Not available
        msg.design_capacity = float('nan')  # Not available
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        msg.location = "36VLIPO"  # Set location to match frame_id
        
        self.publisher.publish(msg)
        
        if int(self.battery_level) % 10 == 0 and abs(self.battery_level - int(self.battery_level)) < 0.1:
            self.get_logger().info(f'Battery level: {self.battery_level:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    battery_simulator = BatterySimulator()
    
    try:
        rclpy.spin(battery_simulator)
    except KeyboardInterrupt:
        battery_simulator.get_logger().info('Battery simulator shutting down')
    finally:
        battery_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
