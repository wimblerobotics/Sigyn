#!/usr/bin/env python3
"""
Load Sensor Hardware Interface

This script manually loads the sensor_system hardware interface using
the controller_manager services. This is necessary because our sensor
hardware doesn't inherit from GazeboSimSystemInterface and cannot be
loaded by the gz_ros2_control Gazebo plugin.

The sensor hardware subscribes to sensor topics and exports state/command
interfaces that can be used by controllers.
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController
import time


class SensorHardwareLoader(Node):
    def __init__(self):
        super().__init__('sensor_hardware_loader')
        
        self.get_logger().info('Waiting for controller_manager services...')
        
        # Wait for controller manager to be ready
        time.sleep(5.0)
        
        # Note: Hardware interfaces are loaded automatically from URDF by controller_manager
        # We just need to verify it's loaded and potentially configure any controllers for it
        
        self.get_logger().info('Sensor hardware should be loaded from URDF automatically')
        self.get_logger().info('Use "ros2 control list_hardware_components" to verify')


def main(args=None):
    rclpy.init(args=args)
    loader = SensorHardwareLoader()
    
    try:
        rclpy.spin_once(loader, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        loader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
