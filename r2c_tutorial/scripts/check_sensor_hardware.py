#!/usr/bin/env python3
"""
Manually load sensor_system hardware interface after Gazebo starts.

This is a workaround because gz_ros2_control cannot load our custom
SensorHardwareInterface (it's not derived from GazeboSimSystemInterface).

This script waits for controller_manager to be ready, then loads the
sensor_system hardware from a separate URDF description.
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ConfigureController, ListHardwareComponents
import time
import sys


class SensorSystemLoader(Node):
    def __init__(self):
        super().__init__('sensor_system_loader')
        
        self.get_logger().info('Sensor System Loader starting...')
        self.get_logger().warn('⚠️  This is a workaround for Gazebo plugin limitations')
        
        # Wait for controller_manager
        self.get_logger().info('Waiting for controller_manager to be ready...')
        time.sleep(5.0)
        
        # Check if sensor_system is already loaded
        list_hw_client = self.create_client(
            ListHardwareComponents,
            '/controller_manager/list_hardware_components'
        )
        
        if not list_hw_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('controller_manager service not available')
            return
        
        request = ListHardwareComponents.Request()
        future = list_hw_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            components = future.result().name
            self.get_logger().info(f'Found hardware components: {components}')
            
            if 'sensor_system' in components:
                self.get_logger().info('✅ sensor_system is already loaded!')
            else:
                self.get_logger().error('❌ sensor_system is NOT loaded')
                self.get_logger().error('   This means controllers cannot access sensor interfaces')
                self.get_logger().error('   The sensor hardware interface must be loaded for controllers to work')
        else:
            self.get_logger().error('Failed to list hardware components')


def main(args=None):
    rclpy.init(args=args)
    loader = SensorSystemLoader()
    
    try:
        rclpy.spin_once(loader, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        loader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
