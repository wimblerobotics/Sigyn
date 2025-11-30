#!/usr/bin/env python3
"""Temperature Sensor Simulator

Publishes a temperature reading on `/temperature_sensor/raw` and responds to
commands on `/temperature_control/command` to change the target temperature.
This acts as a simple stand-in for a physical temperature sensor so the
ros2_control tutorial can demonstrate sensor integration without custom
Gazebo plugins.
"""

import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature


class TemperatureSensorSim(Node):
    def __init__(self) -> None:
        super().__init__('temperature_sensor_sim')

        # Parameters allow tweaking behaviour without touching code
        self.declare_parameter('initial_temperature', 25.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'temperature_sensor')
        self.declare_parameter('time_constant', 2.0)
        self.declare_parameter('noise_stddev', 0.1)

        self._current_temp = float(self.get_parameter('initial_temperature').value)
        self._target_temp = self._current_temp
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._time_constant = max(0.1, float(self.get_parameter('time_constant').value))
        self._noise_stddev = max(0.0, float(self.get_parameter('noise_stddev').value))

        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self._dt = 1.0 / publish_rate
        self._last_update: Optional[rclpy.time.Time] = None

        # Topics: publish sensor reading and subscribe to command topic
        self._publisher = self.create_publisher(Temperature, '/temperature_sensor/raw', 10)
        self._command_sub = self.create_subscription(
            Float64,
            '/temperature_control/command',
            self._command_callback,
            10,
        )

        # Timer drives periodic publishing
        self._timer = self.create_timer(self._dt, self._publish_temperature)

        self.get_logger().info(
            f'Temperature sensor simulator ready. Initial {self._current_temp:.1f}°C, '
            f'target {self._target_temp:.1f}°C, τ={self._time_constant:.2f}s, '
            f'noise σ={self._noise_stddev:.2f}°C.'
        )

    def _command_callback(self, msg: Float64) -> None:
        self._target_temp = float(msg.data)
        self.get_logger().info(f'New temperature setpoint: {self._target_temp:.2f}°C')

    def _publish_temperature(self) -> None:
        now = self.get_clock().now()
        if self._last_update is None:
            dt = self._dt
        else:
            dt = (now - self._last_update).nanoseconds * 1e-9
            if dt <= 0.0:
                dt = self._dt
        self._last_update = now

        # First-order lag towards target temperature for smooth transitions
        alpha = 1.0 - math.exp(-dt / self._time_constant)
        self._current_temp += (self._target_temp - self._current_temp) * alpha

        noise = random.gauss(0.0, self._noise_stddev) if self._noise_stddev > 0.0 else 0.0
        measured_temp = self._current_temp + noise

        msg = Temperature()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        msg.temperature = measured_temp
        msg.variance = self._noise_stddev ** 2

        self._publisher.publish(msg)

    def destroy_node(self) -> None:  # type: ignore[override]
        self._timer.cancel()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TemperatureSensorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
