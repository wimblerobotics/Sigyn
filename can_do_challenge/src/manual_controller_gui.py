#!/usr/bin/env python3
"""
Manual Controller GUI for Can Do Challenge.

Provides a Tkinter GUI to:
1. Step through behavior tree execution one tick at a time
2. Control simulated sensor values (battery, IMU tilt, E-stop)
3. Monitor gripper/elevator status

Publishes to standard topics that bt_nodes.cpp subscribes to.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import BatteryState, Imu
from geometry_msgs.msg import Twist, Quaternion
from sigyn_interfaces.msg import EStopStatus
import tkinter as tk
from tkinter import ttk
import threading
import math


class ManualControllerGUI(Node):
    def __init__(self):
        super().__init__('manual_controller_gui')
        
        # Declare parameters
        self.declare_parameter('tick_rate_hz', 10.0)
        
        # Publishers for simulated sensors
        self.battery_pub = self.create_publisher(
            BatteryState,
            '/sigyn/teensy_bridge/battery/status',
            10
        )
        
        self.imu_pub = self.create_publisher(
            Imu,
            '/sigyn/teensy_bridge/imu/sensor_0',
            10
        )
        
        self.estop_pub = self.create_publisher(
            EStopStatus,
            '/sigyn/teensy_bridge/safety/estop_status',
            10
        )
        
        # Publisher for BT tick command
        self.bt_tick_pub = self.create_publisher(
            Bool,
            '/can_do_challenge/bt_tick',
            10
        )
        
        # Subscribers to monitor gripper/elevator status
        self.elevator_status_sub = self.create_subscription(
            Twist,
            '/cmd_vel_gripper',
            self.gripper_cmd_callback,
            10
        )
        
        # State variables
        self.battery_voltage = 36.0  # Volts
        self.roll_angle = 0.0  # degrees
        self.pitch_angle = 0.0  # degrees
        self.estop_triggered = False
        
        self.elevator_position = 0.0  # meters
        self.extender_position = 0.0  # meters
        
        # Publish timer for sensor data
        self.sensor_timer = self.create_timer(0.1, self.publish_sensor_data)
        
        # Start GUI in separate thread
        self.gui_thread = threading.Thread(target=self.create_gui, daemon=True)
        self.gui_thread.start()
        
        self.get_logger().info('Manual Controller GUI started')
    
    def gripper_cmd_callback(self, msg):
        """Monitor gripper commands and update simulated positions"""
        # linear.x controls elevator
        # angular.z controls extender
        dt = 0.1  # Update rate
        self.elevator_position += msg.linear.x * dt
        self.extender_position += msg.angular.z * dt
        
        # Clamp positions
        self.elevator_position = max(0.0, min(4.0, self.elevator_position))
        self.extender_position = max(0.0, min(0.5, self.extender_position))
    
    def publish_sensor_data(self):
        """Publish simulated sensor data"""
        # Battery
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = 'base_link'
        battery_msg.voltage = self.battery_voltage
        battery_msg.percentage = (self.battery_voltage - 30.0) / (42.0 - 30.0)
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.present = True
        self.battery_pub.publish(battery_msg)
        
        # IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        
        # Convert roll/pitch to quaternion
        roll_rad = math.radians(self.roll_angle)
        pitch_rad = math.radians(self.pitch_angle)
        
        cy = math.cos(0.0 * 0.5)  # yaw = 0
        sy = math.sin(0.0 * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        self.imu_pub.publish(imu_msg)
        
        # E-Stop
        estop_msg = EStopStatus()
        estop_msg.active = self.estop_triggered  # Field is 'active' not 'estop_triggered'
        # EStopStatus has no header field
        self.estop_pub.publish(estop_msg)
    
    def send_bt_tick(self):
        """Send command to advance BT by one tick"""
        msg = Bool()
        msg.data = True
        self.bt_tick_pub.publish(msg)
        self.get_logger().info('BT Tick sent')
    
    def create_gui(self):
        """Create the Tkinter GUI"""
        root = tk.Tk()
        root.title("Can Do Challenge - Manual Controller")
        root.geometry("500x600")
        
        # Tick Button
        tick_frame = ttk.LabelFrame(root, text="Behavior Tree Control", padding=10)
        tick_frame.pack(fill="x", padx=10, pady=10)
        
        tick_btn = ttk.Button(
            tick_frame,
            text="⏭ TICK (Advance BT One Step)",
            command=self.send_bt_tick
        )
        tick_btn.pack(fill="x")
        
        # Battery Control
        battery_frame = ttk.LabelFrame(root, text="Battery Control", padding=10)
        battery_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Label(battery_frame, text="Battery Voltage (V):").pack()
        self.battery_var = tk.DoubleVar(value=self.battery_voltage)
        battery_scale = ttk.Scale(
            battery_frame,
            from_=20.0,
            to=42.0,
            variable=self.battery_var,
            orient="horizontal",
            command=lambda v: setattr(self, 'battery_voltage', float(v))
        )
        battery_scale.pack(fill="x")
        
        self.battery_label = ttk.Label(battery_frame, text=f"{self.battery_voltage:.1f} V")
        self.battery_label.pack()
        
        def update_battery_label(v):
            self.battery_voltage = float(v)
            self.battery_label.config(text=f"{self.battery_voltage:.1f} V")
        
        battery_scale.config(command=update_battery_label)
        
        # IMU Control
        imu_frame = ttk.LabelFrame(root, text="IMU Tilt Control", padding=10)
        imu_frame.pack(fill="x", padx=10, pady=10)
        
        ttk.Label(imu_frame, text="Roll (degrees):").pack()
        self.roll_var = tk.DoubleVar(value=self.roll_angle)
        roll_scale = ttk.Scale(
            imu_frame,
            from_=-45.0,
            to=45.0,
            variable=self.roll_var,
            orient="horizontal"
        )
        roll_scale.pack(fill="x")
        
        self.roll_label = ttk.Label(imu_frame, text=f"{self.roll_angle:.1f}°")
        self.roll_label.pack()
        
        def update_roll_label(v):
            self.roll_angle = float(v)
            self.roll_label.config(text=f"{self.roll_angle:.1f}°")
        
        roll_scale.config(command=update_roll_label)
        
        ttk.Label(imu_frame, text="Pitch (degrees):").pack()
        self.pitch_var = tk.DoubleVar(value=self.pitch_angle)
        pitch_scale = ttk.Scale(
            imu_frame,
            from_=-45.0,
            to=45.0,
            variable=self.pitch_var,
            orient="horizontal"
        )
        pitch_scale.pack(fill="x")
        
        self.pitch_label = ttk.Label(imu_frame, text=f"{self.pitch_angle:.1f}°")
        self.pitch_label.pack()
        
        def update_pitch_label(v):
            self.pitch_angle = float(v)
            self.pitch_label.config(text=f"{self.pitch_angle:.1f}°")
        
        pitch_scale.config(command=update_pitch_label)
        
        # E-Stop Control
        estop_frame = ttk.LabelFrame(root, text="E-Stop Control", padding=10)
        estop_frame.pack(fill="x", padx=10, pady=10)
        
        self.estop_var = tk.BooleanVar(value=self.estop_triggered)
        
        def toggle_estop():
            self.estop_triggered = self.estop_var.get()
            estop_label.config(
                text="🛑 E-STOP TRIGGERED" if self.estop_triggered else "✅ E-Stop Clear"
            )
        
        estop_check = ttk.Checkbutton(
            estop_frame,
            text="Trigger E-Stop",
            variable=self.estop_var,
            command=toggle_estop
        )
        estop_check.pack()
        
        estop_label = ttk.Label(estop_frame, text="✅ E-Stop Clear")
        estop_label.pack()
        
        # Gripper Status Display
        status_frame = ttk.LabelFrame(root, text="Gripper Status (Read-Only)", padding=10)
        status_frame.pack(fill="x", padx=10, pady=10)
        
        self.elevator_status_label = ttk.Label(
            status_frame,
            text=f"Elevator: {self.elevator_position:.3f} m"
        )
        self.elevator_status_label.pack()
        
        self.extender_status_label = ttk.Label(
            status_frame,
            text=f"Extender: {self.extender_position:.3f} m"
        )
        self.extender_status_label.pack()
        
        # Update status labels periodically
        def update_status():
            self.elevator_status_label.config(
                text=f"Elevator: {self.elevator_position:.3f} m"
            )
            self.extender_status_label.config(
                text=f"Extender: {self.extender_position:.3f} m"
            )
            root.after(100, update_status)
        
        update_status()
        
        # Info label
        info_label = ttk.Label(
            root,
            text="Use Groot to visualize behavior tree\nConnect to localhost:1667",
            font=("TkDefaultFont", 9, "italic")
        )
        info_label.pack(side="bottom", pady=10)
        
        root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    
    node = ManualControllerGUI()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
