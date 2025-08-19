import asyncio
import json
import threading
import time
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

import websockets
import cbor2
import zlib

class TelemetryServer(Node):
    def __init__(self):
        super().__init__('sigyn_websocket_server')
        self.get_logger().info('Starting sigyn_websocket server')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # QoS profiles for different topics
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        
        # Map topic uses reliable/transient local QoS (like RViz2)
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_global_costmap, 10)
        self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates', self.on_global_costmap_update, 10)
        self.create_subscription(BatteryState, '/sigyn/teensy_bridge/battery/status', self.on_battery, 10)
        self.create_subscription(DiagnosticArray, '/sigyn/teensy_bridge/diagnostics', self.on_diag, 10)

        # WebSocket state
        self.connected_clients = set()
        self.ws_host = '0.0.0.0'
        self.ws_port = 8765
        self._ws_loop = None
        
        # Store last received data for new clients
        self.last_map_data = None
        self.last_costmap_data = None
        
        # Last command time for deadman switch
        self.last_cmd_time = 0.0
        self.cmd_timeout = 1.0  # 1 second timeout
        
        # Create timer for deadman check
        self.create_timer(0.1, self.check_deadman)
        
        # Start WebSocket server in separate thread
        self.ws_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
        self.ws_thread.start()

    @property
    def ws_loop(self):
        """Wait for the WebSocket event loop to be available."""
        while self._ws_loop is None:
            time.sleep(0.01)
        return self._ws_loop

    def _run_websocket_server(self):
        """Run WebSocket server in separate thread with its own event loop"""
        asyncio.set_event_loop(asyncio.new_event_loop())
        self._ws_loop = asyncio.get_event_loop()
        
        async def handler(websocket, path):
            self.get_logger().info(f'Client connected: {websocket.remote_address}')
            self.connected_clients.add(websocket)
            
            # Send last received map and costmap data to new client
            try:
                if self.last_map_data:
                    await websocket.send(cbor2.dumps({'t': 'map', 'p': self.last_map_data}))
                    self.get_logger().info('Sent stored map data to new client')
                if self.last_costmap_data:
                    await websocket.send(cbor2.dumps({'t': 'global_costmap', 'p': self.last_costmap_data}))
                    self.get_logger().info('Sent stored costmap data to new client')
            except Exception as e:
                self.get_logger().error(f'Error sending stored data to new client: {e}')
            
            try:
                async for message in websocket:
                    await self._handle_client_message(websocket, message)
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().info(f'Client disconnected: {websocket.remote_address}')
            except Exception as e:
                self.get_logger().error(f'WebSocket error: {e}')
            finally:
                self.connected_clients.discard(websocket)

        async def start_server():
            self.get_logger().info(f'WebSocket server starting on {self.ws_host}:{self.ws_port}')
            await websockets.serve(handler, self.ws_host, self.ws_port)
            self.get_logger().info('WebSocket server started')

        self._ws_loop.run_until_complete(start_server())
        self._ws_loop.run_forever()
        
    def check_deadman(self):
        """Check if cmd_vel commands have timed out"""
        current_time = time.time()
        if self.last_cmd_time > 0 and (current_time - self.last_cmd_time) > self.cmd_timeout:
            # Send stop command
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.last_cmd_time = 0.0  # Reset to avoid repeated stops

    def _broadcast(self, topic: str, payload: Dict[str, Any] | bytes):
        """Thread-safe broadcast to all connected clients"""
        if not self.connected_clients:
            return

        def schedule_broadcast():
            try:
                data = cbor2.dumps({'t': topic, 'p': payload})
                
                # Create a task for each client to send data concurrently
                for client in self.connected_clients:
                    if client.open:
                        self.ws_loop.create_task(client.send(data))

            except Exception as e:
                self.get_logger().error(f'Broadcast preparation error: {e}')
        
        # Schedule the broadcast on the WebSocket thread's event loop
        self.ws_loop.call_soon_threadsafe(schedule_broadcast)

    async def _handle_client_message(self, websocket, message: bytes):
        """Handle incoming client messages"""
        try:
            # Messages from client are expected to be CBOR
            cmd = cbor2.loads(message)
        except Exception as e:
            self.get_logger().warn(f'Invalid CBOR from client: {e}')
            return
            
        t = cmd.get('t')
        p = cmd.get('p', {})
        
        if t == 'cmd_vel':
            msg = Twist()
            msg.linear.x = float(p.get('vx', 0.0))
            msg.angular.z = float(p.get('wz', 0.0))
            self.cmd_vel_pub.publish(msg)
            self.last_cmd_time = time.time()  # Update deadman timer
            self.get_logger().debug(f'cmd_vel: vx={msg.linear.x}, wz={msg.angular.z}')
        elif t == 'nav_goal':
            # TODO: wire to NavigateToPose action
            self.get_logger().info(f'nav_goal received: {p}')
        elif t == 'ping':
            # Respond to ping with CBOR
            await websocket.send(cbor2.dumps({'t': 'pong', 'p': {}}))
        else:
            self.get_logger().warn(f'Unknown command type: {t}')

    # --- ROS Callbacks ---
    def on_scan(self, msg: LaserScan):
        """Process LIDAR scan data"""
        # For now, send basic info. TODO: compress to uint16 mm payload
        payload = {
            'n': len(msg.ranges),
            'min_angle': msg.angle_min,
            'max_angle': msg.angle_max,
            'increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }
        self._broadcast('scan', payload)

    def on_map(self, msg: OccupancyGrid):
        """Process static map data and broadcast it compressed."""
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height}. Compressing and sending...')
        payload = {
            'w': msg.info.width,
            'h': msg.info.height,
            'res': msg.info.resolution,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'theta': msg.info.origin.orientation.z
            },
            'data': zlib.compress(bytes(msg.data))
        }
        # Store for new clients
        self.last_map_data = payload
        self._broadcast('map', payload)

    def on_global_costmap(self, msg: OccupancyGrid):
        """Process global costmap data and broadcast it compressed."""
        self.get_logger().debug(f'Global costmap: {msg.info.width}x{msg.info.height}. Compressing and sending...')
        payload = {
            'w': msg.info.width,
            'h': msg.info.height,
            'data': zlib.compress(bytes(msg.data))
        }
        # Store for new clients
        self.last_costmap_data = payload
        self._broadcast('global_costmap', payload)

    def on_global_costmap_update(self, msg: OccupancyGridUpdate):
        """Process costmap update patches and broadcast them compressed."""
        self.get_logger().debug(f'Costmap update: {msg.width}x{msg.height} @ ({msg.x},{msg.y})')
        payload = {
            'x': msg.x,
            'y': msg.y,
            'w': msg.width,
            'h': msg.height,
            'data': zlib.compress(bytes(msg.data))
        }
        self._broadcast('global_costmap_update', payload)

    def on_battery(self, msg: BatteryState):
        payload = {
            'voltage': msg.voltage,
            'current': msg.current,
            'percentage': msg.percentage,
            'power_supply_status': msg.power_supply_status
        }
        self._broadcast('battery', payload)

    def on_diag(self, msg: DiagnosticArray):
        # For now, just forward the whole thing
        # In future, might want to parse and simplify
        payload = {
            'status': [{'name': s.name, 'level': s.level, 'message': s.message} for s in msg.status]
        }
        self._broadcast('diagnostics', payload)


def main(args=None):
    rclpy.init()
    node = TelemetryServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
