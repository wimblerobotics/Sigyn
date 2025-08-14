import asyncio
import json
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

# Minimal server scaffolding; not yet wired to topics. This is to establish structure.

class TelemetryServer(Node):
    def __init__(self):
        super().__init__('sigyn_websocket_server')
        self.get_logger().info('Starting sigyn_websocket server (scaffold)')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions (placeholders; wire up later)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_global_costmap, 10)
        self.create_subscription(OccupancyGridUpdate, '/global_costmap/costmap_updates', self.on_global_costmap_update, 10)
        self.create_subscription(BatteryState, '/sigyn/teensy_bridge/battery/status', self.on_battery, 10)
        self.create_subscription(DiagnosticArray, '/sigyn/teensy_bridge/diagnostics', self.on_diag, 10)

        # WebSocket state
        self.clients = set()
        self.ws_host = '0.0.0.0'
        self.ws_port = 8765

        # Start asyncio server after an event loop is available
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self._start_ws())

    async def _start_ws(self):
        async def handler(websocket, path):
            self.clients.add(websocket)
            try:
                async for message in websocket:
                    await self._handle_client_message(websocket, message)
            finally:
                self.clients.discard(websocket)

        self.get_logger().info(f'WebSocket listening on {self.ws_host}:{self.ws_port}')
        await websockets.serve(handler, self.ws_host, self.ws_port)

    async def _broadcast(self, topic: str, payload: Dict[str, Any] | bytes, binary: bool = True):
        if not self.clients:
            return
        try:
            if binary:
                data = cbor2.dumps({'t': topic, 'p': payload}) if isinstance(payload, dict) else payload
                await asyncio.gather(*[c.send(data) for c in list(self.clients)])
            else:
                data = json.dumps({'t': topic, 'p': payload})
                await asyncio.gather(*[c.send(data) for c in list(self.clients)])
        except Exception as e:
            self.get_logger().warn(f'Broadcast error: {e}')

    async def _handle_client_message(self, websocket, message: str):
        # Expect small JSON commands initially
        try:
            cmd = json.loads(message)
        except Exception:
            return
        t = cmd.get('t')
        p = cmd.get('p', {})
        if t == 'cmd_vel':
            msg = Twist()
            msg.linear.x = float(p.get('vx', 0.0))
            msg.angular.z = float(p.get('wz', 0.0))
            self.cmd_vel_pub.publish(msg)
        elif t == 'nav_goal':
            # TODO: wire to NavigateToPose action
            pass
        elif t == 'camera_select':
            # TODO: implement if needed
            pass

    # --- ROS Callbacks (stubs) ---
    def on_scan(self, msg: LaserScan):
        # Compress to uint16 mm payload later; for now, send minimal sample count
        payload = {'n': len(msg.ranges)}
        asyncio.create_task(self._broadcast('scan', payload, binary=True))

    def on_map(self, msg: OccupancyGrid):
        payload = {'w': msg.info.width, 'h': msg.info.height, 'res': msg.info.resolution}
        asyncio.create_task(self._broadcast('map_meta', payload, binary=True))

    def on_global_costmap(self, msg: OccupancyGrid):
        payload = {'w': msg.info.width, 'h': msg.info.height}
        asyncio.create_task(self._broadcast('global_costmap_meta', payload, binary=True))

    def on_global_costmap_update(self, msg: OccupancyGridUpdate):
        payload = {'x': msg.x, 'y': msg.y, 'w': msg.width, 'h': msg.height}
        asyncio.create_task(self._broadcast('global_costmap_update', payload, binary=True))

    def on_battery(self, msg: BatteryState):
        # Filter by header.frame_id == '36VLIPO'
        if msg.header.frame_id != '36VLIPO':
            return
        # Use volts, current, and levels
        v = float(msg.voltage) if msg.voltage is not None else 0.0
        i = float(msg.current) if msg.current is not None else 0.0
        state = 'red' if v <= 32.0 else ('yellow' if v <= 34.0 else 'green')
        payload = {'v': v, 'i': i, 'state': state}
        asyncio.create_task(self._broadcast('battery', payload, binary=True))

    def on_diag(self, msg: DiagnosticArray):
        # Look for RoboClaw bits in the array; placeholder until exact key is known
        # We'll forward a minimal status if present
        found = False
        for status in msg.status:
            if 'RoboClaw' in status.name or 'roboclaw' in status.name.lower():
                # Search for known keys
                err_bits = None
                for kv in status.values:
                    if kv.key in ('error_bits', 'err', 'RoboClawError'):
                        try:
                            err_bits = int(kv.value, 0)
                        except Exception:
                            pass
                if err_bits is not None:
                    # Decode selected flags on the server (M1_OVERCURRENT, M2_OVERCURRENT, E_STOP)
                    M1_OVERCURRENT = 0x00000001
                    M2_OVERCURRENT = 0x00000002
                    E_STOP = 0x00000020
                    payload = {
                        'err': err_bits,
                        'm1_oc': bool(err_bits & M1_OVERCURRENT),
                        'm2_oc': bool(err_bits & M2_OVERCURRENT),
                        'estop': bool(err_bits & E_STOP),
                    }
                    asyncio.create_task(self._broadcast('roboclaw', payload, binary=True))
                    found = True
                    break
        if not found:
            return


def main():
    rclpy.init()
    node = TelemetryServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
