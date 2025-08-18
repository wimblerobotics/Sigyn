# sigyn_websocket

ROS 2 → WebSocket bridge for Sigyn. Exposes telemetry/control for iOS/macOS clients using CBOR payloads. Video (H.265) via WebRTC is planned separately.

## Build/Run
- Ensure ROS 2 environment is sourced.
- Install Python deps inside your ROS environment: `pip install websockets cbor2` (or via requirements).
- Build with colcon: `colcon build --symlink-install --packages-select sigyn_websocket`
- Run: `ros2 run sigyn_websocket server`

## Topics
- Subscribes: `/scan`, `/map`, `/global_costmap/costmap`, `/global_costmap/costmap_updates`, `/sigyn/teensy_bridge/battery/status`, `/sigyn/teensy_bridge/diagnostics`
- Publishes: `/cmd_vel` (Twist)

See BACKGROUND.md for the design prompt and details.
