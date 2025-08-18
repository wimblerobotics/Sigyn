# sigyn_ios (placeholder)

Multiplatform SwiftUI client (iPhone/iPad/macOS) for Sigyn UI:
- WebSocket client (CBOR) for telemetry/control.
- Panels: Battery (36VLIPO focus), Map with overlays, LIDAR, cmd_vel joystick, RoboClaw status.
- Video via WebRTC (H.265) from OAK-D planned later.

Next steps:
- Initialize a Swift Package/App project.
- Add a CBOR library (e.g., SwiftCBOR) and WebSocket via Network.framework.
- Implement Battery + WS connect first; then Map + LIDAR overlay.
