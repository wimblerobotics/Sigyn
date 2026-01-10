# sigyn_websocket: Background and Design Prompt

This document captures the intent, constraints, and interfaces for the Sigyn robot’s WebSocket-based UI bridge so another AI or developer can continue implementation quickly.

## Goals
- Expose robot telemetry and control over a single WebSocket (CBOR payloads) for iPhone/iPad/macOS clients.
- Keep latency low and bandwidth efficient. Use WebRTC (H.265) for camera video, handled by a separate component later.
- LAN-only MVP; security can be added later.

## Data Sources (ROS 2)
- Battery: `/sigyn/teensy_bridge/battery/status` (`sensor_msgs/BatteryState`)
  - Use only messages where `header.frame_id == '36VLIPO'`.
  - Client coloring: `v <= 32.0 → red`, `v <= 34.0 → yellow`, else green.
- Diagnostics: `/sigyn/teensy_bridge/diagnostics` (`diagnostic_msgs/DiagnosticArray`)
  - Extract RoboClaw error bits. For MVP, publish booleans for `M1_OVERCURRENT_WARNING`, `M2_OVERCURRENT_WARNING`, `E_STOP`.
- LIDAR: `/scan` (`sensor_msgs/LaserScan`) ~10 Hz, ~450 points, range up to ~12 m.
  - Plan: uint16 millimeters per reading in a binary CBOR frame, optional RLE for max-range.
- Map (static): `/map` (`nav_msgs/OccupancyGrid`), once per session.
  - Send PNG (or zlib) + metadata: width, height, resolution, origin (x,y,theta).
- Global Costmap: `/global_costmap/costmap` (full) and `/global_costmap/costmap_updates` (patches)
  - Research summary: costmap_updates provide rectangular patches (x,y,width,height) with data that overlays onto the full costmap. Send full costmap once; then stream updates. Tile/PNG or RLE recommended.
- Control: `/cmd_vel` (`geometry_msgs/Twist`) up to ~20 Hz; add deadman TTL server-side.
- Nav goal: NavigateToPose action proxy (deferred in MVP).

## Wire Protocol
- Transport: WebSocket on robot (e.g., ws://robot.local:8765); LAN only for MVP.
- Encoding: CBOR for telemetry frames; small JSON allowed for client → server commands.
- Topics (examples):
  - `battery`: { v: float, i: float, state: 'green'|'yellow'|'red' }
  - `roboclaw`: { err:uint32, m1_oc:bool, m2_oc:bool, estop:bool }
  - `map_meta`: { w,h,res,origin:{x,y,theta} } + separate binary tile for pixels
  - `global_costmap_meta`: { w,h,res,origin }
  - `global_costmap_update`: { x,y,w,h,data: bytes|RLE }
  - `scan`: binary payload of uint16 mm, count known via header sent once
  - Commands: `cmd_vel` { vx, wz, ttl_ms }, `nav_goal` { x,y,theta,frame }

## MVP Server Responsibilities
- Subscribe to the topics above, throttle as needed.
- Broadcast compact frames with last-value-wins policy on backpressure.
- Enforce cmd_vel rate limits and deadman timer.
- Send full map and costmap once on first subscription, then only updates.

## iOS/macOS Client (separate package: `sigyn_ios`)
- SwiftUI app (multiplatform). Panels: Battery, Map+Overlays, LIDAR, CmdVel joystick, RoboClaw bits.
- WebSocket client using Network.framework; CBOR decode via a Swift CBOR library.
- Video via WebRTC (H.265) later; initial MVP can skip video.

## Camera Path (later)
- OAK-D H.265 preferred. Use WebRTC to deliver low-latency adaptive video; fallback path could be MJPEG at lower FPS.

## Costmap Update Usage (notes)
- `/global_costmap/costmap` is a full OccupancyGrid; `/global_costmap/costmap_updates` is `map_msgs/OccupancyGridUpdate` with `x,y,width,height` defining a patch and `data` for that region.
- Client maintains a backing image/array; apply updates at (x,y) with width×height stride. Render overlay at a lower refresh (e.g., 2–4 Hz) to coalesce multiple updates.

## Non-Goals (MVP)
- Security (TLS/auth), multi-robot routing, remote access.
- Full NavigateToPose action, camera selection, multi-camera.

## Open Questions
- Confirm exact RoboClaw error bit keys in DiagnosticArray; adjust server parsing accordingly.
- Decide on RLE vs. zlib for LIDAR and costmap patches based on rosbag tests.
- Map/costmap tile layout (single large vs tiled) for memory + redraw performance.

## Implementation Status
- Package skeleton created with a Python server scaffold (`server.py`).
- Entry point: `ros2 run sigyn_websocket server` (after colcon build and deps installed).
- Next steps: wire exact topic parsing, add CBOR binary frames for scans and costmap patches, implement cmd_vel deadman.
