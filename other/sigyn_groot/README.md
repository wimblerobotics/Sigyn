# Sigyn Groot - Web-Based BehaviorTree Monitor

Real-time behavior tree monitoring for Sigyn with BT.CPP v3 support.

## Features

- 🔴 **Live monitoring** - Real-time visualization of behavior tree execution
- 📊 **Node status tracking** - Visual feedback for IDLE, RUNNING, SUCCESS, FAILURE states
- 🗂️ **Blackboard inspector** - View all blackboard variables in real-time
- 🌐 **Web-based UI** - Access from any browser, no additional software needed
- ⚡ **Low latency** - WebSocket-based updates for instant feedback

## Quick Start

### 1. Install Dependencies

```bash
pip3 install flask flask-socketio pyzmq
```

### 2. Build Package

```bash
cd ~/sigyn_ws
colcon build --symlink-install --packages-select sigyn_groot
source install/setup.bash
```

### 3. Run Your Behavior Tree

Make sure your BT node has `PublisherZMQ` enabled (already done in perimeter_roamer_v3):

```bash
ros2 launch perimeter_roamer_v3 patrol_using_waypoints_launch.py
```

### 4. Launch Monitor

```bash
ros2 launch sigyn_groot monitor.launch.py
```

### 5. Open Web UI

Navigate to: **http://localhost:8080**

## Usage

- **Node Selection**: Click any node to view detailed information
- **Status Colors**:
  - Gray: IDLE
  - Blue (pulsing): RUNNING
  - Green: SUCCESS
  - Red: FAILURE
- **Statistics**: Top-right shows active/total node count
- **Blackboard**: Bottom panel displays all blackboard variables

## Configuration

Customize ZMQ connection:

```bash
ros2 launch sigyn_groot monitor.launch.py zmq_address:=192.168.1.100 zmq_port:=1667
```

## Architecture

```
BT.CPP v3 (PublisherZMQ) → ZMQ Socket → sigyn_groot → WebSocket → Browser
                                            ↓
                                        Flask Server
```

## Integration with Sigyn

This monitor automatically works with any Sigyn behavior tree node that includes:

```cpp
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_, 10, 1666, 1667);
```

## Troubleshooting

**Monitor shows "Disconnected":**
- Ensure your BT node is running with ZMQ publisher enabled
- Check that port 1666 is not blocked by firewall
- Verify ZMQ address/port parameters match your BT node

**Web page not loading:**
- Check that port 8080 is available
- Try accessing via IP: `http://127.0.0.1:8080`
- Check terminal output for Flask server errors

## License

Apache-2.0
