#!/usr/bin/env python3
"""
BehaviorTree Monitor for Sigyn
Subscribes to ROS2 topic for BT status and streams to web UI
"""

import rclpy
from rclpy.node import Node
import json
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO
import os
from std_msgs.msg import String


class BTMonitorNode(Node):
    """ROS2 node that bridges BT status topic to WebSocket clients."""
    
    def __init__(self, socketio):
        super().__init__('bt_monitor_node')
        self.socketio = socketio
        
        self.get_logger().info('Starting BT Monitor node')
        
        # Tree state
        self.tree_state = {
            'nodes': {},
            'blackboard': {},
            'timestamp': 0
        }
        
        # Subscribe to BT status topic
        self.bt_status_sub = self.create_subscription(
            String,
            '/sigyn/bt_status',
            self.bt_status_callback,
            10
        )
        
        self.get_logger().info('BT Monitor node started, listening on /sigyn/bt_status')
    
    def bt_status_callback(self, msg):
        """Process BT status message from topic."""
        try:
            # Initialize msg count if needed
            if not hasattr(self, '_msg_count'):
                self._msg_count = 0
            
            data = json.loads(msg.data)
            
            if 'nodes' in data:
                timestamp = str(self.get_clock().now().nanoseconds)
                
                # Update all nodes
                for node_data in data['nodes']:
                    node_name = node_data['name']
                    status = node_data['status']
                    
                    # Always update state
                    self.tree_state['nodes'][node_name] = {
                        'status': status,
                        'timestamp': timestamp
                    }
                    
                    # Emit update (even if status unchanged, for UI refresh)
                    self.socketio.emit('node_update', {
                        'node': node_name,
                        'status': status,
                        'timestamp': timestamp
                    })
                    
                    # Debug first few emissions
                    if self._msg_count < 3:
                        print(f"[SOCKETIO] Emitting node_update: {node_name} = {status}")
                
                # Log periodically
                self._msg_count += 1
                
                if self._msg_count % 10 == 0:
                    self.get_logger().info(f'Received {self._msg_count} updates, {len(data["nodes"])} nodes')
                        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
        except Exception as e:
            self.get_logger().error(f'Status callback error: {e}')
    
    def get_tree_state(self):
        """Return current tree state for new clients."""
        return self.tree_state


# Flask app
app = Flask(__name__, 
            template_folder=os.path.join(os.path.dirname(__file__), '..', 'web'),
            static_folder=os.path.join(os.path.dirname(__file__), '..', 'web'))
app.config['SECRET_KEY'] = 'sigyn_groot_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global node instance
bt_node = None


@app.route('/')
def index():
    """Serve the main monitoring page."""
    return render_template('monitor.html')


@socketio.on('connect')
def handle_connect():
    """Handle new client connection."""
    print(f"[SOCKETIO] Client connected")
    if bt_node:
        # Send current tree state to new client
        state = bt_node.get_tree_state()
        print(f"[SOCKETIO] Sending full_state with {len(state.get('nodes', {}))} nodes")
        socketio.emit('full_state', state)


@socketio.on('request_state')
def handle_request_state():
    """Handle client request for current state."""
    if bt_node:
        socketio.emit('full_state', bt_node.get_tree_state())


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    global bt_node
    bt_node = BTMonitorNode(socketio)
    
    # Run Flask in a separate thread
    flask_thread = threading.Thread(
        target=lambda: socketio.run(app, host='0.0.0.0', port=8080, debug=False, use_reloader=False),
        daemon=True
    )
    flask_thread.start()
    
    bt_node.get_logger().info('Web UI available at http://localhost:8080')
    
    try:
        rclpy.spin(bt_node)
    except KeyboardInterrupt:
        pass
    finally:
        bt_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
