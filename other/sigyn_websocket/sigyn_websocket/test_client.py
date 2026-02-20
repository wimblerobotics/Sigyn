#!/usr/bin/env python3
"""
Simple test client for sigyn_websocket server.
Tests basic connectivity and command sending.
"""

import asyncio
import json
import cbor2
import websockets

class TestClient:
    def __init__(self, uri="ws://localhost:8765"):
        self.uri = uri
        self.websocket = None
        
    async def connect(self):
        """Connect to the WebSocket server"""
        try:
            self.websocket = await websockets.connect(self.uri)
            print(f"Connected to {self.uri}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
            
    async def disconnect(self):
        """Disconnect from the server"""
        if self.websocket:
            await self.websocket.close()
            print("Disconnected")
            
    async def send_command(self, topic, payload):
        """Send a command to the server"""
        if not self.websocket:
            print("Not connected!")
            return
            
        command = {'t': topic, 'p': payload}
        # Encode message as CBOR
        message = cbor2.dumps(command)
        await self.websocket.send(message)
        print(f"Sent: {command}")
        
    async def listen(self, duration=5.0):
        """Listen for messages from the server"""
        if not self.websocket:
            print("Not connected!")
            return
            
        print(f"Listening for {duration} seconds...")
        try:
            end_time = asyncio.get_event_loop().time() + duration
            while asyncio.get_event_loop().time() < end_time:
                try:
                    # Wait for message with timeout
                    message = await asyncio.wait_for(self.websocket.recv(), timeout=1.0)
                    
                    # Try to decode as CBOR first, then JSON
                    try:
                        data = cbor2.loads(message)
                        print(f"Received (CBOR): {data}")
                    except:
                        try:
                            data = json.loads(message)
                            print(f"Received (JSON): {data}")
                        except:
                            print(f"Received (raw): {message}")
                            
                except asyncio.TimeoutError:
                    continue
                except websockets.exceptions.ConnectionClosed:
                    print("Connection closed by server")
                    break
                    
        except KeyboardInterrupt:
            print("\nStopped listening")
            
    async def test_basic_commands(self):
        """Test basic command functionality"""
        print("\n=== Testing Basic Commands ===")
        
        # Test ping
        await self.send_command('ping', {})
        await asyncio.sleep(0.5)
        
        # Test cmd_vel commands
        await self.send_command('cmd_vel', {'vx': 0.0, 'wz': 0.0})
        await asyncio.sleep(0.5)
        
        # Test nav_goal
        # await self.send_command('nav_goal', {'x': 1.0, 'y': 2.0, 'theta': 0.5, 'frame': 'map'})

async def main():
    client = TestClient()
    
    if not await client.connect():
        return
        
    try:
        # Send some test commands
        await client.test_basic_commands()
        
        # Listen for telemetry data
        await client.listen(duration=10.0)
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    finally:
        await client.disconnect()

def main_sync():
    """Synchronous entry point"""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == '__main__':
    main_sync()
