#!/usr/bin/env python3
"""
Visualize walls from the database one by one on a costmap.

This script reads the wall database and displays each wall individually
on a costmap topic, allowing navigation through the walls with keyboard input.
"""

import sqlite3
import sys
import select
import termios
import tty
from pathlib import Path
from typing import List, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class WallVisualizer(Node):
    """Visualize walls from database on costmap topic."""
    
    def __init__(self):
        super().__init__('wall_visualizer')
        self.get_logger().info("Wall Visualizer node starting...")
        
        # Define paths
        self.database_path = Path.home() / "sigyn_ws/src/Sigyn/description/models/sigyn_house.db"
        
        # Declare parameters
        self.declare_parameter('costmap_value', 50)
        
        # Get parameters
        self.costmap_value = self.get_parameter('costmap_value').value
        
        # Use map parameters from my_map.yaml
        self.resolution = 0.026176231  # meters per pixel
        self.origin_x = 0.0  # map origin x
        self.origin_y = 0.0  # map origin y
        self.costmap_frame_id = "map"
        
        # Set default costmap size (will be updated from global costmap if available)
        self.map_width = 800
        self.map_height = 600
        
        # Set up publisher
        self.costmap_publisher = self.create_publisher(OccupancyGrid, '/walls_costmap', 10)
        
        # Set up subscriber to get global costmap parameters
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.global_costmap_callback,
            10
        )
        
        # Initialize walls and state
        self.walls = []
        self.current_index = 0
        self.costmap_initialized = False
        
        # Input handling
        self.running = True
        
        # Set up a timer to check for input
        self.input_timer = self.create_timer(0.1, self.check_input)
        
        # Set up terminal for non-blocking input
        self.old_settings = None
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        
    def load_walls_from_database(self) -> List[Dict[str, Any]]:
        """Load walls from the SQLite database."""
        if not self.database_path.exists():
            self.get_logger().error(f"Database not found at: {self.database_path}")
            return []
        
        try:
            conn = sqlite3.connect(str(self.database_path))
            cursor = conn.cursor()
            
            cursor.execute('SELECT * FROM walls ORDER BY id')
            rows = cursor.fetchall()
            
            walls = []
            for row in rows:
                wall_id, name, width, length, x, y, vertical = row
                walls.append({
                    'id': wall_id,
                    'name': name,
                    'width': width,
                    'length': length,
                    'x': x,
                    'y': y,
                    'vertical': bool(vertical)
                })
            
            conn.close()
            self.get_logger().info(f"Loaded {len(walls)} walls from database")
            return walls
            
        except Exception as e:
            self.get_logger().error(f"Error loading walls from database: {e}")
            return []
    
    def create_costmap_for_wall(self, wall: Dict[str, Any]) -> OccupancyGrid:
        """Create a costmap showing only the specified wall."""
        costmap = OccupancyGrid()
        
        # Set header
        costmap.header = Header()
        costmap.header.stamp = self.get_clock().now().to_msg()
        costmap.header.frame_id = self.costmap_frame_id
        
        # Set info
        costmap.info.width = self.map_width
        costmap.info.height = self.map_height
        costmap.info.resolution = self.resolution
        costmap.info.origin.position.x = self.origin_x
        costmap.info.origin.position.y = self.origin_y
        costmap.info.origin.position.z = 0.0
        costmap.info.origin.orientation.w = 1.0
        
        # Initialize with free space (0)
        costmap.data = [0] * (self.map_width * self.map_height)
        
        # Draw the wall
        self.draw_wall_on_costmap(costmap, wall)
        
        # Debug: Check if data is properly set after drawing
        non_zero_after_draw = sum(1 for x in costmap.data if x != 0)
        self.get_logger().info(f"AFTER DRAW: Costmap has {non_zero_after_draw} non-zero pixels")
        
        return costmap
    
    def draw_wall_on_costmap(self, costmap: OccupancyGrid, wall: Dict[str, Any]):
        """Draw a wall on the costmap."""
        # Convert wall coordinates to grid coordinates
        wall_x = wall['x']
        wall_y = wall['y']
        wall_width = wall['width']
        wall_length = wall['length']
        
        # Calculate grid coordinates for the wall rectangle  
        # Use the wall's (x, y) as the corner and draw a rectangle from there
        grid_x_start = int((wall_x - self.origin_x) / self.resolution)
        grid_y_start = int((wall_y - self.origin_y) / self.resolution)
        
        # Based on the database schema and extract_walls.py logic:
        # For north_atrium_wall: width=9.525m, length=0.102m  
        # This should draw from (5.817, 0) to (5.817 + 0.102, 0 + 9.525) = (5.919, 9.525)
        # So: length goes in X direction, width goes in Y direction
        grid_width = max(1, int(wall_length / self.resolution))  # X dimension = length = 0.102m
        grid_height = max(1, int(wall_width / self.resolution))  # Y dimension = width = 9.525m
        
        # Log detailed debug information
        self.get_logger().info(f"Wall '{wall['name']}' at world coords ({wall_x:.3f}, {wall_y:.3f})")
        self.get_logger().info(f"Wall dimensions: width={wall_width:.3f}m, length={wall_length:.3f}m")
        self.get_logger().info(f"Using wall position directly (no centering offset)")
        self.get_logger().info(f"Grid start: ({grid_x_start}, {grid_y_start})")
        self.get_logger().info(f"Grid size: width={grid_width} pixels (X=length), height={grid_height} pixels (Y=width)")
        self.get_logger().info(f"This should draw {grid_width} pixels horizontally, {grid_height} pixels vertically")
        
        # Check if wall is within costmap bounds
        if (grid_x_start < 0 or grid_x_start >= self.map_width or 
            grid_y_start < 0 or grid_y_start >= self.map_height):
            self.get_logger().warn(f"Wall '{wall['name']}' is outside costmap bounds!")
            self.get_logger().warn(f"Grid start: ({grid_x_start}, {grid_y_start}), Map size: ({self.map_width}, {self.map_height})")
            return
        
        # Convert costmap.data to a mutable list
        data = list(costmap.data)
        
        # Draw the wall rectangle
        walls_drawn = 0
        for dy in range(grid_height):
            for dx in range(grid_width):
                grid_x = grid_x_start + dx
                grid_y = grid_y_start + dy
                
                # Check bounds
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    index = grid_y * self.map_width + grid_x
                    data[index] = self.costmap_value
                    walls_drawn += 1
        
        # Update costmap data - ensure it's the right type
        costmap.data = data
        
        # Add verification that the data was actually set
        non_zero_count = sum(1 for x in costmap.data if x != 0)
        self.get_logger().info(f"Drew {walls_drawn} pixels with value {self.costmap_value}")
        self.get_logger().info(f"Costmap now has {non_zero_count} non-zero pixels")
        
        # Debug: print first few non-zero values and their indices
        if walls_drawn > 0:
            sample_indices = []
            for dy in range(min(2, grid_height)):
                for dx in range(min(2, grid_width)):
                    grid_x = grid_x_start + dx
                    grid_y = grid_y_start + dy
                    if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                        index = grid_y * self.map_width + grid_x
                        sample_indices.append(f"idx{index}={costmap.data[index]}")
            self.get_logger().info(f"Sample data: {', '.join(sample_indices)}")
            
            # Also check data type
            self.get_logger().info(f"Data type: {type(costmap.data)}, len: {len(costmap.data)}")
            self.get_logger().info(f"First 10 values: {costmap.data[:10]}")
    
    def print_wall_info(self, wall: Dict[str, Any], index: int, total: int):
        """Print information about the current wall."""
        vertical_str = "Yes" if wall['vertical'] else "No"
        print(f"\n--- Wall {index + 1} of {total} ---")
        print(f"ID: {wall['id']}")
        print(f"Name: {wall['name']}")
        print(f"Width: {wall['width']:.3f} m")
        print(f"Length: {wall['length']:.3f} m")
        print(f"Position: ({wall['x']:.3f}, {wall['y']:.3f}) m")
        print(f"Vertical: {vertical_str}")
        
        # Add coordinate transformation info
        grid_x = int((wall['x'] - self.origin_x) / self.resolution)
        grid_y = int((wall['y'] - self.origin_y) / self.resolution)
        grid_w = max(1, int(wall['width'] / self.resolution))
        grid_l = max(1, int(wall['length'] / self.resolution))
        print(f"Grid coords: ({grid_x}, {grid_y}) size {grid_w}x{grid_l} pixels")
        
        print("Commands: [Enter]=Next, [b]=Previous, [s]=Start, [q]=Quit")
        print("Command: ", end="", flush=True)
    
    def check_input(self):
        """Check for keyboard input using a timer."""
        if not self.running:
            return
            
        try:
            # Check if input is available
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                command = sys.stdin.read(1).lower()
                
                if command == 'q':
                    self.get_logger().info("Quitting...")
                    self.cleanup_and_exit()
                elif command == 'b':
                    # Go to previous wall
                    self.current_index = (self.current_index - 1) % len(self.walls)
                    self.display_current_wall()
                elif command == 's':
                    # Go to start (first wall)
                    self.current_index = 0
                    self.display_current_wall()
                elif command == '\n' or command == '\r' or command == ' ':
                    # Go to next wall (Enter, space, or other)
                    self.current_index = (self.current_index + 1) % len(self.walls)
                    self.display_current_wall()
                    
        except Exception as e:
            self.get_logger().error(f"Input handling error: {e}")
            
    def cleanup_and_exit(self):
        """Clean up terminal settings and exit."""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.running = False
        rclpy.shutdown()
    
    def display_current_wall(self):
        """Display the current wall."""
        if not self.walls:
            print("No walls to display")
            return
        
        wall = self.walls[self.current_index]
        
        # Print wall information
        self.print_wall_info(wall, self.current_index, len(self.walls))
        
        # Create and publish costmap
        costmap = self.create_costmap_for_wall(wall)
        
        # Debug: Check costmap data right before publishing
        non_zero_count = sum(1 for x in costmap.data if x != 0)
        self.get_logger().info(f"BEFORE PUBLISH: Costmap has {non_zero_count} non-zero pixels")
        if non_zero_count > 0:
            # Show some non-zero values
            non_zero_indices = [i for i, x in enumerate(costmap.data) if x != 0][:5]
            self.get_logger().info(f"BEFORE PUBLISH: Non-zero at indices: {non_zero_indices}")
            for idx in non_zero_indices[:3]:
                self.get_logger().info(f"BEFORE PUBLISH: costmap.data[{idx}] = {costmap.data[idx]}")
        
        self.costmap_publisher.publish(costmap)
        
        # Debug: Check costmap data right after publishing
        non_zero_count_after = sum(1 for x in costmap.data if x != 0)
        self.get_logger().info(f"AFTER PUBLISH: Costmap has {non_zero_count_after} non-zero pixels")
        
        self.get_logger().info(f"Published costmap for wall: {wall['name']}")
    
    def global_costmap_callback(self, msg: OccupancyGrid):
        """Callback to get global costmap parameters once at startup."""
        if not self.costmap_initialized:
            self.get_logger().info("Received global costmap, extracting dimensions...")
            
            # Extract only the dimensions from the global costmap
            # Keep the resolution and origin from the map yaml file
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.costmap_frame_id = msg.header.frame_id
            
            self.get_logger().info(f"Using costmap dimensions: {self.map_width}x{self.map_height} pixels")
            self.get_logger().info(f"Using map yaml resolution: {self.resolution:.6f}m/pixel")
            self.get_logger().info(f"Using map yaml origin: ({self.origin_x:.3f}, {self.origin_y:.3f})")
            
            # Load walls from database now that we have costmap parameters
            self.walls = self.load_walls_from_database()
            self.costmap_initialized = True
            
            # Unsubscribe from global costmap since we only need it once
            self.destroy_subscription(self.global_costmap_sub)
            
            # Start the visualization if we have walls
            if self.walls:
                self.get_logger().info("Starting wall visualization...")
                self.start_visualization()
            else:
                self.get_logger().error("No walls found in database")
                
    def start_visualization(self):
        """Start the interactive wall visualization."""
        print(f"Wall Visualizer - Loaded {len(self.walls)} walls")
        print("=" * 50)
        
        # Display first wall
        self.display_current_wall()
        
        # Input handling is now done via timer (self.input_timer)
        
    def run(self):
        """Main execution function."""
        self.get_logger().info("Waiting for global costmap to initialize parameters...")
        
        # Keep the node running and wait for global costmap
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        visualizer = WallVisualizer()
        visualizer.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
