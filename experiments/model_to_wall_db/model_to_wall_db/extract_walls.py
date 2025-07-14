#!/usr/bin/env python3
"""
Extract wall parameters from xacro files and create SQLite database.

This script reads an xacro file containing xacro:wall macro calls,
processes it with xacro to expand the macros, and extracts wall parameters
from the expanded XML to store in an SQLite database.
"""

import os
import sqlite3
import sys
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Dict, Any
import subprocess

import rclpy
from rclpy.node import Node


class WallExtractor(Node):
    """Extract wall parameters from xacro files and create SQLite database."""
    
    def __init__(self):
        super().__init__('wall_extractor')
        self.get_logger().info("Wall Extractor node starting...")
        
        # Define paths
        self.xacro_file_path = Path.home() / "sigyn_ws/src/Sigyn/description/models/home/model.sdf.xacro"
        self.database_path = Path.home() / "sigyn_ws/src/Sigyn/description/models/sigyn_house.db"
        
        # Conversion factor from inches to meters
        self.INCHES_TO_METERS = 0.0254
        
    def parse_xacro_file(self, file_path: Path) -> List[Dict[str, Any]]:
        """
        Parse xacro file and extract wall parameters using xacro processor.
        
        Args:
            file_path: Path to the xacro file
            
        Returns:
            List of dictionaries containing wall parameters
        """
        walls = []
        
        try:
            # Change to the directory containing the xacro file to handle includes
            original_dir = os.getcwd()
            xacro_dir = file_path.parent
            os.chdir(xacro_dir)
            
            # Use xacro processor to expand the file
            result = subprocess.run(
                ['xacro', file_path.name],
                cwd=xacro_dir,
                capture_output=True,
                text=True
            )
            
            # Restore original directory
            os.chdir(original_dir)
            
            if result.returncode != 0:
                self.get_logger().error(f"Xacro processing failed: {result.stderr}")
                return []
            
            expanded_content = result.stdout
            
            # Parse the expanded XML to extract wall information
            walls = self._parse_expanded_xml(expanded_content)
            
        except Exception as e:
            self.get_logger().error(f"Error parsing xacro file: {e}")
            # Restore original directory in case of error
            try:
                os.chdir(original_dir)
            except:
                pass
            return []
            
        return walls
    
    def _parse_expanded_xml(self, xml_content: str) -> List[Dict[str, Any]]:
        """
        Parse expanded XML content to extract wall information.
        
        Args:
            xml_content: The expanded XML content from xacro processor
            
        Returns:
            List of wall dictionaries
        """
        walls = []
        
        try:
            # Parse the XML
            root = ET.fromstring(xml_content)
            
            # Find all link elements (walls become links in the expanded XML)
            for link in root.findall('.//link'):
                wall_info = self._extract_wall_from_link(link)
                if wall_info:
                    walls.append(wall_info)
                    
        except Exception as e:
            self.get_logger().error(f"Error parsing expanded XML: {e}")
            
        return walls
    
    def _extract_wall_from_link(self, link_element) -> Dict[str, Any]:
        """
        Extract wall information from a link element.
        
        Args:
            link_element: XML element representing a wall link
            
        Returns:
            Dictionary with wall parameters or None if not a wall
        """
        try:
            # Get the name from the link
            name = link_element.get('name')
            if not name:
                return None
            
            # Find the pose element
            pose_elem = link_element.find('pose')
            if pose_elem is None:
                return None
            
            # Parse pose: "x y z roll pitch yaw"
            pose_values = pose_elem.text.strip().split()
            if len(pose_values) < 6:
                return None
                
            x_pos = float(pose_values[0])
            y_pos = float(pose_values[1])
            z_pos = float(pose_values[2])
            roll = float(pose_values[3])
            pitch = float(pose_values[4])
            yaw = float(pose_values[5])  # This is the z-rotation
            
            # Find the geometry box element to get dimensions
            box_elem = link_element.find('.//geometry/box')
            if box_elem is None:
                return None
            
            size_elem = box_elem.find('size')
            if size_elem is None:
                return None
            
            # Parse size: "length width height"
            size_values = size_elem.text.strip().split()
            if len(size_values) < 3:
                return None
                
            length = float(size_values[0])
            width = float(size_values[1])
            height = float(size_values[2])
            
            # Calculate the starting position from the center position
            # The pose gives us the center of the wall, we need the corner
            x_start = x_pos - (length / 2.0)
            y_start = y_pos - (width / 2.0)
            
            return {
                'name': name,
                'width': width,
                'length': length,
                'x': x_start,
                'y': y_start,
                'rotation': yaw  # z-rotation from pose
            }
            
        except Exception as e:
            self.get_logger().error(f"Error extracting wall from link: {e}")
            return None
    
    def create_database(self, walls: List[Dict[str, Any]]) -> bool:
        """
        Create SQLite database and insert wall data.
        
        Args:
            walls: List of wall parameter dictionaries
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Ensure the directory exists
            self.database_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Connect to SQLite database
            conn = sqlite3.connect(str(self.database_path))
            cursor = conn.cursor()
            
            # Create table if it doesn't exist
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS walls (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    width REAL NOT NULL,
                    length REAL NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    rotation REAL NOT NULL DEFAULT 0.0
                )
            ''')
            
            # Clear existing data
            cursor.execute('DELETE FROM walls')
            
            # Insert wall data
            for wall in walls:
                cursor.execute('''
                    INSERT INTO walls (name, width, length, x, y, rotation)
                    VALUES (?, ?, ?, ?, ?, ?)
                ''', (
                    wall['name'],
                    wall['width'],
                    wall['length'],
                    wall['x'],
                    wall['y'],
                    wall['rotation']
                ))
            
            conn.commit()
            conn.close()
            
            self.get_logger().info(f"Successfully created database with {len(walls)} walls at {self.database_path}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error creating database: {e}")
            return False
    
    def print_wall_summary(self, walls: List[Dict[str, Any]]) -> None:
        """
        Print a summary of extracted walls.
        
        Args:
            walls: List of wall parameter dictionaries
        """
        self.get_logger().info(f"Extracted {len(walls)} walls:")
        self.get_logger().info("=" * 90)
        self.get_logger().info(f"{'Name':<20} {'Width(m)':<10} {'Length(m)':<10} {'X(m)':<10} {'Y(m)':<10} {'Rotation(rad)':<12}")
        self.get_logger().info("=" * 90)
        
        for wall in walls:
            self.get_logger().info(
                f"{wall['name']:<20} "
                f"{wall['width']:<10.3f} "
                f"{wall['length']:<10.3f} "
                f"{wall['x']:<10.3f} "
                f"{wall['y']:<10.3f} "
                f"{wall['rotation']:<12.3f}"
            )
    
    def run(self) -> None:
        """Main execution function."""
        self.get_logger().info(f"Looking for xacro file at: {self.xacro_file_path}")
        
        if not self.xacro_file_path.exists():
            self.get_logger().error(f"Xacro file not found: {self.xacro_file_path}")
            return
        
        # Parse the xacro file
        walls = self.parse_xacro_file(self.xacro_file_path)
        
        if not walls:
            self.get_logger().error("No walls found or error parsing file")
            return
        
        # Print summary
        self.print_wall_summary(walls)
        
        # Create database
        if self.create_database(walls):
            self.get_logger().info("Wall extraction completed successfully!")
        else:
            self.get_logger().error("Failed to create database")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        extractor = WallExtractor()
        extractor.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
