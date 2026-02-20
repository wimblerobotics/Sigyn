#!/usr/bin/env python3
"""
Test script to verify the wall database was created correctly.
"""

import sqlite3
import sys
from pathlib import Path


def query_wall_database():
    """Query and display the contents of the wall database."""
    db_path = Path.home() / "sigyn_ws/src/Sigyn/description/models/sigyn_house.db"
    
    if not db_path.exists():
        print(f"Database not found at: {db_path}")
        print("Please run 'ros2 run experiments extract_walls' first.")
        return
    
    try:
        conn = sqlite3.connect(str(db_path))
        cursor = conn.cursor()
        
        # Query all walls
        cursor.execute('SELECT * FROM walls ORDER BY id')
        walls = cursor.fetchall()
        
        if not walls:
            print("No walls found in database")
            return
        
        print(f"Found {len(walls)} walls in database:")
        print("=" * 85)
        print(f"{'ID':<5} {'Name':<20} {'Width(m)':<10} {'Length(m)':<10} {'X(m)':<10} {'Y(m)':<10} {'Vertical':<8}")
        print("=" * 85)
        
        total_length = 0
        vertical_count = 0
        horizontal_count = 0
        
        for wall in walls:
            if len(wall) == 7:  # New format with vertical
                wall_id, name, width, length, x, y, vertical = wall
                vertical_str = "Yes" if vertical else "No"
                if vertical:
                    vertical_count += 1
                else:
                    horizontal_count += 1
                print(f"{wall_id:<5} {name:<20} {width:<10.3f} {length:<10.3f} {x:<10.3f} {y:<10.3f} {vertical_str:<8}")
            else:  # Old format without vertical - handle gracefully
                wall_id, name, width, length, x, y = wall
                vertical_str = "Yes" if length < width else "No"
                print(f"{wall_id:<5} {name:<20} {width:<10.3f} {length:<10.3f} {x:<10.3f} {y:<10.3f} {vertical_str:<8}")
            total_length += length
        
        print("=" * 85)
        print(f"Total wall length: {total_length:.3f} meters")
        if len(walls) > 0 and len(walls[0]) == 7:
            print(f"Vertical walls: {vertical_count}, Horizontal walls: {horizontal_count}")
        
        conn.close()
        
    except Exception as e:
        print(f"Error querying database: {e}")


if __name__ == '__main__':
    query_wall_database()
