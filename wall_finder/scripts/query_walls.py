#!/usr/bin/env python3
"""
Script to query and display wall data from the SQLite database.
"""

import sqlite3
import sys
import os

def main():
    db_path = "walls.db"
    
    # Check if database file exists
    if not os.path.exists(db_path):
        print(f"Database file '{db_path}' not found!")
        print("Make sure to run the wall_finder node first.")
        sys.exit(1)
    
    # Connect to database
    try:
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # Query all walls
        cursor.execute("SELECT * FROM walls ORDER BY id")
        walls = cursor.fetchall()
        
        if not walls:
            print("No walls found in database.")
            return
        
        print(f"Found {len(walls)} wall segments:")
        print("-" * 100)
        print(f"{'ID':<3} {'Start X':<8} {'Start Y':<8} {'End X':<8} {'End Y':<8} {'Length':<8} {'Angle':<8} {'Room':<10} {'Wall Name':<10}")
        print("-" * 100)
        
        for wall in walls:
            wall_id, start_x, start_y, end_x, end_y, length, angle, room_name, wall_name = wall
            print(f"{wall_id:<3} {start_x:<8.3f} {start_y:<8.3f} {end_x:<8.3f} {end_y:<8.3f} {length:<8.3f} {angle:<8.3f} {room_name:<10} {wall_name:<10}")
        
        print("-" * 100)
        print(f"Total walls: {len(walls)}")
        
        # Calculate statistics
        total_length = sum(wall[5] for wall in walls)
        avg_length = total_length / len(walls) if walls else 0
        
        print(f"Total wall length: {total_length:.3f} meters")
        print(f"Average wall length: {avg_length:.3f} meters")
        
    except sqlite3.Error as e:
        print(f"Database error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if conn:
            conn.close()

if __name__ == "__main__":
    main()
