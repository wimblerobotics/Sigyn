#!/usr/bin/env python3
"""
Debug wrapper for perimeter_roamer_v2 node.
This script enables remote debugging by starting a debug server before launching the node.
"""

import debugpy
import sys
import os

def main():
    # Configure debugpy for remote debugging
    print("Starting debug server on port 5678...")
    debugpy.listen(("0.0.0.0", 5678))
    
    # Check for wait-for-debugger argument and remove it from sys.argv
    # so it doesn't get passed to the ROS node
    wait_for_debugger = "--wait-for-debugger" in sys.argv
    if wait_for_debugger:
        sys.argv.remove("--wait-for-debugger")
        print("Waiting for debugger to attach...")
        debugpy.wait_for_client()
        print("Debugger attached!")
    
    # Add the package to Python path
    package_path = os.path.dirname(os.path.abspath(__file__))
    if package_path not in sys.path:
        sys.path.insert(0, package_path)
    
    # Import and run the actual node
    try:
        from perimeter_roamer_v2.perimeter_roamer import main as node_main
        print("Starting perimeter_roamer_v2 node with debugging enabled...")
        print(f"ROS arguments being passed: {sys.argv[1:]}")
        node_main()
    except ImportError:
        # Fallback: try to import from current directory
        from perimeter_roamer import main as node_main
        print("Starting perimeter_roamer_v2 node with debugging enabled...")
        print(f"ROS arguments being passed: {sys.argv[1:]}")
        node_main()

if __name__ == "__main__":
    main()
