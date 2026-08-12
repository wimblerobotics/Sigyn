#!/usr/bin/env python3
"""Post-mortem analysis of docking debug bag file.

Extracts and analyzes:
- Goals sent to Nav2 (when, where, from where)
- Robot pose over time
- Transform tree evolution
- Velocity commands vs actual motion
- AprilTag detection events

Usage: ./analyze_docking_bag.py <bag_directory>
"""

import sys
import math
from pathlib import Path
from collections import defaultdict
from datetime import datetime

try:
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
    from rosbags.typesys import get_types_from_msg, register_types
except ImportError:
    print("ERROR: rosbags library not installed")
    print("Install with: pip install rosbags")
    sys.exit(1)


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def analyze_bag(bag_path):
    """Analyze a rosbag and extract critical docking debug information."""
    
    print("=" * 80)
    print("DOCKING DEBUG BAG ANALYSIS")
    print("=" * 80)
    print(f"Bag: {bag_path}")
    print()
    
    # Data structures
    goals = []
    poses = []
    vel_commands = []
    apriltag_detections = []
    staging_poses = []
    dock_poses = []
    
    # TF data
    tf_data = defaultdict(list)
    
    with Reader(bag_path) as reader:
        # Print bag info
        print(f"Duration: {reader.duration / 1e9:.2f} seconds")
        print(f"Messages: {sum(c.msgcount for c in reader.connections)}")
        print()
        
        # Get start time
        start_time = reader.start_time / 1e9
        
        print("Processing messages...")
        for connection, timestamp, rawdata in reader.messages():
            msg = deserialize_cdr(rawdata, connection.msgtype)
            rel_time = (timestamp / 1e9) - start_time
            
            # Goal poses
            if connection.topic == '/goal_pose':
                x = msg.pose.position.x
                y = msg.pose.position.y
                yaw = quaternion_to_yaw(
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                )
                goals.append({
                    'time': rel_time,
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'frame': msg.header.frame_id
                })
            
            # Robot poses
            elif connection.topic == '/amcl_pose':
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                yaw = quaternion_to_yaw(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                )
                # Covariance (first 3 diagonal elements: x, y, yaw)
                cov_x = msg.pose.covariance[0]
                cov_y = msg.pose.covariance[7]
                cov_yaw = msg.pose.covariance[35]
                
                poses.append({
                    'time': rel_time,
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'cov_x': cov_x,
                    'cov_y': cov_y,
                    'cov_yaw': cov_yaw
                })
            
            # Velocity commands
            elif connection.topic == '/cmd_vel_nav':
                vel_commands.append({
                    'time': rel_time,
                    'linear': msg.linear.x,
                    'angular': msg.angular.z
                })
            
            # AprilTag detections
            elif connection.topic == '/oakd_apriltag_node/detections':
                if msg.detections:
                    for det in msg.detections:
                        apriltag_detections.append({
                            'time': rel_time,
                            'id': det.results[0].id if det.results else None,
                            'x': det.bbox.center.position.x,
                            'y': det.bbox.center.position.y,
                            'z': det.bbox.center.position.z
                        })
            
            # Staging pose
            elif connection.topic == '/staging_pose':
                x = msg.pose.position.x
                y = msg.pose.position.y
                yaw = quaternion_to_yaw(
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                )
                staging_poses.append({
                    'time': rel_time,
                    'x': x,
                    'y': y,
                    'yaw': yaw
                })
            
            # Detected dock pose
            elif connection.topic == '/detected_dock_pose':
                x = msg.pose.position.x
                y = msg.pose.position.y
                yaw = quaternion_to_yaw(
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                )
                dock_poses.append({
                    'time': rel_time,
                    'x': x,
                    'y': y,
                    'yaw': yaw
                })
    
    # Analysis
    print()
    print("=" * 80)
    print("ANALYSIS RESULTS")
    print("=" * 80)
    print()
    
    # Goal analysis
    print(f"🎯 GOALS SENT: {len(goals)}")
    for i, goal in enumerate(goals):
        print(f"\n   Goal #{i+1} at t={goal['time']:.2f}s:")
        print(f"      Position: ({goal['x']:.4f}, {goal['y']:.4f})")
        print(f"      Yaw: {goal['yaw']:.4f} rad ({math.degrees(goal['yaw']):.2f}°)")
        print(f"      Frame: {goal['frame']}")
        
        # Find robot pose closest to this goal time
        if poses:
            closest_pose = min(poses, key=lambda p: abs(p['time'] - goal['time']))
            dt = abs(closest_pose['time'] - goal['time'])
            if dt < 0.5:  # Within 500ms
                rx = closest_pose['x']
                ry = closest_pose['y']
                ryaw = closest_pose['yaw']
                
                # Calculate relative geometry
                dist = math.sqrt((goal['x'] - rx)**2 + (goal['y'] - ry)**2)
                bearing = math.atan2(goal['y'] - ry, goal['x'] - rx)
                rotation_needed = goal['yaw'] - ryaw
                # Normalize to [-pi, pi]
                while rotation_needed > math.pi:
                    rotation_needed -= 2 * math.pi
                while rotation_needed < -math.pi:
                    rotation_needed += 2 * math.pi
                
                print(f"\n      Robot state at goal time:")
                print(f"         Position: ({rx:.4f}, {ry:.4f})")
                print(f"         Yaw: {ryaw:.4f} rad ({math.degrees(ryaw):.2f}°)")
                print(f"         Uncertainty: σ_x={math.sqrt(closest_pose['cov_x']):.3f}m, σ_y={math.sqrt(closest_pose['cov_y']):.3f}m")
                print(f"\n      Relative geometry:")
                print(f"         Distance: {dist:.4f}m")
                print(f"         Bearing: {bearing:.4f} rad ({math.degrees(bearing):.2f}°)")
                print(f"         Rotation needed: {rotation_needed:.4f} rad ({math.degrees(rotation_needed):.2f}°)")
                
                if abs(math.degrees(rotation_needed)) > 90:
                    print(f"         ❌ WARNING: Goal is BEHIND robot!")
                elif abs(math.degrees(rotation_needed)) > 45:
                    print(f"         ⚠️  WARNING: Large rotation required")
    
    # Staging pose analysis
    print(f"\n🚪 STAGING POSES: {len(staging_poses)}")
    for i, sp in enumerate(staging_poses):
        print(f"   Staging #{i+1} at t={sp['time']:.2f}s:")
        print(f"      Position: ({sp['x']:.4f}, {sp['y']:.4f})")
        print(f"      Yaw: {sp['yaw']:.4f} rad ({math.degrees(sp['yaw']):.2f}°)")
    
    # AprilTag detection analysis
    print(f"\n🎯 APRILTAG DETECTIONS: {len(apriltag_detections)}")
    if apriltag_detections:
        first = apriltag_detections[0]
        last = apriltag_detections[-1]
        print(f"   First detection at t={first['time']:.2f}s")
        print(f"   Last detection at t={last['time']:.2f}s")
        print(f"   Duration: {last['time'] - first['time']:.2f}s")
    else:
        print(f"   ❌ NO APRILTAG DETECTIONS RECORDED")
    
    # Velocity command analysis
    print(f"\n🚗 VELOCITY COMMANDS: {len(vel_commands)}")
    if vel_commands:
        # Find significant rotations
        large_rotations = [v for v in vel_commands if abs(v['angular']) > 0.3]
        if large_rotations:
            print(f"   Large rotation commands (>0.3 rad/s): {len(large_rotations)}")
            first_rot = large_rotations[0]
            print(f"   First large rotation at t={first_rot['time']:.2f}s: {first_rot['angular']:.3f} rad/s")
    
    # Pose trajectory analysis
    print(f"\n📍 ROBOT TRAJECTORY: {len(poses)} poses")
    if poses and len(poses) > 1:
        start = poses[0]
        end = poses[-1]
        print(f"   Start: ({start['x']:.4f}, {start['y']:.4f}, {math.degrees(start['yaw']):.2f}°)")
        print(f"   End: ({end['x']:.4f}, {end['y']:.4f}, {math.degrees(end['yaw']):.2f}°)")
        dist_traveled = math.sqrt((end['x'] - start['x'])**2 + (end['y'] - start['y'])**2)
        print(f"   Distance: {dist_traveled:.4f}m")
        
        # Check for localization jumps
        max_jump = 0
        for i in range(1, len(poses)):
            dt = poses[i]['time'] - poses[i-1]['time']
            if dt < 1.0:  # Only check consecutive poses
                dx = poses[i]['x'] - poses[i-1]['x']
                dy = poses[i]['y'] - poses[i-1]['y']
                jump = math.sqrt(dx**2 + dy**2)
                if jump > max_jump:
                    max_jump = jump
        
        print(f"   Max localization jump: {max_jump:.4f}m")
        if max_jump > 0.5:
            print(f"   ⚠️  WARNING: Large localization jump detected!")
    
    print()
    print("=" * 80)
    print("DIAGNOSIS RECOMMENDATIONS")
    print("=" * 80)
    print()
    
    # Diagnosis
    if not goals:
        print("❌ NO GOALS RECORDED - Navigation was never initiated")
    elif not staging_poses:
        print("❌ NO STAGING POSES - Docking server may not be publishing")
    elif goals and poses:
        # Check first goal
        goal = goals[0]
        closest_pose = min(poses, key=lambda p: abs(p['time'] - goal['time']))
        if abs(closest_pose['time'] - goal['time']) < 0.5:
            dist = math.sqrt((goal['x'] - closest_pose['x'])**2 + (goal['y'] - closest_pose['y'])**2)
            rotation = abs(goal['yaw'] - closest_pose['yaw'])
            
            if dist < 0.5 and rotation < 0.2:
                print("✅ Goal is very close to current position - may be already at staging")
            elif abs(math.degrees(goal['yaw'] - closest_pose['yaw'])) > 90:
                print("❌ DIAGNOSIS: Goal requires >90° rotation (goal is behind robot)")
                print("   → Dock pose yaw in database is likely incorrect")
                print("   → Check docking_stations.yaml pose[2] (yaw value)")
            elif dist > 3.0:
                print("❌ DIAGNOSIS: Goal is very far from robot")
                print("   → Either dock pose is wrong OR robot is delocalized")
                print("   → Check AMCL pose uncertainty (covariance)")
    
    if len(apriltag_detections) == 0 and poses:
        print("⚠️  No AprilTag detections - robot likely never reached staging pose")
    
    print()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: ./analyze_docking_bag.py <bag_directory>")
        sys.exit(1)
    
    bag_path = Path(sys.argv[1])
    if not bag_path.exists():
        print(f"ERROR: Bag path does not exist: {bag_path}")
        sys.exit(1)
    
    analyze_bag(bag_path)
