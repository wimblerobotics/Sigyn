#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import subprocess
import re
import time
from typing import Dict, List, Tuple, Optional

def run_ros2_command(cmd: List[str], timeout: int = 15) -> Optional[str]:
    """Run a ROS2 command and return its output."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if result.returncode == 0:
            return result.stdout.strip()
        else:
            print(f"⚠️  Command failed: {' '.join(cmd)}")
            if result.stderr:
                print(f"Error: {result.stderr}")
            return None
    except subprocess.TimeoutExpired:
        print(f"⏰ Command timed out: {' '.join(cmd)}")
        return None
    except Exception as e:
        print(f"❌ Error running command {' '.join(cmd)}: {e}")
        return None

def get_topic_list() -> List[str]:
    """Get list of all ROS2 topics."""
    output = run_ros2_command(['ros2', 'topic', 'list'])
    if output:
        return [topic.strip() for topic in output.split('\n') if topic.strip()]
    return []

def get_topic_info(topic: str) -> Tuple[str, str, List[str], List[str]]:
    """Get topic type, QoS info, publishers, and subscribers for a topic."""
    # Get detailed info with publishers and subscribers from verbose output
    verbose_output = run_ros2_command(['ros2', 'topic', 'info', '-v', topic], timeout=10)
    topic_type = "Unknown"
    publishers = []
    subscribers = []
    qos_info = "N/A"
    
    if verbose_output:
        lines = verbose_output.split('\n')
        current_section = None
        found_qos = False
        
        for i, line in enumerate(lines):
            line = line.strip()
            
            if line.startswith('Type:'):
                topic_type = line.split('Type:', 1)[1].strip()
            elif line.startswith('Publisher count:'):
                current_section = "publishers"
            elif line.startswith('Subscription count:'):
                current_section = "subscribers"
            elif line.startswith('Node name:') and current_section:
                node_name = line.split('Node name:', 1)[1].strip()
                if current_section == "publishers":
                    publishers.append(node_name)
                    # Get QoS from first publisher if we haven't found it yet
                    if not found_qos:
                        qos_info = extract_qos_from_lines(lines, i)
                        if qos_info != "N/A":
                            found_qos = True
                elif current_section == "subscribers":
                    subscribers.append(node_name)
                    # If no publishers, get QoS from first subscriber
                    if not found_qos and len(publishers) == 0:
                        qos_info = extract_qos_from_lines(lines, i)
                        if qos_info != "N/A":
                            found_qos = True
    
    return topic_type, qos_info, publishers, subscribers

def extract_qos_from_lines(lines: List[str], start_index: int) -> str:
    """Extract QoS profile from lines starting at given index."""
    reliability = "Unknown"
    durability = "Unknown"
    
    # Look for QoS profile in the next 20 lines
    for j in range(start_index+1, min(start_index+20, len(lines))):
        line = lines[j].strip()
        if line.startswith('Reliability:'):
            reliability = line.split('Reliability:', 1)[1].strip()
        elif line.startswith('Durability:'):
            durability = line.split('Durability:', 1)[1].strip()
        elif line.startswith('Node name:'):
            # Hit another node, stop looking
            break
    
    if reliability != "Unknown" and durability != "Unknown":
        return f"{reliability} / {durability}"
    return "N/A"

def is_topic_active(topic: str) -> bool:
    """Quick check if a topic is actively publishing."""
    try:
        # Use timeout command for faster detection
        result = subprocess.run(['timeout', '6', 'ros2', 'topic', 'hz', topic], 
                              capture_output=True, text=True)
        
        # If we got any output and it contains rate information, topic is active
        return result.stdout and 'average rate:' in result.stdout
    except Exception:
        return False

def get_topic_hz(topic: str) -> Optional[float]:
    """Get the publishing rate of a topic in Hz."""
    try:
        # Use timeout command to limit execution time and get stable readings
        result = subprocess.run(['timeout', '8', 'ros2', 'topic', 'hz', topic], 
                              capture_output=True, text=True)
        
        if result.stdout:
            output = result.stdout.strip()
            # Look for the last "average rate: X.XXX" in the output (most recent)
            rates = []
            for line in output.split('\n'):
                if 'average rate:' in line:
                    match = re.search(r'average rate:\s*([\d.]+)', line)
                    if match:
                        rates.append(float(match.group(1)))
            
            # Return the last (most recent) rate if any found
            if rates:
                return rates[-1]
    except Exception as e:
        print(f"   ⚠️  Error measuring Hz for {topic}: {e}")
    
    return None

def get_topic_bw(topic: str) -> Optional[float]:
    """Get the bandwidth usage of a topic in bytes/sec."""
    try:
        # Use timeout command to limit execution time and get stable readings
        result = subprocess.run(['timeout', '8', 'ros2', 'topic', 'bw', topic], 
                              capture_output=True, text=True)
        
        if result.stdout:
            output = result.stdout.strip()
            # Look for bandwidth information - parse different formats
            bandwidths = []
            for line in output.split('\n'):
                # Look for patterns like "2.18 KB/s from 41 messages"
                if 'kb/s' in line.lower() or 'mb/s' in line.lower() or 'bytes/s' in line.lower():
                    # Try to extract the number and unit
                    match = re.search(r'([\d.]+)\s*(kb|mb|bytes)/s', line.lower())
                    if match:
                        value = float(match.group(1))
                        unit = match.group(2).lower()
                        
                        # Convert to bytes/s
                        if unit == 'kb':
                            bw = value * 1024
                        elif unit == 'mb':
                            bw = value * 1024 * 1024
                        else:  # bytes
                            bw = value
                        
                        bandwidths.append(bw)
            
            # Return the last (most recent) bandwidth if any found
            if bandwidths:
                return bandwidths[-1]
    except Exception as e:
        print(f"   ⚠️  Error measuring bandwidth for {topic}: {e}")
    
    return None

def format_bandwidth(bw: Optional[float]) -> str:
    """Format bandwidth with appropriate units."""
    if bw is None:
        return "N/A"
    
    if bw < 1024:
        return f"{bw:.1f} B/s"
    elif bw < 1024 * 1024:
        return f"{bw/1024:.1f} KB/s"
    else:
        return f"{bw/(1024*1024):.1f} MB/s"

def analyze_topics():
    """Main function to analyze ROS2 topics and generate report."""
    print("🚀 Starting ROS2 topic analysis...")
    
    # Get all topics
    all_topics = get_topic_list()
    if not all_topics:
        print("❌ No topics found!")
        return
    
    print(f"📋 Found {len(all_topics)} topics")
    
    # Remove /bond from the list as requested
    topics = [t for t in all_topics if t != '/bond']
    print(f"📊 Analyzing {len(topics)} topics (excluding /bond)...")
    
    # Analyze each topic
    topic_data = []
    total_topics = len(topics)
    
    for i, topic in enumerate(topics, 1):
        print(f"📊 Analyzing topic {i}/{total_topics}: {topic}")
        
        # Get topic info (type, QoS, publishers, subscribers)
        topic_type, qos_info, publishers, subscribers = get_topic_info(topic)
        print(f"   Type: {topic_type}")
        print(f"   Publishers: {len(publishers)}, Subscribers: {len(subscribers)}")
        print(f"   QoS: {qos_info}")
        
        # Check if topic is active
        print(f"   Checking if active (6 second timeout)...")
        is_active = is_topic_active(topic)
        
        hz = None
        bw = None
        
        if is_active:
            print(f"   ✅ Active - measuring rate and bandwidth...")
            # Get rate and bandwidth for active topics
            hz = get_topic_hz(topic)
            bw = get_topic_bw(topic)
            print(f"   📏 Rate: {hz} Hz, Bandwidth: {bw} bytes/s")
        else:
            print(f"   ⭕ Inactive - skipping measurements")
        
        topic_data.append({
            'name': topic,
            'type': topic_type,
            'hz': hz,
            'bw': bw,
            'qos': qos_info,
            'publishers': publishers,
            'subscribers': subscribers
        })
    
    # Generate markdown report
    generate_report(topic_data)

def generate_report(topic_data: List[Dict]):
    """Generate a markdown report of the topic analysis."""
    
    # Sort topics by name for better readability
    topic_data.sort(key=lambda x: x['name'])
    
    # Calculate summary statistics
    active_topics = [t for t in topic_data if t['hz'] is not None and t['hz'] > 0]
    total_bandwidth = sum(t['bw'] for t in topic_data if t['bw'] is not None)
    
    # Create markdown report
    report = []
    report.append("# ROS2 Topic Analysis Report")
    report.append("")
    report.append(f"📊 **Total Topics:** {len(topic_data)}")
    report.append(f"🟢 **Active Topics:** {len(active_topics)}")
    report.append(f"📡 **Total Bandwidth:** {format_bandwidth(total_bandwidth)}")
    report.append("")
    
    # Topics table
    report.append("## Topic Details")
    report.append("")
    report.append("| Topic | Type | Rate (Hz) | Bandwidth | QoS First Pub | Publishers | Subscribers |")
    report.append("|-------|------|-----------|-----------|---------------|------------|-------------|")
    
    for topic in topic_data:
        name = topic['name']
        topic_type = topic['type']
        hz_str = f"{topic['hz']:.2f}" if topic['hz'] is not None else "N/A"
        bw_str = format_bandwidth(topic['bw'])
        qos_str = topic['qos']
        pub_str = "<br>".join(topic['publishers']) if topic['publishers'] else "—"
        sub_str = "<br>".join(topic['subscribers']) if topic['subscribers'] else "—"
        
        report.append(f"| `{name}` | `{topic_type}` | {hz_str} | {bw_str} | {qos_str} | {pub_str} | {sub_str} |")
    
    report.append("")
    
    # Active topics summary
    if active_topics:
        report.append("## Active Topics Summary")
        report.append("")
        report.append("| Topic | Rate (Hz) | Bandwidth |")
        report.append("|-------|-----------|-----------|")
        
        # Sort active topics by rate (highest first)
        active_topics.sort(key=lambda x: x['hz'], reverse=True)
        
        for topic in active_topics:
            name = topic['name']
            hz_str = f"{topic['hz']:.2f}"
            bw_str = format_bandwidth(topic['bw'])
            report.append(f"| `{name}` | {hz_str} | {bw_str} |")
        
        report.append("")
    
    # High bandwidth topics
    high_bw_topics = [t for t in topic_data if t['bw'] is not None and t['bw'] > 1000]  # > 1KB/s
    if high_bw_topics:
        report.append("## High Bandwidth Topics (>1KB/s)")
        report.append("")
        
        # Sort by bandwidth (highest first)
        high_bw_topics.sort(key=lambda x: x['bw'], reverse=True)
        
        for topic in high_bw_topics[:10]:  # Top 10
            name = topic['name']
            bw_str = format_bandwidth(topic['bw'])
            report.append(f"- `{name}`: {bw_str}")
        
        report.append("")
    
    # QoS summary
    qos_summary = {}
    for topic in topic_data:
        qos = topic['qos']
        if qos != "N/A":
            qos_summary[qos] = qos_summary.get(qos, 0) + 1
    
    if qos_summary:
        report.append("## QoS Profile Summary")
        report.append("")
        for qos, count in sorted(qos_summary.items(), key=lambda x: x[1], reverse=True):
            report.append(f"- **{qos}**: {count} topics")
        report.append("")
    
    # Write report to file
    report_text = '\n'.join(report)
    
    try:
        with open('/home/ros/sigyn_ws/src/Sigyn/topic_analysis_report.md', 'w') as f:
            f.write(report_text)
        print("📄 Report saved to: topic_analysis_report.md")
    except Exception as e:
        print(f"❌ Error saving report: {e}")
    
    # Also print summary to console
    print("\n" + "="*50)
    print("📊 ANALYSIS SUMMARY")
    print("="*50)
    print(f"Total Topics: {len(topic_data)}")
    print(f"Active Topics: {len(active_topics)}")
    print(f"Total Bandwidth: {format_bandwidth(total_bandwidth)}")
    
    if active_topics:
        print(f"\nTop 5 Most Active Topics:")
        for i, topic in enumerate(active_topics[:5], 1):
            print(f"  {i}. {topic['name']}: {topic['hz']:.2f} Hz")
    
    if high_bw_topics:
        print(f"\nTop 5 Highest Bandwidth Topics:")
        for i, topic in enumerate(high_bw_topics[:5], 1):
            print(f"  {i}. {topic['name']}: {format_bandwidth(topic['bw'])}")

if __name__ == "__main__":
    analyze_topics()
