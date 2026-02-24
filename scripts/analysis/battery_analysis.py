#!/usr/bin/env python3
"""
Battery Data Analysis Script
Analyzes BATT2 JSON messages from log files to generate voltage and current statistics
"""

import os
import json
import re
from collections import defaultdict
from pathlib import Path

def extract_battery_data(file_path):
    """Extract BATT2 data from a single log file"""
    battery_data = []
    
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line_num, line in enumerate(f, 1):
                # Look for BATT2 lines with JSON data
                if 'BATT2:' in line:
                    try:
                        # Extract JSON part after "BATT2:"
                        json_start = line.find('BATT2:') + 6
                        json_str = line[json_start:].strip()
                        
                        # Parse JSON
                        data = json.loads(json_str)
                        
                        # Validate required fields
                        if all(key in data for key in ['location', 'V', 'A']):
                            voltage = float(data['V'])
                            current = float(data['A'])
                            
                            # Filter out readings with very low voltages or near-zero current (likely sampling errors)
                            # Use small thresholds to account for floating point precision
                            if voltage > 0.1 and abs(current) > 0.001:
                                battery_data.append({
                                    'file': file_path,
                                    'line': line_num,
                                    'location': data['location'],
                                    'voltage': voltage,
                                    'current': current,
                                    'idx': data.get('idx', 'unknown'),
                                    'state': data.get('state', 'unknown'),
                                    'charge': data.get('charge', 'unknown')
                                })
                    except (json.JSONDecodeError, ValueError, KeyError) as e:
                        print(f"Warning: Failed to parse BATT2 data in {file_path}:{line_num}: {e}")
                        continue
    except Exception as e:
        print(f"Error reading file {file_path}: {e}")
    
    return battery_data

def analyze_battery_data(base_dir):
    """Analyze all battery data from log files in the directory and subdirectories"""
    all_data = []
    total_readings = 0
    filtered_readings = 0
    base_path = Path(base_dir)
    
    # Find all .TXT files in the directory and subdirectories
    log_files = list(base_path.rglob('*.TXT'))
    log_files.extend(list(base_path.rglob('*.txt')))
    
    print(f"Found {len(log_files)} log files to analyze...")
    
    # Process each log file
    for log_file in log_files:
        print(f"Processing: {log_file}")
        data = extract_battery_data(log_file)
        all_data.extend(data)
    
    print(f"Extracted {len(all_data)} valid battery readings (filtered out readings with 0V or 0A)")
    
    # Group data by location
    location_data = defaultdict(list)
    for reading in all_data:
        location_data[reading['location']].append(reading)
    
    return location_data

def generate_statistics(readings):
    """Generate min, max, average statistics for voltage and current"""
    if not readings:
        return None
    
    voltages = [r['voltage'] for r in readings]
    currents = [r['current'] for r in readings]
    
    return {
        'count': len(readings),
        'voltage': {
            'min': min(voltages),
            'max': max(voltages),
            'avg': sum(voltages) / len(voltages)
        },
        'current': {
            'min': min(currents),
            'max': max(currents),
            'avg': sum(currents) / len(currents)
        }
    }

def print_report(location_data):
    """Print a formatted report of battery statistics"""
    print("\n" + "="*80)
    print("BATTERY DATA ANALYSIS REPORT")
    print("="*80)
    
    # Sort locations for consistent output
    sorted_locations = sorted(location_data.keys())
    
    for location in sorted_locations:
        readings = location_data[location]
        stats = generate_statistics(readings)
        
        if stats:
            print(f"\nLocation: {location}")
            print(f"  Sample Count: {stats['count']}")
            print(f"  Voltage (V):")
            print(f"    Minimum:  {stats['voltage']['min']:8.2f} V")
            print(f"    Maximum:  {stats['voltage']['max']:8.2f} V")
            print(f"    Average:  {stats['voltage']['avg']:8.2f} V")
            print(f"  Current (A):")
            print(f"    Minimum:  {stats['current']['min']:8.2f} A")
            print(f"    Maximum:  {stats['current']['max']:8.2f} A")
            print(f"    Average:  {stats['current']['avg']:8.2f} A")
    
    print("\n" + "="*80)
    print("SUMMARY")
    print("="*80)
    print(f"Total locations analyzed: {len(sorted_locations)}")
    print(f"Total readings processed: {sum(len(readings) for readings in location_data.values())}")
    
    # Overall summary table
    print(f"\n{'Location':<12} {'Samples':<8} {'V_Min':<8} {'V_Max':<8} {'V_Avg':<8} {'A_Min':<8} {'A_Max':<8} {'A_Avg':<8}")
    print("-" * 80)
    
    for location in sorted_locations:
        readings = location_data[location]
        stats = generate_statistics(readings)
        if stats:
            print(f"{location:<12} {stats['count']:<8} "
                  f"{stats['voltage']['min']:<8.2f} {stats['voltage']['max']:<8.2f} {stats['voltage']['avg']:<8.2f} "
                  f"{stats['current']['min']:<8.2f} {stats['current']['max']:<8.2f} {stats['current']['avg']:<8.2f}")

def save_csv_report(location_data, output_file):
    """Save detailed report to CSV file"""
    import csv
    
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write header
        writer.writerow(['Location', 'Sample_Count', 'Voltage_Min', 'Voltage_Max', 'Voltage_Avg', 
                        'Current_Min', 'Current_Max', 'Current_Avg'])
        
        # Write data for each location
        sorted_locations = sorted(location_data.keys())
        for location in sorted_locations:
            readings = location_data[location]
            stats = generate_statistics(readings)
            if stats:
                writer.writerow([
                    location,
                    stats['count'],
                    f"{stats['voltage']['min']:.2f}",
                    f"{stats['voltage']['max']:.2f}",
                    f"{stats['voltage']['avg']:.2f}",
                    f"{stats['current']['min']:.2f}",
                    f"{stats['current']['max']:.2f}",
                    f"{stats['current']['avg']:.2f}"
                ])

def main():
    # Analyze battery data from current directory
    base_directory = os.path.dirname(os.path.abspath(__file__))
    location_data = analyze_battery_data(base_directory)
    
    if not location_data:
        print("No battery data found in log files!")
        return
    
    # Print the report
    print_report(location_data)
    
    # Save CSV report
    csv_file = os.path.join(base_directory, 'battery_analysis_report.csv')
    save_csv_report(location_data, csv_file)
    print(f"\nDetailed report saved to: {csv_file}")

if __name__ == "__main__":
    main()
