#!/usr/bin/env python3
"""
Battery Data Histogram Plotter
Creates histograms for voltage and current distributions from BATT2 log data
"""

import os
import json
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from pathlib import Path
import seaborn as sns

# Set style for better-looking plots
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")

def extract_battery_data(file_path):
    """Extract BATT2 data from a single log file"""
    battery_data = []
    
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line_num, line in enumerate(f, 1):
                if 'BATT2:' in line:
                    try:
                        json_start = line.find('BATT2:') + 6
                        json_str = line[json_start:].strip()
                        data = json.loads(json_str)
                        
                        if all(key in data for key in ['location', 'V', 'A']):
                            voltage = float(data['V'])
                            current = float(data['A'])
                            
                            # Filter out readings with very low voltages or zero current (likely sampling errors)
                            # Use a small threshold for voltage to account for floating point precision
                            if voltage > 0.1 and abs(current) > 0.001:
                                battery_data.append({
                                    'location': data['location'],
                                    'voltage': voltage,
                                    'current': current
                                })
                    except (json.JSONDecodeError, ValueError, KeyError):
                        continue
    except Exception as e:
        print(f"Error reading file {file_path}: {e}")
    
    return battery_data

def load_all_battery_data(base_dir):
    """Load all battery data from log files"""
    all_data = []
    base_path = Path(base_dir)
    
    log_files = list(base_path.rglob('*.TXT'))
    log_files.extend(list(base_path.rglob('*.txt')))
    
    print(f"Loading data from {len(log_files)} log files...")
    
    for log_file in log_files:
        data = extract_battery_data(log_file)
        all_data.extend(data)
    
    # Group data by location
    location_data = defaultdict(list)
    for reading in all_data:
        location_data[reading['location']].append(reading)
    
    return location_data

def create_battery_histograms(location_data, save_dir):
    """Create side-by-side histograms for each battery device (voltage left, current right)"""
    locations = sorted(location_data.keys())
    n_locations = len(locations)
    
    print(f"Creating individual plots for {n_locations} devices...")
    
    # Create a separate figure for each battery device
    for i, location in enumerate(locations):
        print(f"  Processing {location} ({i+1}/{n_locations})...")
        readings = location_data[location]
        voltages = [r['voltage'] for r in readings]
        currents = [r['current'] for r in readings]
        
        # Create figure with 2 subplots side by side
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        
        # Choose a consistent color for each device
        color = plt.cm.Set3(i / max(1, n_locations - 1))
        
        # Voltage histogram (left)
        ax1.hist(voltages, bins=50, alpha=0.7, color=color, edgecolor='black', linewidth=0.5)
        ax1.set_title(f'{location} - Voltage Distribution', fontweight='bold', fontsize=14)
        ax1.set_xlabel('Voltage (V)', fontsize=12)
        ax1.set_ylabel('Frequency', fontsize=12)
        ax1.grid(True, alpha=0.3)
        
        # Voltage statistics
        mean_v = np.mean(voltages)
        std_v = np.std(voltages)
        min_v = np.min(voltages)
        max_v = np.max(voltages)
        
        v_stats_text = f'Mean: {mean_v:.2f}V\nStd: {std_v:.2f}V\nMin: {min_v:.2f}V\nMax: {max_v:.2f}V\nSamples: {len(voltages):,}'
        ax1.text(0.95, 0.95, v_stats_text, transform=ax1.transAxes, 
                 verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))
        
        # Current histogram (right)
        ax2.hist(currents, bins=50, alpha=0.7, color=color, edgecolor='black', linewidth=0.5)
        ax2.set_title(f'{location} - Current Distribution', fontweight='bold', fontsize=14)
        ax2.set_xlabel('Current (A)', fontsize=12)
        ax2.set_ylabel('Frequency', fontsize=12)
        ax2.grid(True, alpha=0.3)
        
        # Current statistics
        mean_a = np.mean(currents)
        std_a = np.std(currents)
        min_a = np.min(currents)
        max_a = np.max(currents)
        
        a_stats_text = f'Mean: {mean_a:.2f}A\nStd: {std_a:.2f}A\nMin: {min_a:.2f}A\nMax: {max_a:.2f}A\nSamples: {len(currents):,}'
        ax2.text(0.95, 0.95, a_stats_text, transform=ax2.transAxes, 
                 verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))
        
        # Adjust layout and save
        plt.tight_layout()
        
        # Save individual device plot
        device_file = os.path.join(save_dir, f'{location}_histograms.png')
        plt.savefig(device_file, dpi=300, bbox_inches='tight')
        print(f"    {location} histograms saved to: {device_file}")
        
        # Close the figure to free memory
        plt.close(fig)

def create_combined_overview(location_data, save_dir):
    """Create a single overview plot with all devices in a grid layout"""
    locations = sorted(location_data.keys())
    n_locations = len(locations)
    
    # Create subplots - 2 columns (voltage, current) x n_locations rows
    fig, axes = plt.subplots(n_locations, 2, figsize=(16, 5*n_locations))
    if n_locations == 1:
        axes = axes.reshape(1, -1)
    
    # Add main title with more padding
    fig.suptitle('Battery Data Distribution Overview - All Devices', fontsize=18, fontweight='bold', y=0.98)
    
    colors = plt.cm.Set3(np.linspace(0, 1, n_locations))
    
    for i, location in enumerate(locations):
        readings = location_data[location]
        voltages = [r['voltage'] for r in readings]
        currents = [r['current'] for r in readings]
        
        # Voltage histogram
        axes[i, 0].hist(voltages, bins=50, alpha=0.7, color=colors[i], edgecolor='black', linewidth=0.5)
        axes[i, 0].set_title(f'{location} - Voltage', fontweight='bold', fontsize=12, pad=10)
        axes[i, 0].set_xlabel('Voltage (V)', fontsize=10)
        axes[i, 0].set_ylabel('Frequency', fontsize=10)
        axes[i, 0].grid(True, alpha=0.3)
        
        # Current histogram
        axes[i, 1].hist(currents, bins=50, alpha=0.7, color=colors[i], edgecolor='black', linewidth=0.5)
        axes[i, 1].set_title(f'{location} - Current', fontweight='bold', fontsize=12, pad=10)
        axes[i, 1].set_xlabel('Current (A)', fontsize=10)
        axes[i, 1].set_ylabel('Frequency', fontsize=10)
        axes[i, 1].grid(True, alpha=0.3)
        
        # Add compact statistics
        v_stats = f'μ={np.mean(voltages):.2f}V, σ={np.std(voltages):.2f}V'
        a_stats = f'μ={np.mean(currents):.2f}A, σ={np.std(currents):.2f}A'
        
        axes[i, 0].text(0.98, 0.95, v_stats, transform=axes[i, 0].transAxes, 
                        verticalalignment='top', horizontalalignment='right', fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))
        axes[i, 1].text(0.98, 0.95, a_stats, transform=axes[i, 1].transAxes, 
                        verticalalignment='top', horizontalalignment='right', fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))
    
    # Adjust layout with more spacing
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    overview_file = os.path.join(save_dir, 'battery_data_overview.png')
    plt.savefig(overview_file, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Combined overview saved to: {overview_file}")

def create_correlation_plot(location_data, save_dir):
    """Create scatter plots showing voltage vs current correlations"""
    locations = sorted(location_data.keys())
    n_locations = len(locations)
    
    # Calculate subplot layout
    cols = min(3, n_locations)
    rows = (n_locations + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=(5*cols, 5*rows))
    
    # Handle different subplot configurations
    if rows == 1 and cols == 1:
        axes = [axes]
    elif rows == 1:
        axes = axes.reshape(1, -1)
    elif cols == 1:
        axes = axes.reshape(-1, 1)
    
    # Add main title with proper spacing
    fig.suptitle('Voltage vs Current Correlation by Device', fontsize=16, fontweight='bold', y=0.98)
    
    colors = plt.cm.Set3(np.linspace(0, 1, n_locations))
    
    for i, location in enumerate(locations):
        row = i // cols
        col = i % cols
        
        readings = location_data[location]
        voltages = [r['voltage'] for r in readings]
        currents = [r['current'] for r in readings]
        
        # Create scatter plot with reduced point density for better visualization
        sample_size = min(5000, len(voltages))  # Sample for performance
        indices = np.random.choice(len(voltages), sample_size, replace=False)
        v_sample = [voltages[j] for j in indices]
        a_sample = [currents[j] for j in indices]
        
        ax = axes[row, col] if rows > 1 else axes[col]
        ax.scatter(v_sample, a_sample, alpha=0.3, s=1, color=colors[i])
        ax.set_title(f'{location}', fontweight='bold', fontsize=12, pad=10)
        ax.set_xlabel('Voltage (V)', fontsize=10)
        ax.set_ylabel('Current (A)', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        # Calculate correlation
        correlation = np.corrcoef(voltages, currents)[0, 1]
        ax.text(0.05, 0.95, f'r = {correlation:.3f}', 
               transform=ax.transAxes, fontsize=10,
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.9, edgecolor='gray'))
    
    # Hide unused subplots
    for i in range(n_locations, rows * cols):
        row = i // cols
        col = i % cols
        ax = axes[row, col] if rows > 1 else axes[col]
        ax.set_visible(False)
    
    # Adjust layout with proper spacing
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    correlation_file = os.path.join(save_dir, 'voltage_current_correlation.png')
    plt.savefig(correlation_file, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Correlation plots saved to: {correlation_file}")

def main():
    # Setup
    base_directory = os.path.dirname(os.path.abspath(__file__))
    save_directory = os.path.join(base_directory, 'plots')
    os.makedirs(save_directory, exist_ok=True)
    
    # Load data
    print("Loading battery data...")
    location_data = load_all_battery_data(base_directory)
    
    if not location_data:
        print("No battery data found!")
        return
    
    print(f"Loaded data for {len(location_data)} locations")
    for location, readings in location_data.items():
        print(f"  {location}: {len(readings):,} readings")
    
    # Create plots
    print("\nGenerating individual device histograms...")
    create_battery_histograms(location_data, save_directory)
    
    print(f"\nAll individual device plots saved to: {save_directory}")
    print("\nGenerated files:")
    for location in sorted(location_data.keys()):
        plot_file = os.path.join(save_directory, f'{location}_histograms.png')
        if os.path.exists(plot_file):
            print(f"  - {location}_histograms.png")
    
    # Optionally create overview plots
    print("\nGenerating overview plots...")
    create_combined_overview(location_data, save_directory)
    create_correlation_plot(location_data, save_directory)

if __name__ == "__main__":
    main()
4770,1772