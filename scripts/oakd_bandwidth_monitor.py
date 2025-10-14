#!/usr/bin/env python3
"""
Monitor OAK-D USB bandwidth and performance metrics.
"""

import depthai as dai
import time

def monitor_bandwidth():
    """Monitor OAK-D device bandwidth and stats."""
    
    # Find OAK-D device by serial number (from your launch file)
    device_info = dai.DeviceInfo("14442C1051B665D700")
    
    try:
        with dai.Device(device_info) as device:
            print("=" * 60)
            print(f"Device: {device.getDeviceName()}")
            print(f"USB Speed: {device.getUsbSpeed().name}")
            print(f"MxID: {device.getMxId()}")
            print("=" * 60)
            
            # Get connection info
            connection = device.getConnection()
            print(f"\nConnection Type: {connection}")
            
            # Monitor bandwidth continuously
            print("\nMonitoring bandwidth (Ctrl+C to stop)...")
            print(f"{'Time':<12} {'USB Speed':<15} {'Queues':<20}")
            print("-" * 60)
            
            while True:
                usb_speed = device.getUsbSpeed()
                queue_events = device.getQueueEvents()
                
                print(f"{time.strftime('%H:%M:%S'):<12} {usb_speed.name:<15} {len(queue_events):<20}", end='\r')
                time.sleep(0.5)
                
    except Exception as e:
        print(f"Error: {e}")
        print("\nTrying to find any available OAK device...")
        
        # Fallback: find any OAK device
        with dai.Device() as device:
            print("=" * 60)
            print(f"Device: {device.getDeviceName()}")
            print(f"USB Speed: {device.getUsbSpeed().name}")
            print(f"Connection: {device.getConnection()}")
            print("=" * 60)

if __name__ == "__main__":
    monitor_bandwidth()
