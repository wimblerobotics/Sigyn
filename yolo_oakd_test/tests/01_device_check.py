#!/usr/bin/env python3
import depthai as dai
import sys

print(f"DepthAI Version: {dai.__version__}")

try:
    # Check for connected devices
    device_infos = dai.Device.getAllAvailableDevices()
    print(f"Found {len(device_infos)} devices")

    for i, info in enumerate(device_infos):
        print(f"Device {i}: {info.getMxId()} ({info.state})")

    if len(device_infos) == 0:
        print("No devices found!")
        sys.exit(1)

    # Attempt to connect to the first device
    # Use an empty pipeline to just open the device
    pipeline = dai.Pipeline()
    try:
        with dai.Device(pipeline) as device:
            print(f"Successfully connected to device: {device.getMxId()}")
            print(f"USB Speed: {device.getUsbSpeed()}")
            
            # Check cameras
            cameras = device.getConnectedCameras()
            print(f"Connected cameras: {cameras}")
            
            # Check features
            features = device.getConnectedCameraFeatures()
            for cam in features:
                print(f"Camera: socket={cam.socket}, sensor={cam.sensorName}, type={cam.supportedTypes}")

    except Exception as e:
        print(f"Failed to connect to device: {e}")
        sys.exit(1)

except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)
