#!/usr/bin/env python3
import depthai as dai
import cv2
import time
import sys
import numpy as np

print("Test 2: Camera Preview")

try:
    pipeline = dai.Pipeline()

    # Define source
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(15)

    # Define output
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")

    # Link
    cam_rgb.isp.link(xout_rgb.input)

    print("Pipeline created. Connecting to device...")
    with dai.Device(pipeline) as device:
        print("Device connected. Starting pipeline...")
        
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        frames_captured = 0
        start_time = time.time()
        
        while frames_captured < 5:
            # Try to get a frame
            in_rgb = q_rgb.tryGet()
            
            if in_rgb is not None:
                # Retrieve 'bgr' (opencv format) frame
                frame = in_rgb.getCvFrame()
                frames_captured += 1
                
                print(f"Frame {frames_captured}: {frame.shape}, Timestamp: {in_rgb.getTimestamp()}")
                
                # Check frame content (not completely black)
                mean_val = np.mean(frame)
                print(f"  Mean pixel value: {mean_val:.2f}")
            
            if time.time() - start_time > 5.0:
                 print("Timeout waiting for frames")
                 break
            
            time.sleep(0.01)

        if frames_captured == 0:
            print("Failed to capture any frames")
            sys.exit(1)
            
    print("Test 2 Passed: Captured frames successfully")

except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)
