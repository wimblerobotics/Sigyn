#!/home/ros/sigyn-venv/bin/python3

import cv2
import depthai as dai
import time
import sys

def main():
    print(f"STARTING TEST 01: RGB Camera")
    print(f"DepthAI version: {dai.__version__}")
    
    # Create pipeline
    pipeline = dai.Pipeline()
    
    # Define source and output
    try:
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        
        xout_rgb.setStreamName("rgb")
        
        # Properties
        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(15)
        
        # Linking
        cam_rgb.preview.link(xout_rgb.input)
        print("Pipeline built successfully.")
    except Exception as e:
        print(f"Error building pipeline: {e}")
        sys.exit(1)
    
    # Connect
    print("Connecting to OAK-D...")
    try:
        with dai.Device(pipeline) as device:
            print(f"Connected to {device.getDeviceName()}")
            print(f"MxId: {device.getMxId()}")
            
            # Output queue
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            print("Running for 30 frames...")
            frames_captured = 0
            start_time = time.time()
            max_frames = 30
            
            while frames_captured < max_frames:
                in_rgb = q_rgb.tryGet()
                
                if in_rgb is not None:
                    frames_captured += 1
                    if frames_captured % 10 == 0:
                        print(f"Received frame {frames_captured}/{max_frames}")
                
                # Timeout safety
                if time.time() - start_time > 10:
                    print("Timeout reached!")
                    break
                
                time.sleep(0.01)

            if frames_captured > 0:
                print("Test 01 (RGB): SUCCESS")
            else:
                print("Test 01 (RGB): FAILED (No frames)")
                sys.exit(1)
            
    except Exception as e:
        print(f"Failed to connect or run pipeline: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
