#!/home/ros/sigyn-venv/bin/python3

import cv2
import depthai as dai
import time
import sys
import numpy as np

def main():
    print(f"STARTING TEST 02: Stereo Depth")
    print(f"DepthAI version: {dai.__version__}")
    
    pipeline = dai.Pipeline()
    
    try:
        # Mono Cameras
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        
        xoutDepth.setStreamName("depth")
        
        # Properties
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        
        # Stereo Config
        # Using HIGH_ACCURACY as verified available in 2.28.0.0
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        
        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        stereo.depth.link(xoutDepth.input)
        
        print("Pipeline built successfully.")
    except Exception as e:
        print(f"Error building pipeline: {e}")
        sys.exit(1)

    # Connect
    try:
        with dai.Device(pipeline) as device:
            print(f"Connected to {device.getDeviceName()}")
            
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            
            print("Running for 30 frames...")
            frames_captured = 0
            start_time = time.time()
            max_frames = 30
            
            while frames_captured < max_frames:
                in_depth = q_depth.tryGet()
                
                if in_depth is not None:
                    frames_captured += 1
                    
                    frame = in_depth.getFrame()
                    # Check if we have valid data (not all zeros)
                    valid_pixels = np.count_nonzero(frame)
                    total_pixels = frame.size
                    
                    if frames_captured % 10 == 0:
                        print(f"Received frame {frames_captured}/{max_frames} - Non-zero pixels: {100.0 * valid_pixels / total_pixels:.1f}%")
                
                if time.time() - start_time > 10:
                    print("Timeout reached!")
                    break
                
                time.sleep(0.01)

            if frames_captured > 0:
                print("Test 02 (Stereo): SUCCESS")
            else:
                print("Test 02 (Stereo): FAILED")
                sys.exit(1)
            
    except Exception as e:
        print(f"Failed to connect or run pipeline: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
