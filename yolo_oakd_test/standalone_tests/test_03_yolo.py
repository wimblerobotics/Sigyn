#!/home/ros/sigyn-venv/bin/python3

import cv2
import depthai as dai
import time
import sys
import numpy as np

def main():
    print(f"STARTING TEST 03: YOLO Detection (No Depth)")
    print(f"DepthAI version: {dai.__version__}")
    
    BLOB_PATH = "/home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/can_detector.blob"
    
    pipeline = dai.Pipeline()
    
    try:
        # Camera
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setPreviewSize(640, 640)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(15)
        
        # YOLO Config
        print(f"Loading blob from: {BLOB_PATH}")
        yoloDet = pipeline.create(dai.node.YoloDetectionNetwork)
        yoloDet.setBlobPath(BLOB_PATH)
        yoloDet.setConfidenceThreshold(0.4)
        
        # Yolo Specifics (matching typical YOLOv5 640 params)
        yoloDet.setNumClasses(1)
        yoloDet.setCoordinateSize(4)
        yoloDet.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326])
        yoloDet.setAnchorMasks({"side80": [0, 1, 2], "side40": [3, 4, 5], "side20": [6, 7, 8]})
        yoloDet.setIouThreshold(0.5)
        
        # Outputs
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        xoutDet = pipeline.create(dai.node.XLinkOut)
        xoutDet.setStreamName("nn")
        
        # Linking
        camRgb.preview.link(yoloDet.input)
        
        # Passthrough to see what the NN inputs
        # yoloDet.passthrough.link(xoutRgb.input) 
        # Using camera preview directly for visualization to be safe
        camRgb.preview.link(xoutRgb.input)
        
        yoloDet.out.link(xoutDet.input)
        
        print("Pipeline built successfully.")
    except Exception as e:
        print(f"Error building pipeline: {e}")
        sys.exit(1)

    # Connect
    try:
        with dai.Device(pipeline) as device:
            print(f"Connected to {device.getDeviceName()}")
            
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_det = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            
            print("Running for 60 frames (approx 4 secs)...")
            frames_captured = 0
            start_time = time.time()
            max_frames = 60
            
            detections_found = 0
            
            while frames_captured < max_frames:
                in_rgb = q_rgb.tryGet()
                in_det = q_det.tryGet()
                
                detections = []
                if in_det is not None:
                    detections = in_det.detections
                
                if in_rgb is not None:
                    frames_captured += 1
                    
                    frame = in_rgb.getCvFrame()
                    
                    if len(detections) > 0:
                        detections_found += 1
                        for detection in detections:
                                # Normalize bbox to frame
                                h, w = frame.shape[:2]
                                x1 = int(detection.xmin * w)
                                y1 = int(detection.ymin * h)
                                x2 = int(detection.xmax * w)
                                y2 = int(detection.ymax * h)
                                
                                cv2.putText(frame, f"Can: {detection.confidence:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                
                                if frames_captured % 10 == 0:
                                    print(f"Frame {frames_captured}: Found detection! Conf={detection.confidence:.2f}")

                    # Save one frame with detections if we find any
                    if len(detections) > 0 and detections_found == 1:
                         cv2.imwrite("test_03_detection.jpg", frame)
                         print("Saved test_03_detection.jpg")

                if time.time() - start_time > 15:
                    print("Timeout reached!")
                    break
                
                time.sleep(0.005)

            if frames_captured > 0:
                print(f"Test 03 (YOLO): SUCCESS - Processed {frames_captured} frames, Found {detections_found} frames with detections")
            else:
                print("Test 03 (YOLO): FAILED (No frames)")
                sys.exit(1)
            
    except Exception as e:
        print(f"Failed to connect or run pipeline: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
