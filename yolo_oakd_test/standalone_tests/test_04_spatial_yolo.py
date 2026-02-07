#!/home/ros/sigyn-venv/bin/python3

import cv2
import depthai as dai
import time
import sys
import numpy as np

def main():
    print(f"STARTING TEST 04: Spatial YOLO")
    print(f"DepthAI version: {dai.__version__}")
    
    BLOB_PATH = "/home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/can_detector.blob"
    
    pipeline = dai.Pipeline()
    
    try:
        # 1. RGB Camera
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setPreviewSize(640, 640)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(15)
        
        # 2. Mono Cameras
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        
        monoRight = pipeline.create(dai.node.MonoCamera)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        
        # 3. Stereo Depth
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setSubpixel(True)
        
        # 4. Spatial YOLO
        spatialDet = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        spatialDet.setBlobPath(BLOB_PATH)
        spatialDet.setConfidenceThreshold(0.4)
        spatialDet.input.setBlocking(False)
        spatialDet.setBoundingBoxScaleFactor(0.5)
        spatialDet.setDepthLowerThreshold(100)
        spatialDet.setDepthUpperThreshold(5000)
        
        # YOLO Params
        spatialDet.setNumClasses(1)
        spatialDet.setCoordinateSize(4)
        spatialDet.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326])
        spatialDet.setAnchorMasks({"side80": [0, 1, 2], "side40": [3, 4, 5], "side20": [6, 7, 8]})
        spatialDet.setIouThreshold(0.5)
        
        # Outputs
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        
        xoutDet = pipeline.create(dai.node.XLinkOut)
        xoutDet.setStreamName("detections")
        
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        
        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        
        camRgb.preview.link(spatialDet.input)
        camRgb.preview.link(xoutRgb.input)
        
        stereo.depth.link(spatialDet.inputDepth)
        spatialDet.out.link(xoutDet.input)
        spatialDet.passthroughDepth.link(xoutDepth.input)
        
        print("Pipeline built successfully.")
    except Exception as e:
        print(f"Error building pipeline: {e}")
        sys.exit(1)

    # Connect
    try:
        with dai.Device(pipeline) as device:
            print(f"Connected to {device.getDeviceName()}")
            
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            
            print("Running for 60 frames...")
            frames_captured = 0
            start_time = time.time()
            max_frames = 60
            
            detections_found = 0
            valid_depth_frames = 0
            
            while frames_captured < max_frames:
                in_rgb = q_rgb.tryGet()
                in_det = q_det.tryGet()
                in_depth = q_depth.tryGet()
                
                detections = []
                if in_det is not None:
                    detections = in_det.detections
                
                if in_rgb is not None:
                    frames_captured += 1
                    frame = in_rgb.getCvFrame()
                    
                    if len(detections) > 0:
                        detections_found += 1
                        for detection in detections:
                            # Spatial coordinates are in mm
                            x = detection.spatialCoordinates.x
                            y = detection.spatialCoordinates.y
                            z = detection.spatialCoordinates.z
                            conf = detection.confidence
                            
                            # Note: in DepthAI logic, Z is distance from camera plane
                            print(f"Frame {frames_captured}: Can detected! Pos=[{x:.1f}, {y:.1f}, {z:.1f}] mm, Conf={conf:.2f}")
                            print(f"  BBOX: [xmin:{detection.xmin:.3f}, ymin:{detection.ymin:.3f}, xmax:{detection.xmax:.3f}, ymax:{detection.ymax:.3f}]")
                            
                            # Annotate
                            h, w = frame.shape[:2]
                            x1 = int(detection.xmin * w)
                            y1 = int(detection.ymin * h)
                            x2 = int(detection.xmax * w)
                            y2 = int(detection.ymax * h)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                            cv2.putText(frame, f"Z: {z/1000.0:.2f}m", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                        # Save annotated frame
                        if detections_found == 1:
                            cv2.imwrite("test_04_spatial.jpg", frame)
                            print("Saved test_04_spatial.jpg")

                if in_depth is not None:
                    valid_depth_frames += 1

                if time.time() - start_time > 15:
                    print("Timeout reached!")
                    break
                
                time.sleep(0.005)

            if frames_captured > 0:
                print(f"Test 04 (Spatial): SUCCESS")
                print(f"  Processed {frames_captured} frames")
                print(f"  Found {detections_found} frames with detections")
                print(f"  Received {valid_depth_frames} depth frames")
            else:
                print("Test 04 (Spatial): FAILED (No frames)")
                sys.exit(1)
            
    except Exception as e:
        print(f"Failed to connect or run pipeline: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
