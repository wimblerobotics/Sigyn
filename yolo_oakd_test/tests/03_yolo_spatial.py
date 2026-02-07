#!/usr/bin/env python3
import depthai as dai
import cv2
import time
import sys
import numpy as np
import os

print("Test 3: YOLO Spatial Detection")

blob_path = "/home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/can_detector.blob"
if not os.path.exists(blob_path):
    print(f"Blob not found at {blob_path}")
    sys.exit(1)

try:
    pipeline = dai.Pipeline()

    # --- RGB Camera ---
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setPreviewSize(640, 640) # YOLO input size
    cam_rgb.setFps(15)

    # --- Stereo Depth ---
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setCamera("left")
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setCamera("right")

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setSubpixel(True)
    
    # Link Mono -> Stereo
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    # --- YOLO Spatial Detection ---
    print("Creating YoloSpatialDetectionNetwork node...")
    yolo_spatial = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
    yolo_spatial.setBlobPath(blob_path)
    yolo_spatial.setConfidenceThreshold(0.5)
    
    # YOLO specific settings
    yolo_spatial.setNumClasses(1)
    yolo_spatial.setCoordinateSize(4)
    yolo_spatial.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326])
    yolo_spatial.setAnchorMasks({"side80": [0, 1, 2], "side40": [3, 4, 5], "side20": [6, 7, 8]})
    yolo_spatial.setIouThreshold(0.5)
    
    # Spatial settings
    yolo_spatial.setBoundingBoxScaleFactor(0.5)
    yolo_spatial.setDepthLowerThreshold(100)
    yolo_spatial.setDepthUpperThreshold(10000)

    # Link inputs
    cam_rgb.preview.link(yolo_spatial.input)
    stereo.depth.link(yolo_spatial.inputDepth)

    # --- Outputs ---
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input) # Stream the 640x640 preview

    xout_nn = pipeline.create(dai.node.XLinkOut)
    xout_nn.setStreamName("detections")
    yolo_spatial.out.link(xout_nn.input)

    # --- Run ---
    print("Pipeline created. Connecting to device...")
    with dai.Device(pipeline) as device:
        print("Device connected. Starting pipeline...")
        print(f"USB Speed: {device.getUsbSpeed()}")
        
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        
        start_time = time.time()
        frames = 0
        detections_count = 0
        
        while time.time() - start_time < 10.0:
            in_rgb = q_rgb.tryGet()
            in_det = q_det.tryGet()
            
            if in_rgb is not None:
                frames += 1
                
            if in_det is not None:
                detections = in_det.detections
                if len(detections) > 0:
                    detections_count += len(detections)
                    for detection in detections:
                        print(f"Detection: Label={detection.label}, Conf={detection.confidence:.2f}")
                        print(f"  BBox: {detection.xmin:.2f}, {detection.ymin:.2f}, {detection.xmax:.2f}, {detection.ymax:.2f}")
                        print(f"  Spatial: X={detection.spatialCoordinates.x:.1f}, Y={detection.spatialCoordinates.y:.1f}, Z={detection.spatialCoordinates.z:.1f} mm")
            
            time.sleep(0.001)
            
        print(f"Finished. Processed {frames} frames, found {detections_count} detections.")
        
    print("Test 3 Passed: Pipeline ran without error.")

except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)
