#!/home/ros/sigyn-venv/bin/python3

import cv2
import depthai as dai
import numpy as np
import sys
import os

def main():
    BLOB_PATH = "/home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/best_openvino_2022.1_6shave.blob"
    IMAGE_PATH = "oakd_capture.jpg"
    
    if not os.path.exists(BLOB_PATH):
        print(f"Error: Blob not found at {BLOB_PATH}")
        sys.exit(1)
        
    print(f"Testing Blob: {BLOB_PATH}")
    
    pipeline = dai.Pipeline()
    
    # Define sources and outputs
    xin = pipeline.create(dai.node.XLinkIn)
    xin.setStreamName("in")
    
    yoloDet = pipeline.create(dai.node.YoloDetectionNetwork)
    yoloDet.setBlobPath(BLOB_PATH)
    yoloDet.setConfidenceThreshold(0.1)
    # Using the same anchors/masks as test_04 which were validated against current best.pt
    yoloDet.setNumClasses(1)
    yoloDet.setCoordinateSize(4)
    yoloDet.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326])
    yoloDet.setAnchorMasks({"side80": [0, 1, 2], "side40": [3, 4, 5], "side20": [6, 7, 8]})
    yoloDet.setIouThreshold(0.5)
    
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("nn")
    
    xin.out.link(yoloDet.input)
    yoloDet.out.link(xout.input)
    
    # Load Image
    img = cv2.imread(IMAGE_PATH)
    h, w = img.shape[:2]
    # Resize to 640x640 if needed (it is already 640x640 from previous step but good to be safe)
    # Note: OAK-D usually expects planar BGR or similar. XLinkIn expects simple buffer.
    # We need to transform to correct shape.
    
    # Transform image to planar (C, H, W)
    img_planar = img.transpose(2, 0, 1)
    
    with dai.Device(pipeline) as device:
        q_in = device.getInputQueue("in")
        q_out = device.getOutputQueue("nn", maxSize=4, blocking=False)
        
        # Create ImgFrame
        imgFrame = dai.ImgFrame()
        imgFrame.setData(img_planar)
        imgFrame.setWidth(640)
        imgFrame.setHeight(640)
        imgFrame.setType(dai.ImgFrame.Type.BGR888p) # Planar BGR
        
        print("Sending image to device...")
        q_in.send(imgFrame)
        
        print("Waiting for inference...")
        in_nn = q_out.get() # Blocking get
        
        if in_nn is not None:
            detections = in_nn.detections
            print(f"Received {len(detections)} detections:")
            for detection in detections:
                print(f"  Class: {detection.label}, Conf: {detection.confidence:.3f}")
                print(f"  Box: [{detection.xmin:.3f}, {detection.ymin:.3f}, {detection.xmax:.3f}, {detection.ymax:.3f}]")
                
                # Draw
                x1 = int(detection.xmin * w)
                y1 = int(detection.ymin * h)
                x2 = int(detection.xmax * w)
                y2 = int(detection.ymax * h)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f"Conf: {detection.confidence:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            cv2.imwrite("test_05_blob_result.jpg", img)
            print("Saved test_05_blob_result.jpg")
        else:
            print("No response from NN.")

if __name__ == "__main__":
    main()
