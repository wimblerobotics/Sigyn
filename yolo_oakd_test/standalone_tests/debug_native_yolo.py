#!/home/ros/sigyn-venv/bin/python3

import cv2
import depthai as dai
import torch
import sys
import os
import time

def capture_image():
    print("Initializing OAK-D for image capture...")
    pipeline = dai.Pipeline()
    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setPreviewSize(640, 640)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    camRgb.preview.link(xoutRgb.input)
    
    img_path = "oakd_capture.jpg"
    
    try:
        with dai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            print("Connected to OAK-D. Waiting for camera warmup...")
            
            # Flush initial frames
            time.sleep(2)
            for _ in range(10):
                q_rgb.tryGet()
                
            print("Capturing frame...")
            start = time.time()
            while time.time() - start < 5.0:
                in_rgb = q_rgb.tryGet()
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    cv2.imwrite(img_path, frame)
                    print(f"Image successfully saved to {os.path.abspath(img_path)}")
                    return img_path, frame
                time.sleep(0.1)
                
            print("Timeout: No frame received.")
            return None, None
            
    except Exception as e:
        print(f"Error initializing OAK-D: {e}")
        return None, None

def run_inference(image_path):
    print("\n--- Running Native YOLOv5 Inference ---")
    
    # Paths
    workspace_root = "/home/ros/sigyn_ws/src/Sigyn"
    yolo_local_repo = os.path.join(workspace_root, "yolov5")
    weights_path = os.path.join(workspace_root, "yolo_oakd_test/models/can_detector/weights/best.pt")
    
    if not os.path.exists(weights_path):
        print(f"Error: Weights not found at {weights_path}")
        return

    # Add yolov5 to path so torch.hub can find it if needed, or simply for modules
    if yolo_local_repo not in sys.path:
        sys.path.append(yolo_local_repo)

    print(f"Weights: {weights_path}")
    print(f"Image:   {image_path}")
    
    try:
        # Load model using torch.hub from local repository
        # 'custom' creates a model from local file
        print("Loading model...")
        # Workaround for 'weights_only' if needed, but trying standard load first
        # We explicitly trust the repo
        torch.serialization.add_safe_globals([set]) # Sometimes needed for older pickups
        
        # We use source='local' to avoid hitting github
        model = torch.hub.load(yolo_local_repo, 'custom', path=weights_path, source='local')
        
        # Inference
        print("Running inference on image...")
        results = model(image_path)
        
        # Results
        print("\nINFERENCE RESULTS:")
        results.print()  # Print results to screen
        
        print("\nDETAILED BOXES:")
        # xyxy format: xmin, ymin, xmax, ymax, confidence, class
        df = results.pandas().xyxy[0] 
        print(df)
        
        # Save visualized result
        results.save(save_dir='debug_output')
        print(f"\nAnnotated image saved to 'debug_output/'")

    except Exception as e:
        print(f"Inference failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    saved_img, _ = capture_image()
    if saved_img:
        run_inference(saved_img)
