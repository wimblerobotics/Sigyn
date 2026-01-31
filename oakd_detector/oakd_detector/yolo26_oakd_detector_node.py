#!/usr/bin/env python3
"""
OAK-D detection node using YOLO26 for Coke can detection.
Publishes detections and annotated images.
Subscribes to /sigyn/take_oakd_picture to capture and save images.
"""
import os
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

# Add the virtual environment's site-packages to sys.path for depthai
venv_site_packages = os.path.expanduser('~/sigyn-venv/lib/python3.12/site-packages')
if venv_site_packages not in sys.path:
    sys.path.insert(0, venv_site_packages)

try:
    import depthai as dai
except ImportError as e:
    print(f"ERROR: Failed to import depthai: {e}")
    sys.exit(1)

try:
    from ultralytics import YOLO
except ImportError as e:
    print(f"ERROR: Failed to import ultralytics: {e}")
    print("Please install: pip install ultralytics")
    sys.exit(1)


class YOLO26OakdDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo26_oakd_detector_node')
        self.get_logger().info("YOLO26 OAK-D Detector Node initialized.")
        
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('mxId', '14442C1051B665D700')
        self.declare_parameter('model_path', '')  # Path to YOLO26 model
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('trained_images_dir', '')
        
        # Get parameters
        self.mx_id = self.get_parameter('mxId').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.image_size = self.get_parameter('image_size').get_parameter_value().integer_value
        trained_images_dir = self.get_parameter('trained_images_dir').get_parameter_value().string_value
        
        # Set up trained images directory
        if trained_images_dir:
            self.trained_images_dir = Path(trained_images_dir)
        else:
            # Default to workspace src directory
            self.trained_images_dir = Path('/home/ros/sigyn_ws/src/Sigyn/trained_images')
        
        self.trained_images_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Trained images directory: {self.trained_images_dir}")
        
        # Load YOLO26 model
        if not model_path:
            # Default to looking for the model in the package
            pkg_share_dir = get_package_share_directory('oakd_detector')
            model_path = os.path.join(pkg_share_dir, 'resources', 'yolo26n.pt')
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at: {model_path}")
            self.get_logger().error("Please download your RoboFlow v3 YOLO26 model and specify the path.")
            raise FileNotFoundError(f"Model not found: {model_path}")
        
        self.get_logger().info(f"Loading YOLO26 model from: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO26 model loaded successfully.")
        
        # Publishers
        self.detections_pub = self.create_publisher(
            Detection2DArray, 
            '/oakd/detections', 
            10
        )
        self.annotated_image_pub = self.create_publisher(
            Image, 
            '/oakd/annotated_image', 
            10
        )
        self.raw_image_pub = self.create_publisher(
            Image,
            '/oakd/raw_image',
            10
        )
        
        # Subscriber for image capture requests
        self.capture_sub = self.create_subscription(
            Bool,
            '/sigyn/take_oakd_picture',
            self.capture_callback,
            10
        )
        
        # Initialize OAK-D pipeline
        try:
            self.setup_oakd_pipeline()
        except Exception as e:
            self.get_logger().error(f"Failed to setup OAK-D pipeline: {e}")
            if hasattr(self, 'pipeline') and self.pipeline is not None:
                try:
                    self.pipeline.stop()
                except:
                    pass
            raise
        
        # Timer for processing frames
        self.timer = self.create_timer(0.03, self.process_frame)  # ~30 FPS
        
        self.get_logger().info("Node initialization complete.")
    
    def setup_oakd_pipeline(self):
        """Set up the OAK-D camera pipeline."""
        self.get_logger().info("Setting up OAK-D pipeline...")
        
        pipeline = dai.Pipeline()
        
        # Create RGB color camera for both detection and capture
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(self.image_size, self.image_size)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        
        # Create output queues BEFORE starting pipeline (depthai 3.x requirement)
        # Use preview for detection and video for high-res captures
        self.q_rgb = cam_rgb.preview.createOutputQueue(maxSize=4, blocking=False)
        self.q_video = cam_rgb.video.createOutputQueue(maxSize=4, blocking=False)
        self.get_logger().info("Output queues created")
        
        # Start the pipeline (internally creates and manages device connection)
        pipeline.start()
        self.get_logger().info("OAK-D pipeline started successfully")
        
        # Store pipeline reference to keep it alive
        self.pipeline = pipeline
    
    def process_frame(self):
        """Process frames from OAK-D and run YOLO26 detection."""
        try:
            # Get RGB frame
            in_rgb = self.q_rgb.tryGet()
            if in_rgb is None:
                return
            
            frame = in_rgb.getCvFrame()
            
            # Publish raw image
            raw_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            raw_msg.header.stamp = self.get_clock().now().to_msg()
            raw_msg.header.frame_id = "oakd_rgb_camera_optical_frame"
            self.raw_image_pub.publish(raw_msg)
            
            # Run YOLO26 detection (end-to-end, no NMS needed!)
            results = self.model.predict(
                frame,
                conf=self.confidence_threshold,
                imgsz=self.image_size,
                verbose=False
            )
            
            # Process results
            if len(results) > 0:
                result = results[0]
                
                # Create annotated image (always, even if no detections)
                annotated_frame = result.plot()
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
                annotated_msg.header.stamp = self.get_clock().now().to_msg()
                annotated_msg.header.frame_id = "oakd_rgb_camera_optical_frame"
                self.annotated_image_pub.publish(annotated_msg)
                
                # Create Detection2DArray message (Publish empty if no detections)
                detections_msg = Detection2DArray()
                detections_msg.header.stamp = self.get_clock().now().to_msg()
                detections_msg.header.frame_id = "oakd_rgb_camera_optical_frame"
                
                # Process detections if any found
                if len(result.boxes) > 0:
                    # Process each detection
                    for box in result.boxes:
                        detection = Detection2D()
                        
                        # Bounding box
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        detection.bbox.center.position.x = float((x1 + x2) / 2)
                        detection.bbox.center.position.y = float((y1 + y2) / 2)
                        detection.bbox.size_x = float(x2 - x1)
                        detection.bbox.size_y = float(y2 - y1)
                        
                        # Class and confidence
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                        hypothesis.hypothesis.score = float(box.conf[0])
                        detection.results.append(hypothesis)
                        
                        detections_msg.detections.append(detection)
                    
                self.detections_pub.publish(detections_msg)
                # self.get_logger().info(f"Published {len(result.boxes)} detection(s)")
            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}", throttle_duration_sec=1.0)
    
    def capture_callback(self, msg):
        """Handle image capture requests."""
        if msg.data:
            try:
                # Get high-res frame from video queue
                if self.q_video is not None:
                    frame = self.q_video.get()
                    if frame is not None:
                        img = frame.getCvFrame()
                        
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        filename = f"oakd_capture_{timestamp}.jpg"
                        filepath = self.trained_images_dir / filename
                        
                        # Save the RGB video image
                        cv2.imwrite(str(filepath), img)
                        
                        self.get_logger().info(f"Captured image saved: {filepath}")
                    else:
                        self.get_logger().warn("Capture requested but no frame available.")
                else:
                    self.get_logger().warn("Video queue not available for capture.")
            except Exception as e:
                self.get_logger().error(f"Failed to save captured image: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = YOLO26OakdDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception: {e}")
        else:
            print(f"ERROR: Unhandled exception before node init: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
