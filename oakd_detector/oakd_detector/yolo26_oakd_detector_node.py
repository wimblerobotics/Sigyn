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
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
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
        self.declare_parameter('depth_topic', '/oakd_top/oak/stereo/image_raw')
        self.declare_parameter('image_topic', '/oakd_top/oak/rgb/image_raw')
        self.declare_parameter('depth_camera_info_topic', '/oakd_top/oak/stereo/camera_info')
        self.declare_parameter('depth_window_px', 5)
        
        # Get parameters
        self.mx_id = self.get_parameter('mxId').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.image_size = self.get_parameter('image_size').get_parameter_value().integer_value
        trained_images_dir = self.get_parameter('trained_images_dir').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.depth_camera_info_topic = self.get_parameter('depth_camera_info_topic').get_parameter_value().string_value
        self.depth_window_px = int(self.get_parameter('depth_window_px').get_parameter_value().integer_value)
        
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
        # Use BEST_EFFORT QoS to match subscriber
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        can_detection_qos = QoSProfile(depth=10)
        can_detection_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        self.can_detection_pub = self.create_publisher(
            PointStamped,
            '/oakd/can_detection',
            can_detection_qos
        )

        self.depth_camera_info = None
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.latest_depth_frame = None
        self.latest_image = None
        self.latest_image_stamp = None

        self.depth_camera_info_sub = self.create_subscription(
            CameraInfo,
            self.depth_camera_info_topic,
            self.depth_camera_info_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Subscriber for image capture requests
        self.capture_sub = self.create_subscription(
            Bool,
            '/sigyn/take_oakd_picture',
            self.capture_callback,
            10
        )
        
        # Timer for processing frames (only when new image arrives)
        self.timer = self.create_timer(0.03, self.process_frame)  # ~30 FPS
        
        self.get_logger().info("Node initialization complete.")

    def depth_camera_info_callback(self, msg: CameraInfo):
        if self.depth_camera_info is None:
            self.depth_camera_info = msg
            self.get_logger().info("Depth camera info received for OAK-D")

    def depth_callback(self, msg: Image):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_img
            self.latest_depth_stamp = msg.header.stamp
            self.latest_depth_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Failed to decode depth image: {e}", throttle_duration_sec=1.0)
    
    def image_callback(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = image
            self.latest_image_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Failed to decode RGB image: {e}", throttle_duration_sec=1.0)
    
    def process_frame(self):
        """Process frames from subscribed image topic and run YOLO26 detection."""
        if self.latest_image is None:
            return
            
        try:
            frame = self.latest_image
            
            # Publish raw image
            raw_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            raw_msg.header.stamp = self.latest_image_stamp if self.latest_image_stamp else self.get_clock().now().to_msg()
            raw_msg.header.frame_id = "oak_rgb_camera_optical_frame"
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
                annotated_msg.header.frame_id = "oak_rgb_camera_optical_frame"
                self.annotated_image_pub.publish(annotated_msg)
                
                # Create Detection2DArray message (Publish empty if no detections)
                detections_msg = Detection2DArray()
                detections_msg.header.stamp = self.get_clock().now().to_msg()
                detections_msg.header.frame_id = "oak_rgb_camera_optical_frame"
                
                # Process detections if any found
                if len(result.boxes) > 0:
                    # Process each detection
                    best_box = None
                    best_conf = -1.0
                    # TODO: Handle multiple cans by publishing all detections or selecting via tracking.
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

                        if hypothesis.hypothesis.score > best_conf:
                            best_conf = hypothesis.hypothesis.score
                            best_box = (x1, y1, x2, y2)

                    # If depth is available, compute a 3D point for the best detection
                    if best_box is not None and self.latest_depth is not None and self.depth_camera_info is not None:
                        depth_img = self.latest_depth
                        depth_h, depth_w = depth_img.shape[:2]
                        frame_h, frame_w = frame.shape[:2]

                        x1, y1, x2, y2 = best_box
                        cx = (x1 + x2) / 2.0
                        cy = (y1 + y2) / 2.0

                        self.get_logger().info("==== OAKD 3D DETECTION TRACE ====")
                        self.get_logger().info(f"YOLO bbox (x1,y1,x2,y2): ({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})")
                        self.get_logger().info(f"RGB frame size: {frame_w} x {frame_h}")
                        self.get_logger().info(f"Depth image size: {depth_w} x {depth_h}")
                        self.get_logger().info(f"Depth camera_info size: {self.depth_camera_info.width} x {self.depth_camera_info.height}")
                        self.get_logger().info(f"Depth frame_id: '{self.latest_depth_frame}'")
                        self.get_logger().info(f"Depth camera_info frame_id: '{self.depth_camera_info.header.frame_id}'")
                        self.get_logger().info(f"RGB bbox center (px): ({cx:.1f}, {cy:.1f})")
                        self.get_logger().info(f"SCALING: Using frame_w={frame_w} for mapping (should be camera_info.width={self.depth_camera_info.width})")

                        if frame_w > 0 and frame_h > 0:
                            u = int(round(cx * depth_w / float(frame_w)))
                            v = int(round(cy * depth_h / float(frame_h)))

                            self.get_logger().info(f"Mapped depth pixel (u,v): ({u}, {v})")

                            half = max(0, int(self.depth_window_px // 2))
                            u0 = max(0, u - half)
                            v0 = max(0, v - half)
                            u1 = min(depth_w - 1, u + half)
                            v1 = min(depth_h - 1, v + half)
                            window = depth_img[v0:v1 + 1, u0:u1 + 1]

                            self.get_logger().info(f"Depth window: ({u0}:{u1}, {v0}:{v1}) size={window.shape}")

                            if window.size > 0:
                                if window.dtype == np.uint16:
                                    depth_m = np.median(window) / 1000.0
                                else:
                                    depth_m = float(np.median(window))

                                self.get_logger().info(f"Depth (m): {depth_m:.3f}")

                                if depth_m > 0.01:
                                    # Get depth camera intrinsics directly (no scaling)
                                    fx = self.depth_camera_info.k[0]
                                    fy = self.depth_camera_info.k[4]
                                    cx_cam = self.depth_camera_info.k[2]
                                    cy_cam = self.depth_camera_info.k[5]

                                    self.get_logger().info(
                                        f"Depth cam intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx_cam:.2f} cy={cy_cam:.2f}"
                                    )

                                    # Project using depth pixel coordinates (u, v) not RGB coordinates (cx, cy)
                                    x_3d = (u - cx_cam) * depth_m / fx
                                    y_3d = (v - cy_cam) * depth_m / fy
                                    z_3d = depth_m

                                    self.get_logger().info(
                                        f"3D point in depth cam frame: ({x_3d:.3f}, {y_3d:.3f}, {z_3d:.3f})"
                                    )

                                    point_msg = PointStamped()
                                    point_msg.header.stamp = self.get_clock().now().to_msg()
                                    point_msg.header.frame_id = self.depth_camera_info.header.frame_id
                                    point_msg.point.x = float(x_3d)
                                    point_msg.point.y = float(y_3d)
                                    point_msg.point.z = float(z_3d)
                                    
                                    self.get_logger().info(
                                        f"Publishing to /oakd/can_detection with frame_id='{point_msg.header.frame_id}'"
                                    )
                                    self.get_logger().info("==================================")
                                    
                                    self.can_detection_pub.publish(point_msg)
                                else:
                                    self.get_logger().info("Depth too small (<0.01m), skipping 3D publication")
                                    self.get_logger().info("==================================")
                    
                self.detections_pub.publish(detections_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}", throttle_duration_sec=1.0)
    
    def capture_callback(self, msg):
        """Handle image capture requests."""
        if msg.data:
            try:
                # Use the latest image from subscription
                if self.latest_image is not None:
                    img = self.latest_image
                    
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"oakd_capture_{timestamp}.jpg"
                    filepath = self.trained_images_dir / filename
                    
                    # Save the RGB image
                    cv2.imwrite(str(filepath), img)
                    
                    self.get_logger().info(f"Captured image saved: {filepath}")
                else:
                    self.get_logger().warn("Capture requested but no image available.")
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
