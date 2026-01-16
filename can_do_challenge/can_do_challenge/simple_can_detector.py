#!/usr/bin/env python3
"""
Simple color-based Coke can detector for OAK-D and Pi cameras.
Detects red objects (Coke cans) and provides bounding boxes and distance estimates.
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class SimpleCanDetector(Node):
    def __init__(self):
        super().__init__('simple_can_detector')
        
        # Declare parameters
        self.declare_parameter('camera_name', 'oakd_top')
        self.declare_parameter('use_depth', False)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('min_area_px2', 250)
        self.declare_parameter('min_bbox_height_px', 35)
        self.declare_parameter('max_distance_m', 1.8)
        self.declare_parameter('min_aspect', 0.25)
        self.declare_parameter('max_aspect', 2.0)
        self.declare_parameter('min_center_y_ratio', 0.0)
        self.declare_parameter('max_center_y_ratio', 1.0)
        self.declare_parameter('max_abs_x_m', 0.0)
        self.declare_parameter('log_throttle_sec', 5.0)
        self.declare_parameter('publish_process_heartbeat', True)
        self.declare_parameter('dump_debug_images', False)
        self.declare_parameter('debug_dump_dir', '/tmp/can_detector_debug')
        self.declare_parameter('debug_dump_every_n', 5)
        self.declare_parameter('dump_raw_images', False)
        self.declare_parameter('raw_dump_dir', '/tmp/can_detector_raw')
        self.declare_parameter('raw_dump_every_n', 10)
        
        # Get parameters
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        use_depth = self.get_parameter('use_depth').get_parameter_value().bool_value
        publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        self.min_area_px2 = int(self.get_parameter('min_area_px2').get_parameter_value().integer_value)
        self.min_bbox_height_px = int(self.get_parameter('min_bbox_height_px').get_parameter_value().integer_value)
        self.max_distance_m = float(self.get_parameter('max_distance_m').get_parameter_value().double_value)
        self.min_aspect = float(self.get_parameter('min_aspect').get_parameter_value().double_value)
        self.max_aspect = float(self.get_parameter('max_aspect').get_parameter_value().double_value)
        self.min_center_y_ratio = float(self.get_parameter('min_center_y_ratio').get_parameter_value().double_value)
        self.max_center_y_ratio = float(self.get_parameter('max_center_y_ratio').get_parameter_value().double_value)
        self.max_abs_x_m = float(self.get_parameter('max_abs_x_m').get_parameter_value().double_value)
        self.log_throttle_sec = float(self.get_parameter('log_throttle_sec').get_parameter_value().double_value)
        self.publish_process_heartbeat = bool(self.get_parameter('publish_process_heartbeat').get_parameter_value().bool_value)
        self.dump_debug_images = bool(self.get_parameter('dump_debug_images').get_parameter_value().bool_value)
        self.debug_dump_dir = self.get_parameter('debug_dump_dir').get_parameter_value().string_value
        self.debug_dump_every_n = int(self.get_parameter('debug_dump_every_n').get_parameter_value().integer_value)
        self.dump_raw_images = bool(self.get_parameter('dump_raw_images').get_parameter_value().bool_value)
        self.raw_dump_dir = self.get_parameter('raw_dump_dir').get_parameter_value().string_value
        self.raw_dump_every_n = int(self.get_parameter('raw_dump_every_n').get_parameter_value().integer_value)
        
        self.get_logger().info(f'Starting detector for camera: {camera_name}')
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # TF buffer for coordinate transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera info
        self.camera_info = None
        
        # Detection state
        self.latest_detection = None
        self.detection_time = None
        self.dump_counter = 0
        self.frame_counter = 0
        
        # Color detection parameters for red (Coke can)
        # HSV ranges for red (need two ranges because red wraps around 0/180)
        # Widened ranges for reliability in simulation
        self.red_lower1 = np.array([0, 50, 50])
        self.red_upper1 = np.array([15, 255, 255])
        self.red_lower2 = np.array([160, 50, 50])
        self.red_upper2 = np.array([180, 255, 255])
        
        # Filtering thresholds (tunable per camera instance via ROS params)
        # These defaults intentionally suppress far-away false positives in sim.
        
        # Subscribe to camera topics
        # For OAK-D: /oakd_top/color/image
        # For gripper: /gripper/camera/image
        if 'oakd' in camera_name.lower():
            image_topic = f'/{camera_name}/color/image'
            camera_info_topic = f'/{camera_name}/color/camera_info'
        else:
            image_topic = f'/{camera_name}/camera/image'
            camera_info_topic = f'/{camera_name}/camera/camera_info'
        
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10)
        
        self.get_logger().info(f'Subscribed to {image_topic} and {camera_info_topic}')
        
        # Publisher for detection results
        self.detection_pub = self.create_publisher(
            PointStamped,
            f'/{camera_name}/can_detection',
            10)

        # Publisher to signal each processed frame (even when no detection)
        self.processed_pub = self.create_publisher(
            Header,
            f'/{camera_name}/can_detection/processed',
            10)
        
        # Optional debug image publisher
        if publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                f'/{camera_name}/can_detection/debug',
                10)
        
        self.get_logger().info(f'Simple can detector initialized for {camera_name}')
    
    def camera_info_callback(self, msg):
        """Store camera calibration info."""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera info received')
    
    def image_callback(self, msg):
        """Process image and detect red objects."""
        if self.camera_info is None:
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Optional raw frame dump (independent of detection)
            if self.dump_raw_images:
                self.frame_counter += 1
                if self.frame_counter % max(self.raw_dump_every_n, 1) == 0:
                    os.makedirs(self.raw_dump_dir, exist_ok=True)
                    stamp = msg.header.stamp
                    filename = f"{self.get_name()}_{stamp.sec}_{stamp.nanosec}_raw.jpg"
                    filepath = os.path.join(self.raw_dump_dir, filename)
                    cv2.imwrite(filepath, cv_image)
                    self.get_logger().info(
                        f"Raw frame dumped: {filepath}",
                        throttle_duration_sec=self.log_throttle_sec)
            
            # Convert BGR to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask for red color (two ranges)
            mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Apply morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Publish heartbeat that this frame was processed (regardless of whether can is detected)
            # This allows BT nodes to synchronize with frame rate even when no valid detection
            if self.publish_process_heartbeat:
                header = Header()
                header.stamp = msg.header.stamp
                header.frame_id = msg.header.frame_id
                self.processed_pub.publish(header)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > self.min_area_px2:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(largest_contour)

                    # Reject tiny boxes early (usually noise / far objects)
                    if h < self.min_bbox_height_px:
                        return

                    # Basic shape filter: Coke can should not be extremely skinny or extremely wide
                    aspect = float(w) / float(h) if h > 0 else 999.0
                    if aspect < self.min_aspect or aspect > self.max_aspect:
                        return
                    
                    # Calculate center of bounding box
                    cx = x + w // 2
                    cy = y + h // 2

                    # Reject detections outside the allowed vertical ROI (image-relative)
                    img_h = cv_image.shape[0]
                    if img_h > 0:
                        cy_ratio = cy / float(img_h)
                        if cy_ratio < self.min_center_y_ratio or cy_ratio > self.max_center_y_ratio:
                            return
                    
                    # Estimate distance using bounding box height
                    # Assuming Coke can is ~12cm tall and standard focal length
                    # Distance (m) ≈ (real_height * focal_length) / pixel_height
                    can_height_m = 0.12
                    focal_length = self.camera_info.k[4]  # fy
                    distance = (can_height_m * focal_length) / h if h > 0 else 1.0

                    # Suppress far detections (very commonly false positives in this task)
                    if distance > self.max_distance_m:
                        return
                    
                    # Convert pixel coordinates to 3D point in camera frame
                    # Using pinhole camera model
                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    cx_cam = self.camera_info.k[2]
                    cy_cam = self.camera_info.k[5]
                    
                    # 3D point in camera frame (Optical conventions: X Right, Y Down, Z Forward)
                    x_3d = (cx - cx_cam) * distance / fx
                    y_3d = (cy - cy_cam) * distance / fy
                    z_3d = distance

                    # Reject detections too far off-center horizontally in 3D (optional)
                    if self.max_abs_x_m > 0.0 and abs(x_3d) > self.max_abs_x_m:
                        return
                    
                    # Create PointStamped message in camera optical frame
                    point_msg = PointStamped()
                    # Use optical frame from camera info
                    point_msg.header.frame_id = self.camera_info.header.frame_id
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    # Publish RAW optical coordinates - let TF handle the rotation to base_link
                    point_msg.point.x = x_3d
                    point_msg.point.y = y_3d
                    point_msg.point.z = z_3d
                    
                    # Store logic for 3D point in camera frame
                    # Do NOT transform to map to avoid artifacts from poor distance/localization during rotation
                    self.detection_pub.publish(point_msg)
                        
                    self.latest_detection = point_msg.point
                    self.detection_time = self.get_clock().now()
                        
                    self.get_logger().info(
                        f'Can detected at ({point_msg.point.x:.2f}, {point_msg.point.y:.2f}, '
                        f'{point_msg.point.z:.2f}) in {point_msg.header.frame_id}, '
                        f'distance: {distance:.2f}m bbox=({x},{y},{w},{h})',
                        throttle_duration_sec=self.log_throttle_sec)
                    
                    # Publish debug image if enabled
                    if hasattr(self, 'debug_image_pub'):
                        debug_img = cv_image.copy()
                        cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(debug_img, f'{distance:.2f}m', (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                        debug_msg.header = msg.header
                        self.debug_image_pub.publish(debug_msg)

                    # Optionally dump debug images to disk
                    if self.dump_debug_images:
                        self.dump_counter += 1
                        if self.dump_counter % max(self.debug_dump_every_n, 1) == 0:
                            os.makedirs(self.debug_dump_dir, exist_ok=True)
                            stamp = msg.header.stamp
                            filename = f"{self.get_name()}_{stamp.sec}_{stamp.nanosec}.jpg"
                            filepath = os.path.join(self.debug_dump_dir, filename)
                            debug_img = cv_image.copy()
                            cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
                            cv2.putText(debug_img, f'{distance:.2f}m', (x, y - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            cv2.imwrite(filepath, debug_img)
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCanDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
