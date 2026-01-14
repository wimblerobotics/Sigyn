#!/usr/bin/env python3
"""
Simple color-based Coke can detector for OAK-D and Pi cameras.
Detects red objects (Coke cans) and provides bounding boxes and distance estimates.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
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
        
        # Get parameters
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        use_depth = self.get_parameter('use_depth').get_parameter_value().bool_value
        publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        
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
        
        # Color detection parameters for red (Coke can)
        # HSV ranges for red (need two ranges because red wraps around 0/180)
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([180, 255, 255])
        
        # Minimum contour area to consider as detection
        self.min_area = 500  # pixels²
        
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
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > self.min_area:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    
                    # Calculate center of bounding box
                    cx = x + w // 2
                    cy = y + h // 2
                    
                    # Estimate distance using bounding box height
                    # Assuming Coke can is ~12cm tall and standard focal length
                    # Distance (m) ≈ (real_height * focal_length) / pixel_height
                    can_height_m = 0.12
                    focal_length = self.camera_info.k[4]  # fy
                    distance = (can_height_m * focal_length) / h if h > 0 else 1.0
                    
                    # Convert pixel coordinates to 3D point in camera frame
                    # Using pinhole camera model
                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    cx_cam = self.camera_info.k[2]
                    cy_cam = self.camera_info.k[5]
                    
                    # 3D point in camera frame
                    x_3d = (cx - cx_cam) * distance / fx
                    y_3d = (cy - cy_cam) * distance / fy
                    z_3d = distance
                    
                    # Create PointStamped message in camera optical frame
                    point_msg = PointStamped()
                    # Use optical frame from camera info
                    point_msg.header.frame_id = self.camera_info.header.frame_id
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.point.x = z_3d  # Forward
                    point_msg.point.y = -x_3d  # Left (camera x is right)
                    point_msg.point.z = -y_3d  # Up (camera y is down)
                    
                    # Try to transform to map frame
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            point_msg.header.frame_id,
                            rclpy.time.Time(),
                            timeout=rclpy.duration.Duration(seconds=0.1))
                        
                        point_map = do_transform_point(point_msg, transform)
                        self.detection_pub.publish(point_map)
                        
                        self.latest_detection = point_map.point
                        self.detection_time = self.get_clock().now()
                        
                        self.get_logger().info(
                            f'Can detected at ({point_map.point.x:.2f}, {point_map.point.y:.2f}, '
                            f'{point_map.point.z:.2f}) in map frame, distance: {distance:.2f}m',
                            throttle_duration_sec=1.0)
                    
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                            tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warn(f'TF transform failed: {e}', throttle_duration_sec=5.0)
                        # Publish in camera frame anyway
                        self.detection_pub.publish(point_msg)
                        self.latest_detection = point_msg.point
                        self.detection_time = self.get_clock().now()
                    
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
