#!/usr/bin/env python3
"""
Annotate OAK-D RGB images using on-device spatial detections.
Publishes /oakd/annotated_image by default.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray
from cv_bridge import CvBridge
import cv2


class SpatialDetectionAnnotator(Node):
    def __init__(self):
        super().__init__("oakd_spatial_annotator")
        
        self.camera_info = None

        self.declare_parameter("image_topic", "/oakd_top/oak/rgb/image_raw")
        self.declare_parameter("camera_info_topic", "/oakd_top/oak/rgb/camera_info")
        self.declare_parameter("detections_topic", "/oakd_top/oak/nn/spatial_detections")
        self.declare_parameter("annotated_topic", "/oakd/annotated_image")
        self.declare_parameter("labels", ["Can"])
        self.declare_parameter("min_score", 0.3)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.labels = self.get_parameter("labels").get_parameter_value().string_array_value
        self.min_score = float(self.get_parameter("min_score").get_parameter_value().double_value)

        self.bridge = CvBridge()
        self.latest_detections = None

        detections_qos = QoSProfile(depth=1)
        detections_qos.reliability = ReliabilityPolicy.RELIABLE
        detections_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            Detection3DArray,
            detections_topic,
            self.on_detections,
            detections_qos,
        )
        self.create_subscription(
            Image,
            image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        annotated_qos = QoSProfile(depth=1)
        annotated_qos.reliability = ReliabilityPolicy.RELIABLE
        annotated_qos.durability = DurabilityPolicy.VOLATILE
        self.annotated_pub = self.create_publisher(
            Image,
            annotated_topic,
            annotated_qos,
        )

        self.get_logger().info(
            f"Annotating {image_topic} using {detections_topic} -> {annotated_topic}"
        )

    def on_detections(self, msg: Detection3DArray):
        self.latest_detections = msg
        
    def on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def on_image(self, msg: Image):
        if self.latest_detections is None:
            return
            
        # We need camera info to project 3D -> 2D
        if self.camera_info is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return
            
        # K matrix: fx, 0, cx, 0, fy, cy, 0, 0, 1
        K = self.camera_info.k
        fx = K[0]
        cx = K[2]
        fy = K[4]
        cy = K[5]

        for det in self.latest_detections.detections:
            score = 0.0
            class_id = ""
            if det.results:
                best = max(det.results, key=lambda r: r.score)
                score = float(best.score)
                class_id = str(best.class_id)

            if score < self.min_score:
                continue

            bbox = det.bbox
            pos = bbox.center.position
            # bbox size is Vector3 (x, y, z) in meters
            size = bbox.size
            
            # Project center (x, y, z) to u, v
            # z is depth
            if pos.z > 0.0:
                # Normal 3D projection
                u = int((fx * pos.x / pos.z) + cx)
                v = int((fy * pos.y / pos.z) + cy)
                
                # Approximate 2D size by projecting half-width at that depth
                width_msg = size.x 
                height_msg = size.y
                
                # Project a point (x + w/2, y + h/2, z)
                u_corner = int((fx * (pos.x + width_msg/2) / pos.z) + cx)
                v_corner = int((fy * (pos.y + height_msg/2) / pos.z) + cy)
                
                half_w = abs(u_corner - u)
                half_h = abs(v_corner - v)
            else:
                # Fallback: Treat X/Y as raw pixel coordinates if Z is <= 0 (DepthAI issue)
                # Note: BBox center X/Y in pixels
                u = int(pos.x)
                v = int(pos.y)
                half_w = 40  # Default size since we can't project size
                half_h = 40
            
            # If coordinates are massively out of bounds, maybe they are normalized [0,1]?
            # But the log showed X=184, Y=626 which are likely pixels.

            # If size is zero/invalid, use a default radius
            if half_w < 5: half_w = 20
            if half_h < 5: half_h = 20

            x1 = u - half_w
            y1 = v - half_h
            x2 = u + half_w
            y2 = v + half_h

            label = class_id
            if label.isdigit():
                idx = int(label)
                if 0 <= idx < len(self.labels):
                    label = self.labels[idx]
            if not label:
                label = "obj"

            text = f"{label} {score:.2f} [{pos.z:.2f}m]"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                text,
                (x1, max(0, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)



def main():
    rclpy.init()
    node = SpatialDetectionAnnotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
