#!/usr/bin/env python3
"""One-shot OAK-D can probe.

Subscribes to YOLO 2D detections, depth image, and depth camera info, then:
- maps 2D bbox center to depth pixel
- reads depth value
- projects to 3D in depth camera frame
- transforms to base_link
- prints distances and angles

Exits after the first successful computation.
"""
import math
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs


class OakdCanProbe(Node):
    def __init__(self):
        super().__init__("oakd_can_probe")

        self.declare_parameter("detections_topic", "/oakd/detections")
        self.declare_parameter("depth_topic", "/oakd_top/oak/stereo/image_raw")
        self.declare_parameter("depth_camera_info_topic", "/oakd_top/oak/stereo/camera_info")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("depth_window_px", 5)

        self.detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.depth_camera_info_topic = self.get_parameter("depth_camera_info_topic").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.depth_window_px = int(self.get_parameter("depth_window_px").get_parameter_value().integer_value)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.latest_depth_frame = None
        self.depth_info = None
        self.latest_can_detection = None
        self.latest_can_detection_stamp = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Use BEST_EFFORT QoS for sensor topics
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Detection2DArray, self.detections_topic, self.on_detections, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self.on_depth, sensor_qos)
        self.create_subscription(CameraInfo, self.depth_camera_info_topic, self.on_depth_info, sensor_qos)
        self.create_subscription(PointStamped, "/oakd/can_detection", self.on_can_detection, sensor_qos)

        self.get_logger().info("OakdCanProbe ready. Waiting for one detection...")

    def on_depth_info(self, msg: CameraInfo):
        if self.depth_info is None:
            self.depth_info = msg
            self.get_logger().info(
                f"Depth camera info received: frame_id='{msg.header.frame_id}', size=({msg.width}x{msg.height})"
            )

    def on_depth(self, msg: Image):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = depth_img
            self.latest_depth_stamp = msg.header.stamp
            self.latest_depth_frame = msg.header.frame_id
        except Exception as exc:
            self.get_logger().error(f"Failed to decode depth image: {exc}")

    def on_can_detection(self, msg: PointStamped):
        self.latest_can_detection = msg
        self.latest_can_detection_stamp = msg.header.stamp

    def on_detections(self, msg: Detection2DArray):
        if self.depth_info is None or self.latest_depth is None:
            return
        if not msg.detections:
            return

        det = msg.detections[0]
        cx = float(det.bbox.center.position.x)
        cy = float(det.bbox.center.position.y)
        w = float(det.bbox.size_x)
        h = float(det.bbox.size_y)

        depth_img = self.latest_depth
        depth_h, depth_w = depth_img.shape[:2]
        depth_frame = self.latest_depth_frame or ""

        # Map RGB center to depth pixel (in case resolutions differ)
        if msg.header.frame_id:
            self.get_logger().info(f"Detection frame_id: '{msg.header.frame_id}'")
        u = int(round(cx * depth_w / float(self.depth_info.width)))
        v = int(round(cy * depth_h / float(self.depth_info.height)))
        u = max(0, min(depth_w - 1, u))
        v = max(0, min(depth_h - 1, v))

        half = max(0, int(self.depth_window_px // 2))
        u0 = max(0, u - half)
        v0 = max(0, v - half)
        u1 = min(depth_w - 1, u + half)
        v1 = min(depth_h - 1, v + half)
        window = depth_img[v0:v1 + 1, u0:u1 + 1]

        if window.size == 0:
            self.get_logger().error("Depth window empty; cannot compute depth")
            return

        if window.dtype == np.uint16:
            depth_m = float(np.median(window)) / 1000.0
        else:
            depth_m = float(np.median(window))

        fx = self.depth_info.k[0]
        fy = self.depth_info.k[4]
        cx_cam = self.depth_info.k[2]
        cy_cam = self.depth_info.k[5]

        # Project into depth camera frame
        x_3d = (u - cx_cam) * depth_m / fx
        y_3d = (v - cy_cam) * depth_m / fy
        z_3d = depth_m

        point_msg = PointStamped()
        point_msg.header.stamp = msg.header.stamp
        point_msg.header.frame_id = depth_frame if depth_frame else self.depth_info.header.frame_id
        point_msg.point.x = float(x_3d)
        point_msg.point.y = float(y_3d)
        point_msg.point.z = float(z_3d)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                point_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            point_base = tf2_geometry_msgs.do_transform_point(point_msg, tf)
        except Exception as exc:
            self.get_logger().error(f"TF transform failed: {exc}")
            return

        dx = point_base.point.x
        dy = point_base.point.y
        dz = point_base.point.z
        dist = math.hypot(dx, dy)
        yaw = math.degrees(math.atan2(dy, dx))

        cam_tf = None
        try:
            cam_tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                point_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except Exception:
            cam_tf = None

        self.get_logger().info("---- OAKD CAN PROBE (ONE-SHOT) ----")
        self.get_logger().info(f"Depth image frame_id: '{depth_frame}'")
        self.get_logger().info(f"Depth camera_info frame_id: '{self.depth_info.header.frame_id}'")
        self.get_logger().info(f"BBox center (px): ({cx:.1f}, {cy:.1f}), size=({w:.1f} x {h:.1f})")
        self.get_logger().info(f"Depth image size: ({depth_w} x {depth_h})")
        self.get_logger().info(f"Depth pixel (u,v): ({u}, {v}) window=({u0}:{u1}, {v0}:{v1})")
        self.get_logger().info(f"Depth (m): {depth_m:.3f}")
        self.get_logger().info(
            "Depth cam intrinsics: fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f}".format(fx, fy, cx_cam, cy_cam)
        )
        self.get_logger().info(
            "Point in depth cam frame: ({:.3f}, {:.3f}, {:.3f})".format(x_3d, y_3d, z_3d)
        )
        self.get_logger().info(
            "Point in {}: ({:.3f}, {:.3f}, {:.3f})".format(self.target_frame, dx, dy, dz)
        )
        self.get_logger().info(
            "Distance in {} plane: {:.3f} m, yaw error: {:.1f} deg".format(self.target_frame, dist, yaw)
        )

        if cam_tf is not None:
            t = cam_tf.transform.translation
            self.get_logger().info(
                "{} origin in {}: ({:.3f}, {:.3f}, {:.3f})".format(point_msg.header.frame_id, self.target_frame, t.x, t.y, t.z)
            )

        if self.latest_can_detection is not None:
            try:
                det_tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.latest_can_detection.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
                det_base = tf2_geometry_msgs.do_transform_point(self.latest_can_detection, det_tf)
                ddx = det_base.point.x
                ddy = det_base.point.y
                ddz = det_base.point.z
                ddist = math.hypot(ddx, ddy)
                self.get_logger().info(
                    "/oakd/can_detection in {}: ({:.3f}, {:.3f}, {:.3f}) dist={:.3f} m".format(
                        self.target_frame, ddx, ddy, ddz, ddist
                    )
                )
            except Exception as exc:
                self.get_logger().error(f"TF transform failed for /oakd/can_detection: {exc}")

        # One-shot exit
        rclpy.shutdown()


def main():
    rclpy.init()
    node = OakdCanProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
