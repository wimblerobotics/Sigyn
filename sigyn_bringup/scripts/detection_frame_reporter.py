#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sigyn_interfaces.msg import DetectionArray, OakdDetectionArray
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class DetectionFrameReporter(Node):
    def __init__(self) -> None:
        super().__init__("detection_frame_reporter")

        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("oakd_topic", "/oakd/can_detections")
        self.declare_parameter("pi_detection_topic", "/gripper/camera/detections")
        self.declare_parameter("pi_point_topic", "/gripper/camera/can_detection")
        self.declare_parameter("pi_nominal_distance_m", 0.3)
        self.declare_parameter("pi_focal_length_px", 500.0)
        self.declare_parameter("pi_image_width_px", 640.0)
        self.declare_parameter("pi_image_height_px", 640.0)
        self.declare_parameter("report_period_sec", 0.5)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._oakd_topic = str(self.get_parameter("oakd_topic").value)
        self._pi_detection_topic = str(self.get_parameter("pi_detection_topic").value)
        self._pi_point_topic = str(self.get_parameter("pi_point_topic").value)
        self._pi_nominal_distance = float(self.get_parameter("pi_nominal_distance_m").value)
        self._pi_focal_length = float(self.get_parameter("pi_focal_length_px").value)
        self._pi_image_width = float(self.get_parameter("pi_image_width_px").value)
        self._pi_image_height = float(self.get_parameter("pi_image_height_px").value)
        self._report_period = float(self.get_parameter("report_period_sec").value)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._last_oakd_msg: Optional[OakdDetectionArray] = None
        self._last_pi_detection_msg: Optional[DetectionArray] = None
        self._last_pi_point_msg: Optional[PointStamped] = None
        self._last_missing_oakd_log_ns = 0
        self._last_missing_pi_detection_log_ns = 0
        self._last_missing_pi_point_log_ns = 0

        self.create_subscription(
            OakdDetectionArray,
            self._oakd_topic,
            self._on_oakd,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DetectionArray,
            self._pi_detection_topic,
            self._on_pi_detection,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            self._pi_point_topic,
            self._on_pi_point,
            qos_profile_sensor_data,
        )

        self.create_timer(self._report_period, self._report)
        self.get_logger().info(
            "Reporting detections in target_frame='%s' from OAK-D='%s', Pi DetectionArray='%s', Pi PointStamped='%s'"
            % (
                self._target_frame,
                self._oakd_topic,
                self._pi_detection_topic,
                self._pi_point_topic,
            )
        )

    def _on_oakd(self, msg: OakdDetectionArray) -> None:
        self._last_oakd_msg = msg

    def _on_pi_detection(self, msg: DetectionArray) -> None:
        self._last_pi_detection_msg = msg

    def _on_pi_point(self, msg: PointStamped) -> None:
        self._last_pi_point_msg = msg

    def _report(self) -> None:
        if self._last_oakd_msg is not None:
            self._report_oakd(self._last_oakd_msg)
        else:
            self._log_missing_once_per_5s(
                "oakd",
                "[OAKD] no messages received yet on '%s'" % self._oakd_topic,
            )

        if self._last_pi_detection_msg is not None:
            self._report_pi_detection_array(self._last_pi_detection_msg)
        else:
            self._log_missing_once_per_5s(
                "pi_detection",
                "[PI detection_array] no messages received yet on '%s'" % self._pi_detection_topic,
            )

        if self._last_pi_point_msg is not None:
            self._report_pi_point(self._last_pi_point_msg)
        else:
            self._log_missing_once_per_5s(
                "pi_point",
                "[PI point] no messages received yet on '%s'" % self._pi_point_topic,
            )

    def _report_oakd(self, msg: OakdDetectionArray) -> None:
        if not msg.detections:
            self.get_logger().info("[OAKD] no detections in latest frame")
            return

        best = max(msg.detections, key=lambda detection: detection.confidence)
        source_frame = best.header.frame_id or msg.header.frame_id
        point_in_target = self._transform_point(best.spatial_camera, source_frame, best.header.stamp)
        age_sec = self._age_seconds(best.header.stamp)
        if point_in_target is None:
            self.get_logger().warn(
                "[OAKD] TF failed from '%s' to '%s' raw=(%.3f, %.3f, %.3f) class='%s' score=%.2f age=%.2fs"
                % (
                    source_frame,
                    self._target_frame,
                    best.spatial_camera.x,
                    best.spatial_camera.y,
                    best.spatial_camera.z,
                    best.class_name,
                    best.confidence,
                    age_sec,
                )
            )
            return

        self.get_logger().info(
            "[OAKD] %s=(%.3f, %.3f, %.3f) from frame='%s' raw=(%.3f, %.3f, %.3f) bbox_center=(%d,%d) score=%.2f age=%.2fs"
            % (
                self._target_frame,
                point_in_target.x,
                point_in_target.y,
                point_in_target.z,
                source_frame,
                best.spatial_camera.x,
                best.spatial_camera.y,
                best.spatial_camera.z,
                best.bbox_center_x,
                best.bbox_center_y,
                best.confidence,
                age_sec,
            )
        )

    def _report_pi_detection_array(self, msg: DetectionArray) -> None:
        if not msg.detections:
            self.get_logger().info("[PI detection_array] no detections in latest frame")
            return

        best = max(msg.detections, key=lambda detection: detection.confidence)
        source_frame = best.header.frame_id or msg.header.frame_id
        age_sec = self._age_seconds(best.header.stamp if best.header.stamp.sec or best.header.stamp.nanosec else msg.header.stamp)

        raw_center = best.center
        transformed_center = self._transform_point(raw_center, source_frame, msg.header.stamp)

        bbox_center_x, bbox_center_y = self._bbox_center(best)
        reconstructed_point = None
        if bbox_center_x is not None and bbox_center_y is not None:
            reconstructed_raw = Point(
                x=(bbox_center_x - self._pi_image_width / 2.0) * self._pi_nominal_distance / self._pi_focal_length,
                y=(bbox_center_y - self._pi_image_height / 2.0) * self._pi_nominal_distance / self._pi_focal_length,
                z=self._pi_nominal_distance,
            )
            reconstructed_point = self._transform_point(reconstructed_raw, source_frame, msg.header.stamp)
        else:
            reconstructed_raw = None

        raw_center_norm = math.sqrt(raw_center.x ** 2 + raw_center.y ** 2 + raw_center.z ** 2)
        if transformed_center is not None:
            self.get_logger().info(
                "[PI detection_array raw-center] %s=(%.3f, %.3f, %.3f) from frame='%s' raw-center=(%.3f, %.3f, %.3f) |raw|=%.3f score=%.2f age=%.2fs"
                % (
                    self._target_frame,
                    transformed_center.x,
                    transformed_center.y,
                    transformed_center.z,
                    source_frame,
                    raw_center.x,
                    raw_center.y,
                    raw_center.z,
                    raw_center_norm,
                    best.confidence,
                    age_sec,
                )
            )
        else:
            self.get_logger().warn(
                "[PI detection_array raw-center] TF failed from '%s' to '%s' raw-center=(%.3f, %.3f, %.3f) score=%.2f age=%.2fs"
                % (
                    source_frame,
                    self._target_frame,
                    raw_center.x,
                    raw_center.y,
                    raw_center.z,
                    best.confidence,
                    age_sec,
                )
            )

        if reconstructed_raw is not None and reconstructed_point is not None:
            self.get_logger().info(
                "[PI detection_array BT-style] %s=(%.3f, %.3f, %.3f) from frame='%s' reconstructed=(%.3f, %.3f, %.3f) bbox_center=(%.1f,%.1f) nominal_z=%.3f"
                % (
                    self._target_frame,
                    reconstructed_point.x,
                    reconstructed_point.y,
                    reconstructed_point.z,
                    source_frame,
                    reconstructed_raw.x,
                    reconstructed_raw.y,
                    reconstructed_raw.z,
                    bbox_center_x,
                    bbox_center_y,
                    self._pi_nominal_distance,
                )
            )
        elif reconstructed_raw is not None:
            self.get_logger().warn(
                "[PI detection_array BT-style] TF failed from '%s' to '%s' reconstructed=(%.3f, %.3f, %.3f) bbox_center=(%.1f,%.1f)"
                % (
                    source_frame,
                    self._target_frame,
                    reconstructed_raw.x,
                    reconstructed_raw.y,
                    reconstructed_raw.z,
                    bbox_center_x,
                    bbox_center_y,
                )
            )

    def _report_pi_point(self, msg: PointStamped) -> None:
        point_in_target = self._transform_point(msg.point, msg.header.frame_id, msg.header.stamp)
        age_sec = self._age_seconds(msg.header.stamp)
        if point_in_target is None:
            self.get_logger().warn(
                "[PI point] TF failed from '%s' to '%s' raw=(%.3f, %.3f, %.3f) age=%.2fs"
                % (
                    msg.header.frame_id,
                    self._target_frame,
                    msg.point.x,
                    msg.point.y,
                    msg.point.z,
                    age_sec,
                )
            )
            return

        self.get_logger().info(
            "[PI point] %s=(%.3f, %.3f, %.3f) from frame='%s' raw=(%.3f, %.3f, %.3f) age=%.2fs"
            % (
                self._target_frame,
                point_in_target.x,
                point_in_target.y,
                point_in_target.z,
                msg.header.frame_id,
                msg.point.x,
                msg.point.y,
                msg.point.z,
                age_sec,
            )
        )

    def _transform_point(self, point: Point, source_frame: str, stamp) -> Optional[Point]:
        if not source_frame:
            return None

        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.header.stamp = stamp
        point_stamped.point = point

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=0.1),
            )
            transformed = do_transform_point(point_stamped, transform)
            return transformed.point
        except TransformException:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                transformed = do_transform_point(point_stamped, transform)
                return transformed.point
            except TransformException as exc:
                self.get_logger().debug(
                    "TF failed from '%s' to '%s': %s" % (source_frame, self._target_frame, exc)
                )
                return None

    def _bbox_center(self, detection) -> Tuple[Optional[float], Optional[float]]:
        has_bbox = any(
            abs(value) > 1.0e-6
            for value in (detection.x_min, detection.y_min, detection.x_max, detection.y_max)
        )
        if has_bbox:
            return (
                (float(detection.x_min) + float(detection.x_max)) / 2.0,
                (float(detection.y_min) + float(detection.y_max)) / 2.0,
            )

        center = detection.center
        if abs(center.z) < 1.0e-6 and (abs(center.x) > 1.0 or abs(center.y) > 1.0):
            return float(center.x), float(center.y)

        return None, None

    def _age_seconds(self, stamp) -> float:
        try:
            return (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1.0e9
        except Exception:
            return float("nan")

    def _log_missing_once_per_5s(self, key: str, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        attr_name = {
            "oakd": "_last_missing_oakd_log_ns",
            "pi_detection": "_last_missing_pi_detection_log_ns",
            "pi_point": "_last_missing_pi_point_log_ns",
        }[key]
        last_ns = getattr(self, attr_name)
        if now_ns - last_ns >= int(5.0e9):
            self.get_logger().info(message)
            setattr(self, attr_name, now_ns)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionFrameReporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()