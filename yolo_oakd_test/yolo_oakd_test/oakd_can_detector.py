#!/home/ros/sigyn-venv/bin/python3
"""
OAK-D Can Detector Node - Ultralytics Single-Output Support
Modified to use dai.node.NeuralNetwork with custom YOLOv5 output parsing
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
import depthai as dai
import cv2
import numpy as np
import threading
import time
import traceback
import math
import itertools
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# Custom message
try:
    from yolo_oakd_test.msg import CanDetection
except ImportError:
    CanDetection = None

class OakdCanDetector(Node):
    def __init__(self):
        super().__init__('oakd_can_detector')
        
        self.declare_parameter('blob_path', '')
        self.blob_path = self.get_parameter('blob_path').value
        
        if not self.blob_path:
            self.get_logger().error("No blob_path provided via parameter!")
            # Fallback for testing
            self.blob_path = "/home/ros/sigyn_ws/src/Sigyn/yolo_oakd_test/models/can_detector.blob"
            
        self.get_logger().info(f"[[INIT]] Using blob: {self.blob_path}")

        # Parameters
        self.declare_parameter('camera_frame', 'oak_rgb_camera_optical_frame')
        self.declare_parameter('spatial_axis_map', 'x,y,z')
        self.declare_parameter('log_tf_debug', True)
        self.declare_parameter('debug_logging', False)
        self.declare_parameter('debug_log_interval_sec', 2.0)
        self.declare_parameter('depth_publish_every', 5)
        self.declare_parameter('expected_target_base', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('suggest_axis_map', True)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.spatial_axis_map = self.get_parameter('spatial_axis_map').value
        self.log_tf_debug = self.get_parameter('log_tf_debug').value
        self.debug_logging = bool(self.get_parameter('debug_logging').value)
        self.debug_log_interval_sec = float(self.get_parameter('debug_log_interval_sec').value)
        self.depth_publish_every = max(1, int(self.get_parameter('depth_publish_every').value))
        self.expected_target_base = self.get_parameter_or(
            'expected_target_base',
            Parameter('expected_target_base', Parameter.Type.DOUBLE_ARRAY, [])
        ).value
        self.suggest_axis_map = self.get_parameter('suggest_axis_map').value

        # Publishers
        self.pub_rgb = self.create_publisher(Image, '/oakd_top/rgb_preview', 10)
        self.pub_depth = self.create_publisher(Image, '/oakd_top/depth_image', 10)
        if CanDetection:
            self.get_logger().info("[[INIT]] CanDetection message type loaded successfully.")
            self.pub_det = self.create_publisher(CanDetection, '/oakd_top/can_detections', 10)
        else:
            self.get_logger().warn("[[INIT]] CanDetection message type NOT loaded.")
            self.pub_det = None
            
        self.pub_viz = self.create_publisher(Image, '/oakd_top/annotated_image', 10)
        self.pub_marker = self.create_publisher(PointStamped, '/oakd_top/can_point_camera', 10)
        self.pub_marker_raw = self.create_publisher(PointStamped, '/oakd_top/can_point_camera_raw', 10)
        # Heartbeat publisher for WaitForDetection BT node
        self.pub_heartbeat = self.create_publisher(Detection2DArray, '/oakd/object_detector_heartbeat', 10)
        
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Detection parameters (for Ultralytics single-output parsing)
        # ModelConverter reports output layout NCD: [1, 5, N].
        self.confidence_threshold = 0.65
        self.iou_threshold = 0.45
        self.max_candidates_before_nms = 300
        self.max_final_detections = 1
        self.min_box_size_px = 6
        self.input_width = 416
        self.input_height = 416
        self.output_layout = "NCD"
        self._logged_layer_info = False
        self._last_tf_warn_time = 0.0
        self._last_debug_log_time = 0.0

        # DepthAI Pipeline Control
        self.running = True
        self.thread = threading.Thread(target=self.run_pipeline)
        self.thread.start()
        self.get_logger().info("[[INIT]] Pipeline thread started.")
        self.get_logger().info(f"[[INIT]] camera_frame='{self.camera_frame}', spatial_axis_map='{self.spatial_axis_map}', log_tf_debug={self.log_tf_debug}")
        if isinstance(self.expected_target_base, list) and len(self.expected_target_base) == 3:
            self.get_logger().info(
                f"[[INIT]] expected_target_base={self.expected_target_base}, suggest_axis_map={self.suggest_axis_map}"
            )

    def nms(self, boxes, scores, iou_threshold=0.5):
        """Non-maximum suppression for post-processing detections."""
        if len(boxes) == 0:
            return []
        
        boxes = np.array(boxes)
        scores = np.array(scores)
        
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]
        
        return keep

    @staticmethod
    def _quat_to_rpy(q):
        # Convert quaternion to roll, pitch, yaw
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _parse_axis_map(self, map_str):
        tokens = [t.strip() for t in map_str.split(',') if t.strip()]
        if len(tokens) != 3:
            self.get_logger().warn(f"[[AXIS_MAP]] Invalid spatial_axis_map '{map_str}', using 'x,y,z'.")
            return [(0, 1), (1, 1), (2, 1)]

        axis_index = {'x': 0, 'y': 1, 'z': 2}
        mapping = []
        for token in tokens:
            sign = -1 if token.startswith('-') else 1
            axis = token[1:] if token.startswith('-') else token
            if axis not in axis_index:
                self.get_logger().warn(f"[[AXIS_MAP]] Invalid token '{token}', using identity.")
                return [(0, 1), (1, 1), (2, 1)]
            mapping.append((axis_index[axis], sign))
        return mapping

    @staticmethod
    def _axis_map_to_string(perm, signs):
        axis_names = ['x', 'y', 'z']
        parts = []
        for i in range(3):
            axis = axis_names[perm[i]]
            parts.append(f"-{axis}" if signs[i] < 0 else axis)
        return ",".join(parts)

    @staticmethod
    def _best_axis_map(raw_vec, target_vec):
        best_err = None
        best_perm = None
        best_signs = None

        for perm in itertools.permutations(range(3)):
            for signs in itertools.product([1, -1], repeat=3):
                mapped = [signs[i] * raw_vec[perm[i]] for i in range(3)]
                err = sum((mapped[i] - target_vec[i]) ** 2 for i in range(3))
                if best_err is None or err < best_err:
                    best_err = err
                    best_perm = perm
                    best_signs = signs

        if best_perm is None:
            return None, None

        return OakdCanDetector._axis_map_to_string(best_perm, best_signs), best_err

    def run_pipeline(self):
        pipeline = dai.Pipeline()
        
        try:
            # 1. RGB Camera
            camRgb = pipeline.create(dai.node.ColorCamera)
            camRgb.setPreviewSize(self.input_width, self.input_height)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setInterleaved(False)
            # Keep display/annotation path in OpenCV-native BGR.
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            camRgb.setFps(15)
            self.get_logger().info("[[PIPELINE]] RGB Camera node created.")
            
            # 2. Mono Cameras
            monoLeft = pipeline.create(dai.node.MonoCamera)
            monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
            
            monoRight = pipeline.create(dai.node.MonoCamera)
            monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            self.get_logger().info("[[PIPELINE]] Mono Camera nodes created.")
            
            # 3. Stereo Depth
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
            stereo.setSubpixel(False)
            self.get_logger().info("[[PIPELINE]] Stereo Depth node created.")
            
            # 4. Generic Neural Network (Ultralytics single-output)
            nn = pipeline.create(dai.node.NeuralNetwork)
            nn.setBlobPath(self.blob_path)
            nn.setNumInferenceThreads(2)
            nn.input.setBlocking(False)

            # Convert camera preview to RGB for NN input (export config expects RGB).
            manip_nn = pipeline.create(dai.node.ImageManip)
            manip_nn.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)
            manip_nn.setMaxOutputFrameSize(self.input_width * self.input_height * 3)
            
            self.get_logger().info(f"[[PIPELINE]] Generic Neural Network configured for Ultralytics YOLOv5 (confidence_threshold={self.confidence_threshold}, iou_threshold={self.iou_threshold}).")
            
            # Outputs
            xoutRgb = pipeline.create(dai.node.XLinkOut)
            xoutRgb.setStreamName("rgb")
            
            xoutNN = pipeline.create(dai.node.XLinkOut)
            xoutNN.setStreamName("nn")
            
            xoutDepth = pipeline.create(dai.node.XLinkOut)
            xoutDepth.setStreamName("depth")
            
            # Linking
            monoLeft.out.link(stereo.left)
            monoRight.out.link(stereo.right)

            camRgb.preview.link(manip_nn.inputImage)
            manip_nn.out.link(nn.input)
            camRgb.preview.link(xoutRgb.input)
            
            nn.out.link(xoutNN.input)
            stereo.depth.link(xoutDepth.input)
            
            self.get_logger().info("[[PIPELINE]] Links established. Starting device...")
            
            with dai.Device(pipeline) as device:
                self.get_logger().info(f"[[DEVICE]] Connected to OAK-D: {device.getMxId()}")
                self.get_logger().info(f"[[DEVICE]] USB Speed: {device.getUsbSpeed()}")
                
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
                q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                
                self.get_logger().info("[[LOOP]] Entering main loop.")
                
                loop_count = 0
                latest_frame_bgr = None
                latest_frame_depth = None
                depth_pub_counter = 0

                while self.running and rclpy.ok():
                    loop_count += 1

                    in_rgb = q_rgb.tryGet()
                    in_nn = q_nn.tryGet()
                    in_depth = q_depth.tryGet()
                    
                    if in_rgb:
                        latest_frame_bgr = in_rgb.getCvFrame()
                        msg_img = self.bridge.cv2_to_imgmsg(latest_frame_bgr, "bgr8")
                        msg_img.header.stamp = self.get_clock().now().to_msg()
                        msg_img.header.frame_id = "oak_rgb_camera_optical_frame"
                        self.pub_rgb.publish(msg_img)

                        # Keep annotated stream alive even before detections/depth are available.
                        self.pub_viz.publish(msg_img)
                    
                    if in_depth:
                        latest_frame_depth = in_depth.getFrame()
                        depth_pub_counter += 1
                        if depth_pub_counter % self.depth_publish_every == 0:
                            # Depth visualization is CPU-heavy; publish at reduced cadence.
                            depth_down = latest_frame_depth[::4, ::4]
                            if depth_down.max() > 0:
                                depth_vis = (depth_down / depth_down.max() * 255).astype(np.uint8)
                                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                                msg_depth = self.bridge.cv2_to_imgmsg(depth_vis, "bgr8")
                                msg_depth.header.stamp = self.get_clock().now().to_msg()
                                msg_depth.header.frame_id = "oak_rgb_camera_optical_frame"
                                self.pub_depth.publish(msg_depth)

                    if in_nn and latest_frame_bgr is not None:
                        # Parse raw YOLOv5 output: [1, 5+num_classes, num_detections]
                        # For 1 class at 416x416: [1, 5, 3549]
                        try:
                            debug_now = False
                            if self.debug_logging:
                                now_s = time.time()
                                if now_s - self._last_debug_log_time >= self.debug_log_interval_sec:
                                    debug_now = True
                                    self._last_debug_log_time = now_s

                            # Use explicit output layer when available.
                            layer_names = in_nn.getAllLayerNames()
                            if not layer_names:
                                continue

                            if not self._logged_layer_info:
                                self.get_logger().info(f"[[DEBUG]] NN output layers: {layer_names}")
                                self._logged_layer_info = True
                            output_layer = "output0" if "output0" in layer_names else layer_names[0]
                            
                            output_tensor = in_nn.getLayerFp16(output_layer)
                            output_array = np.array(output_tensor).astype(np.float32)
                            
                            # Log raw shape for debugging
                            if debug_now:
                                self.get_logger().info(f"[[DEBUG]] Raw output shape: {output_array.shape}, size: {output_array.size}, dtype: {output_array.dtype}")
                            
                            # Reshape to [num_detections, 5+num_classes]
                            # Expected shape: (1, 5, 3549) -> (3549, 5)
                            if output_array.size == 0:
                                continue
                            
                            # Handle different output shapes
                            if len(output_array.shape) == 1:
                                # ModelConverter config reports NCD output [1, 5, N].
                                total_elements = output_array.size
                                num_values_per_det = 5
                                if total_elements % num_values_per_det != 0:
                                    self.get_logger().warn(f"[[DEBUG]] Unexpected flat output size={total_elements}, not divisible by 5")
                                    continue
                                num_detections = total_elements // num_values_per_det
                                output_array = output_array.reshape(num_values_per_det, num_detections).T
                                if debug_now:
                                    self.get_logger().info(f"[[DEBUG]] Reshaped flat output ({self.output_layout}) to: {output_array.shape}")
                            elif len(output_array.shape) == 3:
                                # Shape: (1, 5, 3549) -> (3549, 5)
                                output_array = output_array.transpose(0, 2, 1).reshape(-1, 5)
                                if debug_now:
                                    self.get_logger().info(f"[[DEBUG]] Reshaped from 3D to: {output_array.shape}")
                            elif len(output_array.shape) == 2:
                                # Already in 2D shape
                                if output_array.shape[1] == 5:
                                    pass  # Shape: (3549, 5) - correct
                                elif output_array.shape[0] == 5:
                                    output_array = output_array.T  # Shape: (5, 3549) -> (3549, 5)
                                if debug_now:
                                    self.get_logger().info(f"[[DEBUG]] Already 2D shape: {output_array.shape}")
                            
                            raw_confidences = output_array[:, 4]
                            
                            if debug_now:
                                self.get_logger().info(f"[[DEBUG]] Raw confidence range: [{raw_confidences.min():.2f}, {raw_confidences.max():.2f}]")
                            
                            def sigmoid(x):
                                return 1 / (1 + np.exp(-np.clip(x, -88, 88)))

                            # Use direct probabilities if already in [0,1], otherwise treat as logits.
                            if raw_confidences.min() < 0.0 or raw_confidences.max() > 1.0:
                                probabilities = sigmoid(raw_confidences)
                            else:
                                probabilities = raw_confidences

                            mask = probabilities > self.confidence_threshold
                            filtered_dets = output_array[mask]
                            filtered_confidences = probabilities[mask]
                            
                            if len(filtered_dets) == 0:
                                # Publish passthrough annotated image + empty heartbeat
                                msg_ann = self.bridge.cv2_to_imgmsg(latest_frame_bgr, "bgr8")
                                msg_ann.header.stamp = self.get_clock().now().to_msg()
                                msg_ann.header.frame_id = "oak_rgb_camera_optical_frame"
                                self.pub_viz.publish(msg_ann)

                                hb_msg = Detection2DArray()
                                hb_msg.header = msg_ann.header
                                self.pub_heartbeat.publish(hb_msg)
                                continue
                            
                            if debug_now:
                                self.get_logger().info(f"[[DEBUG]] Filtered detections: {len(filtered_dets)}, prob range: [{filtered_confidences.min():.3f}, {filtered_confidences.max():.3f}]")
                                self.get_logger().info(f"[[DEBUG]] Sample detection: x={filtered_dets[0,0]:.3f}, y={filtered_dets[0,1]:.3f}, w={filtered_dets[0,2]:.3f}, h={filtered_dets[0,3]:.3f}, raw_conf={filtered_dets[0,4]:.3f}")

                            # Keep top-K candidates before NMS to prevent flooding when parser is imperfect
                            if len(filtered_dets) > self.max_candidates_before_nms:
                                top_idx = np.argsort(filtered_confidences)[-self.max_candidates_before_nms:]
                                filtered_dets = filtered_dets[top_idx]
                                filtered_confidences = filtered_confidences[top_idx]
                            
                            # Assume [x_center, y_center, w, h] and scale when coordinates are normalized.
                            boxes_xywh = filtered_dets[:, :4].copy()

                            # Some exports produce normalized coordinates in [0,1]. Detect and scale.
                            if np.percentile(np.abs(boxes_xywh[:, :2]), 95) <= 1.5 and np.percentile(np.abs(boxes_xywh[:, 2:4]), 95) <= 1.5:
                                boxes_xywh[:, 0] *= self.input_width
                                boxes_xywh[:, 1] *= self.input_height
                                boxes_xywh[:, 2] *= self.input_width
                                boxes_xywh[:, 3] *= self.input_height

                            # Clamp to image bounds
                            boxes_xywh[:, 0] = np.clip(boxes_xywh[:, 0], 0, self.input_width - 1)
                            boxes_xywh[:, 1] = np.clip(boxes_xywh[:, 1], 0, self.input_height - 1)
                            boxes_xywh[:, 2] = np.clip(boxes_xywh[:, 2], 1, self.input_width)
                            boxes_xywh[:, 3] = np.clip(boxes_xywh[:, 3], 1, self.input_height)

                            # Remove tiny boxes that are usually decode artifacts
                            size_mask = (boxes_xywh[:, 2] >= self.min_box_size_px) & (boxes_xywh[:, 3] >= self.min_box_size_px)
                            boxes_xywh = boxes_xywh[size_mask]
                            filtered_confidences = filtered_confidences[size_mask]
                            if len(boxes_xywh) == 0:
                                msg_ann = self.bridge.cv2_to_imgmsg(latest_frame_bgr, "bgr8")
                                msg_ann.header.stamp = self.get_clock().now().to_msg()
                                msg_ann.header.frame_id = "oak_rgb_camera_optical_frame"
                                self.pub_viz.publish(msg_ann)
                                continue
                            
                            if debug_now and len(boxes_xywh) > 0:
                                self.get_logger().info(f"[[DEBUG]] After clamping: x={boxes_xywh[0,0]:.1f}, y={boxes_xywh[0,1]:.1f}, w={boxes_xywh[0,2]:.1f}, h={boxes_xywh[0,3]:.1f}")
                            
                            # Convert to xyxy format for NMS
                            boxes_xyxy = np.zeros_like(boxes_xywh)
                            boxes_xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2  # x1
                            boxes_xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2  # y1
                            boxes_xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2  # x2
                            boxes_xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2  # y2
                            
                            # Apply NMS
                            keep_indices = self.nms(boxes_xyxy, filtered_confidences, self.iou_threshold)
                            
                            if len(keep_indices) == 0:
                                msg_ann = self.bridge.cv2_to_imgmsg(latest_frame_bgr, "bgr8")
                                msg_ann.header.stamp = self.get_clock().now().to_msg()
                                msg_ann.header.frame_id = "oak_rgb_camera_optical_frame"
                                self.pub_viz.publish(msg_ann)
                                continue
                            
                            final_boxes = boxes_xyxy[keep_indices]
                            final_scores = filtered_confidences[keep_indices]
                            final_centers = boxes_xywh[keep_indices, :2]  # [x_center, y_center]

                            if len(final_scores) > self.max_final_detections:
                                order = np.argsort(final_scores)[::-1][:self.max_final_detections]
                                final_boxes = final_boxes[order]
                                final_scores = final_scores[order]
                                final_centers = final_centers[order]
                            
                            # Process detections
                            annotated = None
                            if latest_frame_bgr is not None:
                                annotated = latest_frame_bgr.copy()
                            
                            for i, (box, score, center) in enumerate(zip(final_boxes, final_scores, final_centers)):
                                # Get depth at detection center
                                cx = int(center[0])
                                cy = int(center[1])
                                
                                # Clamp to image bounds
                                cx = max(0, min(cx, self.input_width - 1))
                                cy = max(0, min(cy, self.input_height - 1))
                                
                                # Sample depth (depth is aligned to RGB) when available
                                z_mm = 0.0 if latest_frame_depth is None else latest_frame_depth[cy, cx]
                                
                                if z_mm == 0.0:
                                    # Draw 2D detection even if depth is unavailable/invalid
                                    if annotated is not None:
                                        x1, y1, x2, y2 = box.astype(int)
                                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 255), 2)
                                        cv2.putText(annotated, f"2D C:{score:.2f}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                                    continue
                                
                                # Compute spatial coordinates from depth
                                # DepthAI camera intrinsics approximation for OAK-D Lite at 416x416
                                # Typical HFOV ~73 degrees
                                focal_length = self.input_width / (2 * math.tan(math.radians(73 / 2)))
                                
                                # Convert pixel coordinates to camera coordinates (millimeters)
                                x_mm = (cx - self.input_width / 2) * z_mm / focal_length
                                y_mm = (cy - self.input_height / 2) * z_mm / focal_length
                                
                                # Draw accepted detection
                                if annotated is not None:
                                    x1, y1, x2, y2 = box.astype(int)
                                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                    label_str = f"Z: {z_mm/1000.0:.2f}m C:{score:.2f}"
                                    cv2.putText(annotated, label_str, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
                                # Pass bounding box to process_detection
                                bbox = box.astype(int).tolist()  # [x1, y1, x2, y2]
                                self.process_detection(x_mm, y_mm, z_mm, float(score), bbox)
                            
                            # Publish annotated image
                            if annotated is not None:
                                msg_ann = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                                msg_ann.header.stamp = self.get_clock().now().to_msg()
                                msg_ann.header.frame_id = "oak_rgb_camera_optical_frame"
                                self.pub_viz.publish(msg_ann)

                                # Publish heartbeat
                                hb_msg = Detection2DArray()
                                hb_msg.header = msg_ann.header
                                self.pub_heartbeat.publish(hb_msg)
                        
                        except Exception as e:
                            self.get_logger().error(f"[[DETECTION]] Error parsing network output: {e}")
                            self.get_logger().error(traceback.format_exc())

                    time.sleep(0.001)
                    
        except Exception as e:
            self.get_logger().error(f"[[FATAL]] Pipeline failed: {e}")
            self.get_logger().error(traceback.format_exc())

    def process_detection(self, x_mm, y_mm, z_mm, confidence, bbox=None):
            raw_x_m = x_mm / 1000.0
            raw_y_m = y_mm / 1000.0
            raw_z_m = z_mm / 1000.0

            axis_map = self._parse_axis_map(self.spatial_axis_map)
            raw_vec = [raw_x_m, raw_y_m, raw_z_m]
            mapped_vec = [sign * raw_vec[idx] for idx, sign in axis_map]

            stamp = self.get_clock().now().to_msg()

            pt_raw = PointStamped()
            pt_raw.header.frame_id = self.camera_frame
            pt_raw.header.stamp = stamp
            pt_raw.point.x = raw_x_m
            pt_raw.point.y = raw_y_m
            pt_raw.point.z = raw_z_m
            self.pub_marker_raw.publish(pt_raw)

            pt_msg = PointStamped()
            pt_msg.header.frame_id = self.camera_frame
            pt_msg.header.stamp = stamp
            pt_msg.point.x = mapped_vec[0]
            pt_msg.point.y = mapped_vec[1]
            pt_msg.point.z = mapped_vec[2]
            self.pub_marker.publish(pt_msg)
            
            try:
                if self.tf_buffer.can_transform("base_link", self.camera_frame, rclpy.time.Time()):
                    t = self.tf_buffer.lookup_transform("base_link", self.camera_frame, rclpy.time.Time())

                    if self.log_tf_debug:
                        roll, pitch, yaw = self._quat_to_rpy(t.transform.rotation)

                    pt_base_raw = tf2_geometry_msgs.do_transform_point(pt_raw, t)
                    pt_base = tf2_geometry_msgs.do_transform_point(pt_msg, t)

                    # Optional: suggest axis mapping based on expected base_link target
                    if self.suggest_axis_map and isinstance(self.expected_target_base, list) and len(self.expected_target_base) == 3:
                        try:
                            target_msg = PointStamped()
                            target_msg.header.frame_id = "base_link"
                            target_msg.header.stamp = pt_msg.header.stamp
                            target_msg.point.x = float(self.expected_target_base[0])
                            target_msg.point.y = float(self.expected_target_base[1])
                            target_msg.point.z = float(self.expected_target_base[2])

                            t_inv = self.tf_buffer.lookup_transform(self.camera_frame, "base_link", rclpy.time.Time())
                            target_in_cam = tf2_geometry_msgs.do_transform_point(target_msg, t_inv)

                            raw_vec = [raw_x_m, raw_y_m, raw_z_m]
                            tgt_vec = [target_in_cam.point.x, target_in_cam.point.y, target_in_cam.point.z]

                            best_map, best_err = self._best_axis_map(raw_vec, tgt_vec)
                        except Exception as exc:
                            self.get_logger().warn(f"[[SUGGEST]] Failed to compute axis map suggestion: {exc}")

                    if self.pub_det:
                        det_msg = CanDetection()
                        det_msg.header = pt_base.header
                        det_msg.class_name = "can"
                        det_msg.confidence = float(confidence)
                        
                        # Bounding box fields
                        if bbox is not None:
                            det_msg.bbox_xmin = int(bbox[0])
                            det_msg.bbox_ymin = int(bbox[1])
                            det_msg.bbox_xmax = int(bbox[2])
                            det_msg.bbox_ymax = int(bbox[3])
                            det_msg.bbox_center_x = int((bbox[0] + bbox[2]) / 2)
                            det_msg.bbox_center_y = int((bbox[1] + bbox[3]) / 2)
                        else:
                            det_msg.bbox_xmin = 0
                            det_msg.bbox_ymin = 0
                            det_msg.bbox_xmax = 0
                            det_msg.bbox_ymax = 0
                            det_msg.bbox_center_x = 0
                            det_msg.bbox_center_y = 0
                        
                        # Spatial coordinates
                        det_msg.spatial_camera.x = float(mapped_vec[0])
                        det_msg.spatial_camera.y = float(mapped_vec[1])
                        det_msg.spatial_camera.z = float(mapped_vec[2])
                        det_msg.spatial_base_link = pt_base.point
                        
                        # Distance from camera
                        det_msg.distance_from_camera = float(raw_z_m)
                        
                        # Debug log (empty for now)
                        det_msg.debug_log = []
                        
                        self.pub_det.publish(det_msg)
                else:
                    now_s = time.time()
                    if now_s - self._last_tf_warn_time > 5.0:
                        self.get_logger().warn(f"[[TRANSFORM]] Cannot transform from '{self.camera_frame}' to base_link yet.")
                        self._last_tf_warn_time = now_s

                    # Still publish camera-frame detection when TF is unavailable.
                    if self.pub_det:
                        det_msg = CanDetection()
                        det_msg.header.stamp = stamp
                        det_msg.header.frame_id = self.camera_frame
                        det_msg.class_name = "can"
                        det_msg.confidence = float(confidence)

                        if bbox is not None:
                            det_msg.bbox_xmin = int(bbox[0])
                            det_msg.bbox_ymin = int(bbox[1])
                            det_msg.bbox_xmax = int(bbox[2])
                            det_msg.bbox_ymax = int(bbox[3])
                            det_msg.bbox_center_x = int((bbox[0] + bbox[2]) / 2)
                            det_msg.bbox_center_y = int((bbox[1] + bbox[3]) / 2)
                        else:
                            det_msg.bbox_xmin = 0
                            det_msg.bbox_ymin = 0
                            det_msg.bbox_xmax = 0
                            det_msg.bbox_ymax = 0
                            det_msg.bbox_center_x = 0
                            det_msg.bbox_center_y = 0

                        det_msg.spatial_camera.x = float(mapped_vec[0])
                        det_msg.spatial_camera.y = float(mapped_vec[1])
                        det_msg.spatial_camera.z = float(mapped_vec[2])
                        det_msg.spatial_base_link.x = float(mapped_vec[0])
                        det_msg.spatial_base_link.y = float(mapped_vec[1])
                        det_msg.spatial_base_link.z = float(mapped_vec[2])
                        det_msg.distance_from_camera = float(raw_z_m)
                        det_msg.debug_log = ["no_tf_base_link"]
                        self.pub_det.publish(det_msg)
            except Exception as e:
                self.get_logger().error(f"[[TRANSFORM]] Error during transformation: {e}")

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = OakdCanDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
