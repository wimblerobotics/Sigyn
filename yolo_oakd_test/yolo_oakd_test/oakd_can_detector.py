#!/home/ros/sigyn-venv/bin/python3
"""
OAK-D Can Detector Node - Debugging Mode
Heavily instrumented for tracing.
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
        self.declare_parameter('expected_target_base', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('suggest_axis_map', True)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.spatial_axis_map = self.get_parameter('spatial_axis_map').value
        self.log_tf_debug = self.get_parameter('log_tf_debug').value
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
        # self.get_logger().info("[[PIPELINE]] Building pipeline...")
        pipeline = dai.Pipeline()
        
        try:
            # 1. RGB Camera
            camRgb = pipeline.create(dai.node.ColorCamera)
            camRgb.setPreviewSize(640, 640)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setInterleaved(False)
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
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
            stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
            stereo.setSubpixel(True)
            # stereo.setLeftRightCheck(True) # Disabled to match test_04
            self.get_logger().info("[[PIPELINE]] Stereo Depth node created.")
            
            # 4. Spatial YOLO
            spatialDet = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
            spatialDet.setBlobPath(self.blob_path)
            
            # Confidence Threshold (restored to 0.4 to match working test_04)
            CONF_THRESH = 0.4 
            spatialDet.setConfidenceThreshold(CONF_THRESH)
            self.get_logger().info(f"[[PIPELINE]] Spatial YOLO Confidence Threshold set to: {CONF_THRESH}")
            
            spatialDet.input.setBlocking(False)
            
            # REMOVED: camRgb.setPreviewKeepAspectRatio(False) 
            # We want to default to True (Center Crop) to match test_04 and avoid distortion.
            
            # Spatial Config
            spatialDet.setBoundingBoxScaleFactor(0.5)
            spatialDet.setDepthLowerThreshold(100)
            spatialDet.setDepthUpperThreshold(5000)
            
            # YOLO Config 
            spatialDet.setNumClasses(1)
            spatialDet.setCoordinateSize(4)
            spatialDet.setAnchors([10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326])
            spatialDet.setAnchorMasks({"side80": [0, 1, 2], "side40": [3, 4, 5], "side20": [6, 7, 8]})
            spatialDet.setIouThreshold(0.5)
            self.get_logger().info("[[PIPELINE]] Spatial YOLO configured.")
            
            # Outputs
            xoutRgb = pipeline.create(dai.node.XLinkOut)
            xoutRgb.setStreamName("rgb")
            
            xoutDet = pipeline.create(dai.node.XLinkOut)
            xoutDet.setStreamName("detections")
            
            xoutDepth = pipeline.create(dai.node.XLinkOut)
            xoutDepth.setStreamName("depth")
            
            # Linking
            monoLeft.out.link(stereo.left)
            monoRight.out.link(stereo.right)
            
            camRgb.preview.link(spatialDet.input)
            camRgb.preview.link(xoutRgb.input)
            
            stereo.depth.link(spatialDet.inputDepth)
            spatialDet.out.link(xoutDet.input)
            spatialDet.passthroughDepth.link(xoutDepth.input)
            
            self.get_logger().info("[[PIPELINE]] Links established. Starting device...")
            
            with dai.Device(pipeline) as device:
                self.get_logger().info(f"[[DEVICE]] Connected to OAK-D: {device.getMxId()}")
                self.get_logger().info(f"[[DEVICE]] USB Speed: {device.getUsbSpeed()}")
                
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
                q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                
                self.get_logger().info("[[LOOP]] Entering main loop.")
                
                loop_count = 0
                latest_frame_rgb = None

                while self.running and rclpy.ok():
                    loop_count += 1
                    
                    # Log heartbeat every 100 loops (~3-5 seconds depending on fps)
                    # if loop_count % 100 == 0:
                        #  self.get_logger().info(f"[[LOOP]] Heartbeat {loop_count}. Running...")

                    in_rgb = q_rgb.tryGet()
                    in_det = q_det.tryGet()
                    in_depth = q_depth.tryGet()
                    
                    if in_rgb:
                        latest_frame_rgb = in_rgb.getCvFrame()
                        # if loop_count % 30 == 0:
                            # self.get_logger().info("[[DATA]] RGB Frame received.")
                            
                        msg_img = self.bridge.cv2_to_imgmsg(latest_frame_rgb, "bgr8")
                        msg_img.header.stamp = self.get_clock().now().to_msg()
                        msg_img.header.frame_id = "oak_rgb_camera_optical_frame"
                        self.pub_rgb.publish(msg_img)
                    
                    if in_depth:
                        frame_depth = in_depth.getFrame()
                        # Simple visualizer for depth
                        depth_down = frame_depth[::4, ::4]
                        if depth_down.max() > 0:
                            depth_vis = (depth_down / depth_down.max() * 255).astype(np.uint8)
                            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                            msg_depth = self.bridge.cv2_to_imgmsg(depth_vis, "bgr8")
                            msg_depth.header.stamp = self.get_clock().now().to_msg()
                            msg_depth.header.frame_id = "oak_rgb_camera_optical_frame"
                            self.pub_depth.publish(msg_depth)

                    if in_det:
                        detections = in_det.detections
                        # if len(detections) > 0:
                            #  self.get_logger().info(f"[[DETECTION]] Packet received with {len(detections)} detection(s).")
                        
                        annotated = None
                        if latest_frame_rgb is not None:
                             annotated = latest_frame_rgb.copy()
                        
                        for i, detection in enumerate(detections):
                            # LOG EVERY SINGLE FIELD
                            # self.get_logger().info(
                            #     f"[[DETECTION #{i}]] Status: RAW >> "
                            #     f"Conf: {detection.confidence:.3f} | "
                            #     f"Box: [x:{detection.xmin:.2f}, y:{detection.ymin:.2f}, w:{detection.xmax-detection.xmin:.2f}, h:{detection.ymax-detection.ymin:.2f}] | "
                            #     f"Spatial: [x:{detection.spatialCoordinates.x:.1f}, y:{detection.spatialCoordinates.y:.1f}, z:{detection.spatialCoordinates.z:.1f}]"
                            # )
                            
                            # Filter Logic
                            repro_status = "ACCEPTED"
                            repro_reason = ""

                            # 1. Filter invalid depth
                            if detection.spatialCoordinates.z == 0.0:
                                repro_status = "REJECTED"
                                repro_reason = "Zero Depth (Z=0.0)"
                            
                            if repro_status == "REJECTED":
                                self.get_logger().warn(f"[[DETECTION #{i}]] REJECTED: {repro_reason}")
                                if annotated is not None:
                                    h, w = annotated.shape[:2]
                                    x1 = int(detection.xmin * w)
                                    y1 = int(detection.ymin * h)
                                    x2 = int(detection.xmax * w)
                                    y2 = int(detection.ymax * h)
                                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                                    cv2.putText(annotated, f"REJECT: {repro_reason}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                                continue

                            # If Accepted
                            # self.get_logger().info(f"[[DETECTION #{i}]] ACCEPTED. Processing...")

                            x_mm = detection.spatialCoordinates.x
                            y_mm = detection.spatialCoordinates.y
                            z_mm = detection.spatialCoordinates.z
                            
                            if annotated is not None:
                                h, w = annotated.shape[:2]
                                x1 = int(detection.xmin * w)
                                y1 = int(detection.ymin * h)
                                x2 = int(detection.xmax * w)
                                y2 = int(detection.ymax * h)
                                
                                # Draw ACCEPTED boxes in GREEN
                                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                label_str = f"Z: {z_mm/1000.0:.2f}m C:{detection.confidence:.2f}"
                                cv2.putText(annotated, label_str, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            self.process_detection(x_mm, y_mm, z_mm, detection.confidence)
                            
                        # Publish annotated image if we have one
                        if annotated is not None:
                            msg_ann = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                            msg_ann.header.stamp = self.get_clock().now().to_msg()
                            msg_ann.header.frame_id = "oak_rgb_camera_optical_frame"
                            self.pub_viz.publish(msg_ann)

                            # Publish Heartbeat for BT Nodes (Detection2DArray)
                            # We send an empty message if no detections, or populate it if we wanted to match the full interface.
                            # For WaitForDetection, an empty message is sufficient to prove life.
                            hb_msg = Detection2DArray()
                            hb_msg.header = msg_ann.header
                            self.pub_heartbeat.publish(hb_msg)
                        elif len(detections) > 0: # If we have detections but no image, still send heartbeat
                             hb_msg = Detection2DArray()
                             hb_msg.header.stamp = self.get_clock().now().to_msg()
                             hb_msg.header.frame_id = "oak_rgb_camera_optical_frame"
                             self.pub_heartbeat.publish(hb_msg)

                    # self.get_logger().info("[[LOOP]] Frame tick processed.")
                    time.sleep(0.001)
                    
        except Exception as e:
            self.get_logger().error(f"[[FATAL]] Pipeline failed: {e}")
            self.get_logger().error(traceback.format_exc())

    def process_detection(self, x_mm, y_mm, z_mm, confidence):
            raw_x_m = x_mm / 1000.0
            raw_y_m = y_mm / 1000.0
            raw_z_m = z_mm / 1000.0

            axis_map = self._parse_axis_map(self.spatial_axis_map)
            raw_vec = [raw_x_m, raw_y_m, raw_z_m]
            mapped_vec = [sign * raw_vec[idx] for idx, sign in axis_map]

            # self.get_logger().info(
            #     f"[[PROCESS]] Raw camera coords (DepthAI): x={raw_x_m:.3f}, y={raw_y_m:.3f}, z={raw_z_m:.3f}"
            # )
            # self.get_logger().info(
            #     f"[[PROCESS]] Mapped camera coords -> frame '{self.camera_frame}': x={mapped_vec[0]:.3f}, y={mapped_vec[1]:.3f}, z={mapped_vec[2]:.3f} using map='{self.spatial_axis_map}'"
            # )

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
                        # self.get_logger().info(
                        #     f"[[TF]] base_link <- {self.camera_frame} translation=({t.transform.translation.x:.3f}, {t.transform.translation.y:.3f}, {t.transform.translation.z:.3f}) "
                        #     f"rpy=({math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, {math.degrees(yaw):.1f}) deg"
                        # )

                    pt_base_raw = tf2_geometry_msgs.do_transform_point(pt_raw, t)
                    pt_base = tf2_geometry_msgs.do_transform_point(pt_msg, t)

                    # self.get_logger().info(
                    #     f"[[TRANSFORM]] BASE_LINK (raw): x={pt_base_raw.point.x:.3f}, y={pt_base_raw.point.y:.3f}, z={pt_base_raw.point.z:.3f}"
                    # )
                    # self.get_logger().info(
                    #     f"[[TRANSFORM]] BASE_LINK (mapped): x={pt_base.point.x:.3f}, y={pt_base.point.y:.3f}, z={pt_base.point.z:.3f}"
                    # )

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

                            # self.get_logger().info(
                            #     f"[[TARGET]] base_link target=({target_msg.point.x:.3f}, {target_msg.point.y:.3f}, {target_msg.point.z:.3f}) -> {self.camera_frame} ({tgt_vec[0]:.3f}, {tgt_vec[1]:.3f}, {tgt_vec[2]:.3f})"
                            # )

                            best_map, best_err = self._best_axis_map(raw_vec, tgt_vec)
                            # if best_map is not None:
                                # self.get_logger().info(
                                #     f"[[SUGGEST]] Best spatial_axis_map='{best_map}' (sse={best_err:.6f})"
                                # )
                        except Exception as exc:
                            self.get_logger().warn(f"[[SUGGEST]] Failed to compute axis map suggestion: {exc}")

                    if self.pub_det:
                        det_msg = CanDetection()
                        det_msg.header = pt_base.header
                        det_msg.class_name = "can"
                        det_msg.confidence = confidence
                        det_msg.spatial_camera.x = mapped_vec[0]
                        det_msg.spatial_camera.y = mapped_vec[1]
                        det_msg.spatial_camera.z = mapped_vec[2]
                        det_msg.spatial_base_link = pt_base.point
                        self.pub_det.publish(det_msg)
                        # self.get_logger().info("[[PUBLISH]] CanDetection message released.")
                else:
                    self.get_logger().warn(f"[[TRANSFORM]] Cannot transform from '{self.camera_frame}' to base_link yet.")
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
