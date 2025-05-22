#!/usr/bin/env python3
"""
Standalone OAK-D detection node.
Publishes RGB Image, depth disparity image, pointcloud2 (approx), and runs YOLOv4 tiny spatial detection.
"""
print("INFO: oakd_detector_node.py script started execution.", flush=True)

import os
import site
import sys  # Explicitly import sys for sys.path
from ament_index_python.packages import get_package_share_directory

# Add the virtual environment's site-packages to sys.path
venv_site_packages = os.path.expanduser('~/depthai-venv/lib/python3.12/site-packages')
if venv_site_packages not in sys.path:
    sys.path.insert(0, venv_site_packages)

# Attempt to import depthai
try:
    import depthai as dai
    print("INFO: Successfully imported depthai.")
except ImportError as e:
    print(f"ERROR: Failed to import depthai: {e}")
    sys.exit(1)

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge  # Make sure CvBridge is imported
import numpy as np
import struct


class OakdDetectorNode(Node):
    def __init__(self):
        super().__init__('oakd_detector_node')
        self.get_logger().info("OakdDetectorNode initialized.")
        self.bridge = CvBridge()
        # Publishers
        self.rgb_pub = self.create_publisher(Image, 'rgb/image_raw', 10)
        self.disparity_pub = self.create_publisher(Image, 'stereo/disparity', 10)
        self.pcl_pub = self.create_publisher(PointCloud2, '/stereo/points', 10)
        self.nn_bbox_pub = self.create_publisher(Image, 'nn/bounding_boxes', 10)
        self.nn_labels_pub = self.create_publisher(Image, 'nn/labels', 10)

        self.get_logger().info("Attempting to create DepthAI pipeline...")
        pipeline = dai.Pipeline()
        self.get_logger().info("Pipeline object created.")

        cam = pipeline.createColorCamera()
        cam.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setPreviewSize(416, 416)  # Resize input to match NN dimensions
        cam.setInterleaved(False)  # Ensure data is planar (CHW)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)  # Set color order to BGR
        self.get_logger().info("Color camera configured.")

        xout_rgb = pipeline.createXLinkOut()
        xout_rgb.setStreamName('rgb')
        cam.video.link(xout_rgb.input)
        self.get_logger().info("RGB XLinkOut configured.")

        stereo = pipeline.createStereoDepth()
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)  # Align depth map to RGB socket
        self.get_logger().info("StereoDepth configured.")

        # Link mono cameras to stereo depth
        monoLeft = pipeline.createMonoCamera()
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoLeft.out.link(stereo.left)

        monoRight = pipeline.createMonoCamera()
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")
        monoRight.out.link(stereo.right)
        self.get_logger().info("Mono cameras linked to StereoDepth.")

        xout_disp = pipeline.createXLinkOut()
        xout_disp.setStreamName('disparity')
        stereo.disparity.link(xout_disp.input)
        self.get_logger().info("Disparity XLinkOut configured.")

        nn = pipeline.createYoloSpatialDetectionNetwork()
        blob_file = 'yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob'

        # Path for the blob within the package
        pkg_share_dir = get_package_share_directory('oakd_detector')
        resource_dir = os.path.join(pkg_share_dir, 'resources')
        blob_path = os.path.join(resource_dir, blob_file)

        if not os.path.exists(blob_path):
            self.get_logger().error(f"NN Blob file not found at: {blob_path}. Ensure it's in oakd_detector/resources and installed via setup.py.")
            # Fallback to depthai_examples path if not found in local package
            self.get_logger().warn("Attempting to use blob from depthai_examples as a fallback...")
            try:
                depthai_examples_share = get_package_share_directory('depthai_examples')
                blob_path_fallback = os.path.join(depthai_examples_share, 'resources', 'models', blob_file)
                if os.path.exists(blob_path_fallback):
                    blob_path = blob_path_fallback
                    self.get_logger().info(f"Using fallback blob: {blob_path}")
                else:
                    self.get_logger().error(f"Fallback blob also not found: {blob_path_fallback}. NN will likely fail.")
                    # Handle missing blob appropriately, e.g., by not configuring the NN or raising an error
            except Exception as ex:
                self.get_logger().error(f"Error accessing fallback blob path: {ex}")

        if os.path.exists(blob_path):
            nn.setBlobPath(blob_path)
            nn.setConfidenceThreshold(0.5)
            nn.setNumInferenceThreads(2)
            nn.input.setBlocking(False)
            nn.input.setQueueSize(4)
            nn.setAnchorMasks({
                'side13': [0, 1, 2],
                'side26': [3, 4, 5]
            })
            nn.setSpatialCalculationAlgorithm(dai.SpatialLocationCalculatorAlgorithm.AVERAGE)  # Set spatial calculation algorithm
            self.get_logger().info("Anchor masks and spatial calculation configured.")
            self.get_logger().info(f"YOLO Spatial Detection Network configured with blob: {blob_path}")
            cam.preview.link(nn.input)  # Link preview from color camera to NN input
            self.get_logger().info("Color camera preview linked to NN input.")
            xout_nn = pipeline.createXLinkOut()  # Create XLinkOut for NN output (detections)
            xout_nn.setStreamName("nn")
            nn.out.link(xout_nn.input)
            self.get_logger().info("NN output XLinkOut configured.")
            # For YoloSpatialDetectionNetwork, depth input is also needed
            stereo.depth.link(nn.inputDepth)
            self.get_logger().info("Stereo depth linked to NN inputDepth.")

        else:
            self.get_logger().error("No valid NN blob path found. Spatial detection will not work.")

        self.get_logger().info("Attempting to start DepthAI device...")
        try:
            # Get mxId from parameters
            self.declare_parameter('mxId', '14442C1051B665D700')  # Default if not set
            mx_id = self.get_parameter('mxId').get_parameter_value().string_value

            device_info = None
            if mx_id and mx_id.upper() != "ANY":
                found, device_info_mx = dai.Device.getDeviceByMxId(mx_id)
                if not found:
                    self.get_logger().error(f"No OAK-D device found with mxId: {mx_id}")
                    raise RuntimeError(f"No OAK-D device found with mxId: {mx_id}")
                device_info = device_info_mx
                self.get_logger().info(f"Using device with mxId: {mx_id}")

            self.device = dai.Device(pipeline, device_info)
            self.get_logger().info("DepthAI device started successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to start DepthAI device: {e}")
            raise

        self.q_rgb = self.device.getOutputQueue('rgb', maxSize=4, blocking=False)
        self.q_disp = self.device.getOutputQueue('disparity', maxSize=4, blocking=False)
        if os.path.exists(blob_path):  # Only get NN queue if NN was configured
            self.q_nn = self.device.getOutputQueue('nn', maxSize=4, blocking=False)
        else:
            self.q_nn = None
        self.get_logger().info("Output queues initialized.")

        self.timer = self.create_timer(0.03, self.tick)  # Approx 30 FPS
        self.get_logger().info("Node tick timer started.")

    def tick(self):
        try:
            in_rgb = self.q_rgb.tryGet()
            if in_rgb is not None:
                img = in_rgb.getCvFrame()
                msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "oakd_rgb_camera_optical_frame"
                self.rgb_pub.publish(msg)
        except RuntimeError as e:
            self.get_logger().error(f"Failed to read from RGB stream: {e}")
            return

        try:
            in_disp = self.q_disp.tryGet()
            if in_disp is not None:
                disp_frame = in_disp.getFrame()
                disp_vis = cv2.normalize(disp_frame, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                disp_vis = cv2.cvtColor(disp_vis, cv2.COLOR_GRAY2BGR)

                msg = self.bridge.cv2_to_imgmsg(disp_vis, 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "oakd_stereo_camera_optical_frame"
                self.disparity_pub.publish(msg)
        except RuntimeError as e:
            self.get_logger().error(f"Failed to read from disparity stream: {e}")
            return

        if self.q_nn:
            try:
                in_nn = self.q_nn.tryGet()
                if in_nn is not None:
                    detections = in_nn.detections
                    if detections:
                        self.get_logger().debug(f"Detections: {len(detections)}")
                        for detection in detections:
                            bbox_msg = Image()
                            bbox_msg.header.stamp = self.get_clock().now().to_msg()
                            bbox_msg.header.frame_id = "oakd_nn_bounding_boxes"
                            # Populate bbox_msg with bounding box data
                            self.nn_bbox_pub.publish(bbox_msg)

                            label_msg = Image()
                            label_msg.header.stamp = self.get_clock().now().to_msg()
                            label_msg.header.frame_id = "oakd_nn_labels"
                            # Populate label_msg with label data
                            self.nn_labels_pub.publish(label_msg)
            except RuntimeError as e:
                self.get_logger().error(f"Failed to read from NN stream: {e}")
                return

        pcl_msg = PointCloud2()
        pcl_msg.header.stamp = self.get_clock().now().to_msg()
        pcl_msg.header.frame_id = "oakd_stereo_camera_optical_frame"
        # Populate pcl_msg with point cloud data
        self.pcl_pub.publish(pcl_msg)


def main(args=None):
    print("INFO: main() function started.", flush=True)
    rclpy.init(args=args)
    print("INFO: rclpy.init() called.", flush=True)

    node = None
    try:
        node = OakdDetectorNode()
        print("INFO: OakdDetectorNode instantiated.", flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("INFO: KeyboardInterrupt caught, shutting down.", flush=True)
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception in main: {e}")
        else:
            print(f"ERROR: Unhandled exception before node init: {e}", flush=True)
    finally:
        if node:
            node.destroy_node()
            print("INFO: Node destroyed.", flush=True)
        rclpy.shutdown()
        print("INFO: rclpy.shutdown() called.", flush=True)


if __name__ == '__main__':
    main()
