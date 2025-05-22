import sys
import os

# Determine the path to the virtual environment's site-packages directory
# Assuming the script is in <workspace>/src/<package_name>/<package_name>/
# And the venv is in <workspace>/src/<package_name>/venv/
script_dir = os.path.dirname(os.path.realpath(__file__))
package_root = os.path.abspath(os.path.join(script_dir, '..')) # Goes up one level from oakd_detector/oiv7.py to oakd_detector/
venv_path = os.path.join(package_root, 'venv')
python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
site_packages_path = os.path.join(venv_path, 'lib', python_version, 'site-packages')

# Add the venv's site-packages to sys.path if it's not already there
if site_packages_path not in sys.path:
    sys.path.insert(0, site_packages_path)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class OIV7DetectorNode(Node):
    def __init__(self):
        super().__init__('oiv7_detector')
        self.model = YOLO("yolov8n-oiv7.pt")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/oakd_top/color/preview/image',  # Change this to your image topic
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("OIV7 YOLOv8n detector node started and subscribing to /image_raw...")

    def image_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Run prediction
        results = self.model.predict(source=cv_image)
        
        # Process results
        if results and len(results[0]) > 0:
            self.get_logger().info(f"Detected {len(results[0])} objects.")
            # To draw results on the image and display (requires a GUI environment)
            annotated_frame = results[0].plot()
            cv2.imshow("YOLOv8 Inference", annotated_frame)
            cv2.waitKey(1)
        else:
            self.get_logger().info("No objects detected.")
            # Optionally, display the original image if no detections
            # cv2.imshow("YOLOv8 Inference", cv_image)
            # cv2.waitKey(1)

        # The full results object can be verbose, log specific parts if needed
        # self.get_logger().info(f"Prediction results: {results}")


def main(args=None):
    rclpy.init(args=args)
    oiv7_detector_node = OIV7DetectorNode()
    try:
        rclpy.spin(oiv7_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        oiv7_detector_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows() # If using cv2.imshow

if __name__ == '__main__':
    main()