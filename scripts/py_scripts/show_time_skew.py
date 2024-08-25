from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg._range import Range
from std_msgs.msg import String
from sensor_msgs.msg._laser_scan import LaserScan

class ShowTimeSkew(Node):

    def reset_measurements(self):
        self.start_time_ns_ = self.get_clock().now().nanoseconds
        self.callback_count_ = 0
        self.min_skew_ = 1.0e9
        self.max_skew_ = -1.0e9
        self.start_callback_count_ = 0;
        self.total_skew_ = 0.0

    def __init__(self):
        super().__init__('show_time_skew')
        self.declare_parameter('topic', 'need_topic_name')
        self.reset_measurements()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to the topic.
        topic_name = self.get_parameter('topic').get_parameter_value().string_value
        self.get_logger().info(f'Subscrbing to topic: {topic_name}')
        self.subscription = self.create_subscription(
            #Range,
            LaserScan,
            topic_name,
            self.listener_callback,
            qos_profile,
        )
        
        self.subscription  # prevent unused variable warning


    # Process a time-of-flight sensor message of type Range.
    def listener_callback(self, msg: Range):
        now_ns = self.get_clock().now().nanoseconds
        msg_ns = Time.from_msg(msg.header.stamp).nanoseconds
        skew_s = (now_ns - msg_ns) / 1.0e9
        if skew_s < self.min_skew_:
            self.min_skew_ = skew_s
        if skew_s > self.max_skew_:
            self.max_skew_ = skew_s
        self.total_skew_ += skew_s
        self.callback_count_ = self.callback_count_ + 1

        if (now_ns - self.start_time_ns_) > 1000000000:
            self.get_logger().info(f"Callbacks in one second: {self.callback_count_ - self.start_callback_count_}, min_ms: {self.min_skew_*1000:.3f}, max_ms: {self.max_skew_*1000:.3f}, avs_ms: {(self.total_skew_/self.callback_count_)*1000:.3f}")
            self.reset_measurements()

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ShowTimeSkew()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
