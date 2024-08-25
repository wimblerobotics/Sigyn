from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String
import subprocess


class MeasureWifi(Node):

    def __init__(self):
        super().__init__('measure_wifi')
        self.x_ = 0.0
        self.y_ = 0.0
        self.publisher_ = self.create_publisher(String, 'wifi_strength', 10)
        timer_period = 1.0  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        self.x_ = msg.pose.pose.position.x
        self.y_ = msg.pose.pose.position.y
        # print(f'pose: {self.x_}, {self.y_}')
# data: 
# b''wlp38s0   IEEE 802.11  ESSID:"xxxxxxxx"  \n
#              Mode:Managed  Frequency:5.745 GHz  Access Point: xx:xx:xx:xx:xx:xx   \n
#              Bit Rate=130 Mb/s   Tx-Power=22 dBm   \n
#              Retry short limit:7   RTS thr:off   Fragment thr:off\n
#              Power Management:on\n
#              Link Quality=37/70  Signal level=-73 dBm  \n
#              Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0\n
#              Tx excessive retries:1  Invalid misc:34160   Missed beacon:0\n\n

    def timer_callback(self):
        msg = String()
        result = subprocess.run(['iwconfig'], capture_output=True, text=True) #stdout=subprocess.PIPE)
        iwconfig_result = result.stdout
        regex_br = re.compile(r'Bit Rate=(\d+ [MKG]b/s)[\r\n\sA-Za-z0-9:=-]*Link Quality=(\d+/\d+)\s+Signal level=([-\d]+ dBm)', re.MULTILINE)
        find_br = re.search(regex_br, iwconfig_result)
        if (find_br != None):
            groups = find_br.groups()
            br = groups[0]
            lq = groups[1]
            sl = groups[2]
            json = f'{{"x": "{self.x_:.3f}", "y": "{self.y_:.3f}", "bit_rate\": "{br}", "link_quality": "{lq}", "signal_level": "{sl}"}}'
            msg.data = json
            self.publisher_.publish(msg)
            print(f'sending: {json}')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MeasureWifi()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
