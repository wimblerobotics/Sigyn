import rclpy
from rclpy.node import Node
import subprocess
import re
import sqlite3

class WifiDataCollector(Node):
    def __init__(self):
        super().__init__('wifi_data_collector')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.db_path = '/home/ros/sigyn_ws/src/Sigyn/wifi_data.db'  # Database path
        self.create_table()
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def create_table(self):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS wifi_data (
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                x REAL,
                y REAL,
                bit_rate REAL,
                link_quality REAL,
                signal_level REAL
            )
        """)
        conn.commit()
        conn.close()

    def get_wifi_data(self):
        try:
            output = subprocess.check_output(["iwconfig", "wlp9s0"]).decode("utf-8")

            # Extract bit rate
            bit_rate_match = re.search(r"Bit Rate[:=](?P<bit_rate>\d+\.?\d*) (Mb/s|Gb/s)", output)
            print(f'bit_rate_match: {bit_rate_match}')
            if bit_rate_match:
                bit_rate = float(bit_rate_match.group("bit_rate"))
                if bit_rate_match.group(2) == "Gb/s":
                    bit_rate *= 1000  # Convert Gb/s to Mb/s
            else:
                bit_rate = None

            # Extract link quality
            link_quality_match = re.search(r"Link Quality=(?P<link_quality>\d+/\d+)", output)
            if link_quality_match:
                link_quality_str = link_quality_match.group("link_quality")
                link_quality = float(link_quality_str.split('/')[0]) / float(link_quality_str.split('/')[1])
            else:
                link_quality = None

            # Extract signal level
            signal_level_match = re.search(r"Signal level[:=](?P<signal_level>-?\d+) dBm", output)
            signal_level = float(signal_level_match.group("signal_level")) if signal_level_match else None

            return bit_rate, link_quality, signal_level

        except subprocess.CalledProcessError:
            self.get_logger().warn("Could not retrieve wifi data")
            return None, None, None

    def timer_callback(self):
        bit_rate, link_quality, signal_level = self.get_wifi_data()
        if bit_rate is not None and link_quality is not None and signal_level is not None:
            self.insert_data(bit_rate, link_quality, signal_level)
            self.get_logger().info(f"X: {self.x}, Y: {self.y}, Bit Rate: {bit_rate}, Link Quality: {link_quality}, Signal Level: {signal_level}")
        else:
            self.get_logger().warn("Could not retrieve all wifi data, skipping insertion")

    def insert_data(self, bit_rate, link_quality, signal_level):
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("""
            INSERT INTO wifi_data (x, y, bit_rate, link_quality, signal_level)
            VALUES (?, ?, ?, ?, ?)
        """, (self.x, self.y, bit_rate, link_quality, signal_level))
        conn.commit()
        conn.close()

def main(args=None):
    rclpy.init(args=args)
    wifi_data_collector = WifiDataCollector()
    rclpy.spin(wifi_data_collector)
    rclpy.shutdown()
