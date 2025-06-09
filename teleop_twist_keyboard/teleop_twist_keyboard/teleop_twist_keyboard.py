#!/usr/bin/env python3

from __future__ import print_function

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist
stamped = False
twist_frame = ''

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate, node):
        super(PublishThread, self).__init__()
        self.node = node
        self.publisher = node.create_publisher(TwistMsg, 'cmd_vel', 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.topic_name))
            time.sleep(0.5)
            i += 1
            i = i % 5
        if not rclpy.ok():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = self.node.get_clock().now().to_msg()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = float(self.x * self.speed)
            twist.linear.y = float(self.y * self.speed)
            twist.linear.z = float(self.z * self.speed)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(self.th * self.turn)

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        if settings is None:
            # Not a terminal (e.g., running in launch file)
            # Just wait and return empty string to keep the node running
            import time
            time.sleep(timeout)
            return ''
        
        try:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            # Always restore terminal settings, even if an exception occurs
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    try:
        return termios.tcgetattr(sys.stdin)
    except termios.error:
        # Not a terminal (e.g., running in launch file)
        return None

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    if old_settings is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        
        # Declare parameters
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)
        self.declare_parameter('speed_limit', 1000.0)
        self.declare_parameter('turn_limit', 1000.0)
        self.declare_parameter('repeat_rate', 0.0)
        self.declare_parameter('key_timeout', 0.5)
        self.declare_parameter('stamped', False)
        self.declare_parameter('frame_id', '')

def main():
    global stamped, twist_frame, TwistMsg
    
    settings = saveTerminalSettings()
    
    rclpy.init()
    
    node = TeleopTwistKeyboard()
    
    # Warn if terminal settings not available
    if settings is None:
        print("WARNING: Running without terminal access (e.g., in launch file).")
        print("Keyboard input will not work. Use 'ros2 run teleop_twist_keyboard teleop_twist_keyboard' for keyboard control.")
        print("Node will continue running to allow topic subscriptions but keyboard input is disabled.")
    
    speed = node.get_parameter('speed').get_parameter_value().double_value
    turn = node.get_parameter('turn').get_parameter_value().double_value
    speed_limit = node.get_parameter('speed_limit').get_parameter_value().double_value
    turn_limit = node.get_parameter('turn_limit').get_parameter_value().double_value
    repeat = node.get_parameter('repeat_rate').get_parameter_value().double_value
    key_timeout = node.get_parameter('key_timeout').get_parameter_value().double_value
    stamped = node.get_parameter('stamped').get_parameter_value().bool_value
    twist_frame = node.get_parameter('frame_id').get_parameter_value().string_value
    
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat, node)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while rclpy.ok():
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        node.destroy_node()
        rclpy.shutdown()
        restoreTerminalSettings(settings)
        
        # Additional terminal cleanup for Linux
        if sys.platform != 'win32' and settings is not None:
            try:
                # Reset terminal to sane state
                import subprocess
                subprocess.run(['stty', 'sane'], check=False)
            except:
                pass

if __name__=="__main__":
    main()
