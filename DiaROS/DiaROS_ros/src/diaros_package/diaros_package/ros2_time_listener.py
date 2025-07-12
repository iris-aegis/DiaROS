import rclpy
from rclpy.node import Node
from interfaces.msg import Iss
from diaros.timestamp_display import TimestampDisplay
import threading
import sys

class Ros2TimeListener(Node):
    def __init__(self, timestamp_display):
        super().__init__('ros2_time_listener')
        self.subscription = self.create_subscription(Iss, 'SStoDM', self.listener_callback, 10)
        self.timestamp_display = timestamp_display

    def listener_callback(self, msg):
        self.timestamp_display.update(msg.timestamp)

def main(args=None):
    pass

if __name__ == '__main__':
    main()
