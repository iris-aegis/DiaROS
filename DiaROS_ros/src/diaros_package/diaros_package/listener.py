#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import List 
import sys
import io
import requests

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(List, 'chatter', self.chatter_callback, 1)
        self.pub_test = self.create_publisher(List, 'test_topic', 1)

    def chatter_callback(self, msg):
        #self.get_logger().info("Relay: {0} {1} {2} {3}".format( msg.n, msg.you, msg.bot, msg.frequency ) )
        sys.stdout.write( "{0} {1} {2} {3}\n".format( msg.n, msg.you, msg.bot, msg.frequency ) )
        sys.stdout.flush()
        
        pubmsg = List()
        pubmsg.n         = msg.n
        pubmsg.you       = msg.you
        pubmsg.bot       = msg.bot
        pubmsg.frequency = msg.frequency
        self.pub_test.publish(pubmsg)
        
        #data = { 'num': msg.n, 'you': msg.you, 'bot': msg.bot, 'frequency':msg.frequency }
        #requests.post('http://localhost:3000/data', data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    #finally:
    #    listener.destroy_node()
    #    rclpy.shutdown()

if __name__ == '__main__':
    main()