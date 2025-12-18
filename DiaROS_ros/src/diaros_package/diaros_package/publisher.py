#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import List 
from voice_dialogue.voicedialogue import VoiceDialogue
import sys
import os

class Publisher(Node):
    def __init__(self, voiceDialogue):
        super().__init__('talker')
        self.voicedialogue = voiceDialogue
        self.pub = self.create_publisher(List, 'chatter', 1)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def recieve(self, res):
        self.word = res

    def timer_callback(self):
        msg = List()
        msg.n = self.voicedialogue.log()['turn']
        msg.you = self.voicedialogue.log()['you']
        msg.bot = self.voicedialogue.log()['bot']
        msg.frequency = self.voicedialogue.log()['frequency']
        sys.stdout.write(f"{msg.frequency} {self.voicedialogue.log()['grad']} {self.voicedialogue.log()['power']}\n")
        sys.stdout.flush()
                """
                if abs(self.voicedialogue.log()['grad']) < 10.0 and msg.frequency < 50.0 and self.voicedialogue.log()['power'] < 0.20: 
                    sys.stdout.write("no voice\n")
                    sys.stdout.flush()
                else :
                    sys.stdout.write("xxxxxxxxxxxxxxxxxxx\n")
                    sys.stdout.flush()
                """        
        #self.get_logger().info("Pub: {0} {1} {2} {3}".format( msg.n, msg.you, msg.bot, msg.frequency ) )
        self.pub.publish(msg)

def work1(pub):
    rclpy.spin(pub)

def work2(voicedialogue):
    voicedialogue.runRecognition()

def work3(voice_dialogue):
    voice_dialogue.runAnswer()

def shutdown(pub, voicedialogue):
    while True:
        key = sys.stdin.readline().strip()
        if key == "kill":
            sys.stdout.write("kill command received.\n")
            sys.stdout.flush()
            sys.exit()

def main(args=None):
    voicedialogue = VoiceDialogue()
    rclpy.init(args=args)
    pub = Publisher(voicedialogue)
    t1 = threading.Thread(target=work1, args=(pub,))
    t2 = threading.Thread(target=work2, args=(voicedialogue,))
    t3 = threading.Thread(target=work3, args=(voicedialogue,))

    t1.setDaemon(True)
    t2.setDaemon(True)
    t3.setDaemon(True)

    t1.start()
    t2.start()
    t3.start()
    shutdown(pub, voicedialogue)    


if __name__ == '__main__':
    main()