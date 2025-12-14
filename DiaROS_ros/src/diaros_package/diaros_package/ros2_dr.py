import rclpy
import threading
import sys
import requests
import json
from rclpy.node import Node
from interfaces.msg import Iasr
from interfaces.msg import Isa
from interfaces.msg import Inlg
from interfaces.msg import Iss
from interfaces.msg import Imm


class RosDataReceiver(Node):
    def __init__(self):
        super().__init__('data_receiver')
        self.turn = 0 # 0: YOU 1: BOT
        self.prevyou = ""
        self.prevbot = ""
        self.asr = { "you": "", "is_final": False }
        self.sa = {
            "prevgrad" : 0.0,
            "frequency": 0.0,
            "grad"     : 0.0,
            "power"    : 0.0,
            "zerocross": 0   }
        self.nlg = { "reply": "" }
        self.ss = { "is_speaking": False }
        self.sub_asr = self.create_subscription(Iasr, 'ASRtoDR', self.asr_update, 1)
        self.sub_sa = self.create_subscription(Isa, 'SAtoDR', self.sa_update, 1)
        self.sub_nlg = self.create_subscription(Inlg, 'NLGtoDR', self.nlg_update, 1)
        self.sub_ss = self.create_subscription(Iss, 'SStoDR', self.ss_update, 1)
        # self.pub_mm = self.create_publisher(Imm, 'MM', 1)
        self.timer = self.create_timer(0.02, self.request)

    def asr_update(self, asr):
        self.asr["you"] = asr.you
        self.asr["is_final"] = asr.is_final

    def sa_update(self, sa):
        self.sa["prevgrad"] = sa.prevgrad
        self.sa["frequency"] = sa.frequency
        self.sa["grad"] = sa.grad
        self.sa["power"] = sa.power
        self.sa["zerocross"] = sa.zerocross

    def nlg_update(self, nlg):
        self.nlg["reply"] = nlg.reply
    
    def ss_update(self, ss):
        self.ss["is_speaking"] = ss.is_speaking

    def request(self):
        if self.prevbot == self.nlg["reply"] and not self.prevyou == self.asr["you"]:
            self.turn = 0
        elif self.prevyou == self.asr["you"] and not self.prevbot == self.nlg["reply"]:
            self.turn = 1

        self.prevyou = self.asr["you"]
        self.prevbot = self.nlg["reply"]
        mm = Imm()
        mm.mod = "dr"
        # self.pub_mm.publish(mm)

        data = { 'turn': self.turn, 
                 'asr': json.dumps(self.asr, ensure_ascii=False), 
                 'nlg': json.dumps(self.nlg, ensure_ascii=False), 'sa': json.dumps(self.sa) }
        print(data)
        requests.post('http://localhost:3000/data', data)


def runROS(pub):
    rclpy.spin(pub)

def shutdown():
    while True:
        key = input()
        if key == "kill":
            print("kill command received.")
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    rdr = RosDataReceiver()

    ros = threading.Thread(target=runROS, args=(rdr,))

    ros.setDaemon(True)

    ros.start()
    shutdown()

if __name__ == '__main__':
    main()