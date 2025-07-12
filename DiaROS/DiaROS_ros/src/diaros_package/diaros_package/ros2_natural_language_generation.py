import rclpy
import threading
import sys
from rclpy.node import Node
from interfaces.msg import Idm
from interfaces.msg import Inlg
from interfaces.msg import Imm
from diaros.naturalLanguageGeneration import NaturalLanguageGeneration
class RosNaturalLanguageGeneration(Node):
    def __init__(self, naturalLanguageGeneration):
        super().__init__('natural_language_generation')
        self.naturalLanguageGeneration = naturalLanguageGeneration
        self.sub_dm = self.create_subscription(Idm, 'DMtoNLG', self.dm_update, 1)
        self.pub_nlg = self.create_publisher(Inlg, 'NLGtoSS', 1)  # NLG→SpeechSynthesis用
        # self.pub_nlg_dr = self.create_publisher(Inlg, 'NLGtoDR', 1)
        # self.pub_mm = self.create_publisher(Imm, 'MM', 1)
        self.timer = self.create_timer(0.02, self.ping)
        self.last_sent_reply = None

    def dm_update(self, msg):
        words = list(msg.words)
        if words:
            self.naturalLanguageGeneration.update(words)

    def ping(self):
        # 応答が生成されたらpublish
        if hasattr(self.naturalLanguageGeneration, "last_reply") and self.naturalLanguageGeneration.last_reply != self.last_sent_reply:
            nlg_msg = Inlg()
            nlg_msg.reply = self.naturalLanguageGeneration.last_reply
            self.pub_nlg.publish(nlg_msg)  # ここでNLG生成文をNLGtoSSトピックで送信
            # self.pub_nlg_dr.publish(nlg_msg)  # ← コメントアウト
            self.last_sent_reply = self.naturalLanguageGeneration.last_reply
        mm = Imm()
        mm.mod = "nlg"
        # self.pub_mm.publish(mm)

def runROS(node):
    rclpy.spin(node)

def runNLG(naturalLanguageGeneration):
    naturalLanguageGeneration.run()

def shutdown():
    while True:
        key = input()
        if key == "kill":
            print("kill command received.")
            sys.exit()

def main(args=None):
    naturalLanguageGeneration = NaturalLanguageGeneration()
    rclpy.init(args=args)
    rnlg = RosNaturalLanguageGeneration(naturalLanguageGeneration)

    ros = threading.Thread(target=runROS, args=(rnlg,))
    mod = threading.Thread(target=runNLG, args=(naturalLanguageGeneration,))

    ros.setDaemon(True)
    mod.setDaemon(True)

    ros.start()
    mod.start()
    shutdown()

if __name__ == '__main__':
    main()