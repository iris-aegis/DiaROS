import rclpy
import threading
import sys
from interfaces.msg import Iasr
from interfaces.msg import Imm
from rclpy.node import Node

"""
言語理解モジュールを組み込む場合利用する
(現在は音声認識結果を対話管理へ流しているだけ)
"""

class RosLanguageUnderstanding(Node):
    def __init__(self, languageUnderstanding):
        super().__init__('language_understanding')
        self.languageUnderstanding = languageUnderstanding
        self.sub_asr = self.create_subscription(Iasr, 'ASRtoLU', self.send, 1)
        self.pub_lu = self.create_publisher(Iasr, 'LUtoDM', 1)
        self.pub_mm = self.create_publisher(Imm, 'MM', 1)
        self.timer = self.create_timer(1, self.ping)
        sys.stdout.write('LanguageUnderstanding start up.\n')
        sys.stdout.write('=====================================================\n')

    def send(self, asr):
        dm = Iasr()
        dm.you = asr.you
        dm.is_final = asr.is_final
        self.pub_lu.publish(dm)

    def ping(self):
        mm = Imm()
        mm.mod = "lu"
        self.pub_mm.publish(mm)

def runROS(pub):
    rclpy.spin(pub)

def shutdown():
    while True:
        key = input()
        if key == "kill":
            print("kill command received.")
            sys.exit()

def main(args=None):
    lu = ""
    rclpy.init(args=args)
    rlu = RosLanguageUnderstanding(lu)

    ros = threading.Thread(target=runROS, args=(rlu,))

    ros.setDaemon(True)

    ros.start()
    shutdown()

if __name__ == '__main__':
    main()