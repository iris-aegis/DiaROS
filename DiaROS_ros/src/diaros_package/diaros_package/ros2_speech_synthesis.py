import rclpy
import threading
import sys
from rclpy.node import Node
from interfaces.msg import Inlg
from interfaces.msg import Iss
from interfaces.msg import Imm
from diaros.speechSynthesis import SpeechSynthesis
# from interfaces.msg import Time
from datetime import datetime

class RosSpeechSynthesis(Node):
    def __init__(self, speechSynthesis):
        super().__init__('speech_synthesis')
        self.speechSynthesis = speechSynthesis
        self.sub_nlg = self.create_subscription(Inlg, 'NLGtoSS', self.play, 1)
        self.pub_ss = self.create_publisher(Iss, 'SStoDM', 1)
        # self.pub_ss = self.create_publisher(Iss, 'SStoDR', 1)
        # self.pub_mm = self.create_publisher(Imm, 'MM', 1)
        # self.pub_wav = self.create_publisher(SynthWav, 'SynthWav', 1)  # ← 削除
        self.timer = self.create_timer(0.02, self.send)
        self.is_speaking = False

    def play(self, nlg):
        text = str(nlg.reply)
        # 対話生成結果受信時のログ出力
        # now = datetime.now()
        # timestamp = now.strftime('%H:%M:%S.%f')[:-3]
        # sys.stdout.write(f"[{timestamp}][SS] 受信した対話生成内容: {nlg.reply}\n")
        # sys.stdout.flush()
        wav_path = self.speechSynthesis.run(text)
        # 音声合成後、ファイル名をIssでpublish
        # wav_msg = SynthWav()
        # wav_msg.filename = wav_path if wav_path else ""
        # self.pub_wav.publish(wav_msg)
        # if not self.is_speaking:
        #     text = str(nlg.reply)
        #     print("speaking..."+text)
        #     self.is_speaking = True
        #     self.speechSynthesis.run(text)
        #     print("finish..."+text)
        #     self.is_speaking = False

    def send(self):
        ss = Iss()
        ss.is_speaking = self.speechSynthesis.speak_end
        # print(ss.is_speaking)
        # 追記
        now = datetime.now()
        ss.timestamp = now.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        # 直近の合成ファイル名を取得して送信
        if hasattr(self.speechSynthesis, 'last_tts_file'):
            ss.filename = self.speechSynthesis.last_tts_file if self.speechSynthesis.last_tts_file else ""
        else:
            ss.filename = ""
        self.pub_ss.publish(ss)
        self.speechSynthesis.speak_end = False

        mm = Imm()
        mm.mod = "ss"
        # self.pub_mm.publish(mm)

def runROS(pub):
    rclpy.spin(pub)

def shutdown():
    while True:
        key = input()
        if key == "kill":
            print("kill command received.")
            sys.exit()

def main(args=None):
    ss = SpeechSynthesis()
    rclpy.init(args=args)
    rss = RosSpeechSynthesis(ss)

    ros = threading.Thread(target=runROS, args=(rss,))

    ros.setDaemon(True)

    ros.start()
    shutdown()

if __name__ == '__main__':
    main()