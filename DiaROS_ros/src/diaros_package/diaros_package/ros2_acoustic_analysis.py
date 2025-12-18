import rclpy
import threading
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
from interfaces.msg import Iaa, Imm
from diaros.acousticAnalysis import AcousticAnalysis
import numpy as np

# ============================================================
# ログレベル設定
# ============================================================
SHOW_BASIC_LOGS = True   # 基本ログ表示（メッセージ送受信、エラーなど）
SHOW_DEBUG_LOGS = False  # デバッグログ表示（詳細な処理内容、中間データなど）

class RosAcousticAnalysis(Node):
    def __init__(self, acousticAnalysis):
        super().__init__('acoustic_analysis')
        self.acousticAnalysis = acousticAnalysis

        # RELIABLE QoSプロファイルを定義（分散実行対応）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub_audio = self.create_subscription(Float32MultiArray, 'mic_audio_float32', self.audio_callback, qos_profile)  # 修正
        self.pub_iaa = self.create_publisher(Iaa, 'AAtoDM', qos_profile)
        # self.pub_mm = self.create_publisher(Imm, 'MM', qos_profile)
        self.timer = self.create_timer(0.02, self.publish_acoustic)
        self.last_sent = None

    def audio_callback(self, msg):
        audio_np = np.array(msg.data, dtype=np.float32)
        # float32→int16変換
        audio_int16 = (audio_np * 32767).astype(np.int16).tobytes()
        self.acousticAnalysis.update(audio_int16)

    def publish_acoustic(self):
        if self.acousticAnalysis.last_result is not None and self.acousticAnalysis.last_result != self.last_sent:
            result = self.acousticAnalysis.last_result
            msg = Iaa()
            msg.f0 = float(result["f0"])
            msg.grad = float(result["grad"])
            msg.power = float(result["power"])
            msg.zerocross = int(result["zerocross"])
            self.pub_iaa.publish(msg)
            self.last_sent = result
        mm = Imm()
        mm.mod = "acoustic"
        # self.pub_mm.publish(mm)

def runROS(node):
    rclpy.spin(node)

def runAA(acousticAnalysis):
    acousticAnalysis.run()

def shutdown():
    while True:
        key = sys.stdin.readline().strip()
        if key == "kill":
            if SHOW_BASIC_LOGS:
                sys.stdout.write("kill command received.\n")
                sys.stdout.flush()
            sys.exit()

def main(args=None):
    aa = AcousticAnalysis(16000)
    rclpy.init(args=args)
    raa = RosAcousticAnalysis(aa)

    ros = threading.Thread(target=runROS, args=(raa,))
    mod = threading.Thread(target=runAA, args=(aa,))

    ros.setDaemon(True)
    mod.setDaemon(True)

    ros.start()
    mod.start()
    shutdown()

if __name__ == '__main__':
    main()
