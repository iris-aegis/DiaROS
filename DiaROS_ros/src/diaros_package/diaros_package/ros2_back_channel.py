# ============================================================
# ログレベル設定
# ============================================================
SHOW_BASIC_LOGS = True   # 基本ログ表示（メッセージ送受信、エラーなど）
SHOW_DEBUG_LOGS = False  # デバッグログ表示（詳細な処理内容、中間データなど）

import rclpy
import threading
import sys
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
from interfaces.msg import Ibc  # 追加
from diaros.backChannel import main as back_channel_main, push_audio_data, back_channel_result_queue

class RosBackChannel(Node):
    def __init__(self):
        super().__init__('back_channel')

        # RELIABLE QoSプロファイルを定義（分散実行対応）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(Float32MultiArray, 'mic_audio_float32', self.listener_callback, qos_profile)
        self.pub_bc = self.create_publisher(Ibc, 'BCtoDM', qos_profile)  # 追加
        self.timer = self.create_timer(0.01, self.publish_back_channel)  # 追加
        self.send_count = 0  # 送信回数カウンタ追加
        self.recv_count = 0  # 受信回数カウンタ追加
        if SHOW_BASIC_LOGS:
            self.get_logger().info(' Listening to mic_audio_float32...')

    def listener_callback(self, msg):
        audio_np = np.array(msg.data, dtype=np.float32)
        push_audio_data(audio_np)
        self.recv_count += 1
        first_val = audio_np[0] if len(audio_np) > 0 else None
        # sys.stdout.write(f"[ros2_back_channel] Received mic_audio_float32 #{self.recv_count} (len={len(audio_np)}) first={first_val}\n")
        sys.stdout.flush()

    def publish_back_channel(self):
        if not back_channel_result_queue.empty():
            (result_value, confidence) = back_channel_result_queue.get()
            msg = Ibc()
            msg.result = int(result_value)
            msg.confidence = float(confidence)
            self.pub_bc.publish(msg)
            self.send_count += 1
            # 送信時刻と推論値を全桁表示
            now = time.time()
            now_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now*1000)%1000):03d}"
            # print(f" Send#{self.send_count} {now_str} result={msg.result} confidence={msg.confidence:.10f}")
            # sys.stdout.flush()

def runROS(node):
    rclpy.spin(node)

def runBackChannel():
    back_channel_main()

def shutdown():
    while True:
        key = sys.stdin.readline().strip()
        if key == "kill":
            if SHOW_BASIC_LOGS:
                sys.stdout.write("kill command received.\n")
                sys.stdout.flush()
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    node = RosBackChannel()
    ros = threading.Thread(target=runROS, args=(node,))
    bc = threading.Thread(target=runBackChannel)
    ros.setDaemon(True)
    bc.setDaemon(True)
    ros.start()
    bc.start()
    shutdown()

if __name__ == '__main__':
    main()
