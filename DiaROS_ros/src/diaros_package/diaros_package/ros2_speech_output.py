import rclpy
import threading
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from interfaces.msg import Iso
from diaros.speechOutput import SpeechOutput
import time
from datetime import datetime

# ============================================================
# ログレベル設定
# ============================================================
SHOW_BASIC_LOGS = True
SHOW_DEBUG_LOGS = False


class RosSpeechOutput(Node):
    def __init__(self, speech_output):
        super().__init__('speech_output')
        self.speech_output = speech_output

        # RELIABLE QoSプロファイルを定義（分散実行対応）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # DMからの音声再生リクエストを購読
        self.sub_dm = self.create_subscription(Iso, 'DMtoSO', self.play_request, qos_profile)

        # タイマー（状態監視用、オプション）
        self.timer = self.create_timer(0.1, self.monitor_state)

        if SHOW_BASIC_LOGS:
            sys.stdout.write("[ROS2-SO] ros2_speech_output ノード初期化完了\n")
            sys.stdout.flush()

    def play_request(self, msg):
        """音声再生リクエストを受信

        Args:
            msg (Iso): 音声再生リクエストメッセージ
        """
        if SHOW_DEBUG_LOGS:
            now_dt = datetime.now()
            timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[{timestamp}] [ROS2-SO-DEBUG] 音声再生リクエスト受信: type={msg.audio_type}, stage={msg.stage}, request_id={msg.request_id}\n")
            sys.stdout.flush()

        # SpeechOutputモジュールに再生リクエストを渡す
        self.speech_output.play_request_callback(
            audio_type=msg.audio_type,
            stage=msg.stage,
            wav_path=msg.wav_path,
            duration_sec=msg.duration_sec,
            request_id=msg.request_id,
            timestamp_ns=msg.timestamp_ns,
            session_id=msg.session_id
        )

    def monitor_state(self):
        """状態監視タイマー（必要に応じて再生状態を発行）"""
        # 現在は何もしない（将来の拡張用）
        pass


def main(args=None):
    rclpy.init(args=args)

    # SpeechOutputモジュールを初期化
    speech_output = SpeechOutput()
    speech_output.start()

    # ROS2ノードを作成
    ros_speech_output = RosSpeechOutput(speech_output)

    if SHOW_BASIC_LOGS:
        sys.stdout.write("[ROS2-SO] SpeechOutput ノード起動完了\n")
        sys.stdout.flush()

    try:
        rclpy.spin(ros_speech_output)
    except KeyboardInterrupt:
        if SHOW_BASIC_LOGS:
            sys.stdout.write("\n[ROS2-SO] KeyboardInterrupt を検出しました。終了します。\n")
            sys.stdout.flush()
    finally:
        # SpeechOutputを停止
        speech_output.stop()

        # ノードを破棄
        ros_speech_output.destroy_node()
        rclpy.shutdown()

        if SHOW_BASIC_LOGS:
            sys.stdout.write("[ROS2-SO] ノード終了\n")
            sys.stdout.flush()


if __name__ == '__main__':
    main()
