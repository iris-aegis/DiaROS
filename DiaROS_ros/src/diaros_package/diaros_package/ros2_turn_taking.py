### ros2_turn_taking.py ###
# ============================================================
# ログレベル設定
# ============================================================
SHOW_BASIC_LOGS = True   # 基本ログ表示（メッセージ送受信、エラーなど）
SHOW_DEBUG_LOGS = True  # デバッグログ表示（詳細な処理内容、中間データなど）

import rclpy
import threading
import sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray, Float32, String
import numpy as np
from diaros.turnTaking import TurnTaking, push_audio_data, turn_taking_result_queue, push_asr_result, silero_vad_result_queue, vad_iterator_result_queue  # TurnTakingを実行するために読み込み
from interfaces.msg import Itt, Iasr

class RosTurnTaking(Node):
    def __init__(self):
        super().__init__('turn_taking')

        # RELIABLE QoSプロファイルを定義（分散実行対応）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'mic_audio_float32',
            self.listener_callback,
            qos_profile
        )
        # ASR結果を受信するサブスクリプション追加
        self.asr_subscription = self.create_subscription(
            Iasr,
            'ASRtoNLU',
            self.asr_callback,
            qos_profile
        )
        self.pub_tt = self.create_publisher(Itt, 'TTtoDM', qos_profile)

        # SileroVAD監視用パブリッシャー（10ms間隔手法）
        self.pub_silero_speech_prob = self.create_publisher(Float32, 'silero_vad/speech_probability', qos_profile)
        self.pub_silero_speech_ratio = self.create_publisher(Float32, 'silero_vad/speech_ratio', qos_profile)
        self.pub_silero_interval = self.create_publisher(Float32, 'silero_vad/judgment_interval_ms', qos_profile)
        self.pub_silero_segments = self.create_publisher(Float32, 'silero_vad/speech_segments_count', qos_profile)
        self.pub_silero_status = self.create_publisher(String, 'silero_vad/status', qos_profile)

        # VADIterator監視用パブリッシャー（32ms間隔手法）
        self.pub_iterator_speech_state = self.create_publisher(Float32, 'vad_iterator/speech_state', qos_profile)  # 0=無声, 1=音声
        self.pub_iterator_event_interval = self.create_publisher(Float32, 'vad_iterator/event_interval_ms', qos_profile)
        self.pub_iterator_total_events = self.create_publisher(Float32, 'vad_iterator/total_events', qos_profile)
        self.pub_iterator_event_type = self.create_publisher(String, 'vad_iterator/event_type', qos_profile)  # start/end/null
        self.pub_iterator_status = self.create_publisher(String, 'vad_iterator/status', qos_profile)

        self.timer = self.create_timer(0.0001, self.publish_turn_taking)  # 100ms→0.1msに短縮
        self.silero_timer = self.create_timer(0.001, self.publish_silero_vad)  # 1ms間隔でSileroVAD結果をパブリッシュ
        self.iterator_timer = self.create_timer(0.001, self.publish_vad_iterator)  # 1ms間隔でVADIterator結果をパブリッシュ
        self.recv_count = 0  # 受信回数カウンタ追加
        if SHOW_BASIC_LOGS:
            self.get_logger().info('[ros2_turn_taking] Listening to mic_audio_float32 and ASRtoNLU...')
            self.get_logger().info('[ros2_turn_taking] SileroVAD (10ms) + VADIterator (32ms) monitoring topics initialized')

    def listener_callback(self, msg):
        audio_np = np.array(msg.data, dtype=np.float32)
        push_audio_data(audio_np)
        self.recv_count += 1
        first_val = audio_np[0] if len(audio_np) > 0 else None
        
        # ★デバッグ：音声受信ログ（間引いて表示）
        if SHOW_DEBUG_LOGS and self.recv_count % 100 == 0:
            import datetime
            timestamp = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[{timestamp}][ROS_TT] mic_audio_float32受信 #{self.recv_count} (len={len(audio_np)})\n")
            sys.stdout.flush()
            
        # sys.stdout.write(f"[ros2_turn_taking] Received mic_audio_float32 #{self.recv_count} (len={len(audio_np)}) first={first_val}\n")
        # sys.stdout.flush()
        # print(f"[ros2_turn_taking] received buffer size: {len(audio_np)}")

    def asr_callback(self, msg):
        """ASR結果を受信してTurnTakingに転送"""
        push_asr_result(msg.you, msg.is_final)
        # デバッグ出力（必要に応じて）
        # if "[雑音]" in msg.you or "[無音]" in msg.you:
        #     sys.stdout.write(f"[ros2_turn_taking] ASR結果転送: '{msg.you}' (is_final={msg.is_final})\n")
        #     sys.stdout.flush()

    def publish_turn_taking(self):
        if not turn_taking_result_queue.empty():
            import datetime
            queue_get_time = datetime.datetime.now()
            queue_get_timestamp = queue_get_time.strftime('%H:%M:%S.%f')[:-3]
            
            (result_value, confidence) = turn_taking_result_queue.get()
            msg = Itt()
            msg.result = result_value
            msg.confidence = confidence
            
            publish_time = datetime.datetime.now()
            publish_timestamp = publish_time.strftime('%H:%M:%S.%f')[:-3]
            self.pub_tt.publish(msg)
            
            if SHOW_DEBUG_LOGS:
                sys.stdout.write(f"[{queue_get_timestamp}][ROS_TT] キューから取得, [{publish_timestamp}][ROS_TT] DM向け送信完了 (result={result_value}, conf={confidence:.3f})\n")
                sys.stdout.flush()

    def publish_silero_vad(self):
        """SileroVAD結果をROSトピックにパブリッシュ（RQT監視用）"""
        if not silero_vad_result_queue.empty():
            try:
                vad_result = silero_vad_result_queue.get()
                
                # 音声確率
                speech_prob_msg = Float32()
                speech_prob_msg.data = float(vad_result['speech_probability'])
                self.pub_silero_speech_prob.publish(speech_prob_msg)
                
                # 音声活動率
                speech_ratio_msg = Float32()
                speech_ratio_msg.data = float(vad_result['speech_ratio'])
                self.pub_silero_speech_ratio.publish(speech_ratio_msg)
                
                # 判定間隔
                interval_msg = Float32()
                interval_msg.data = float(vad_result['interval_ms'])
                self.pub_silero_interval.publish(interval_msg)
                
                # 音声セグメント数
                segments_msg = Float32()
                segments_msg.data = float(vad_result['speech_segments'])
                self.pub_silero_segments.publish(segments_msg)
                
                # ステータス（音声/無声＋状態変化）
                status_msg = String()
                status = "音声" if vad_result['is_speech'] else "無声"
                if vad_result['state_changed']:
                    status += "_変化"
                status_msg.data = status
                self.pub_silero_status.publish(status_msg)
                
                # デバッグ出力（状態変化時のみ）
                if vad_result['state_changed']:
                    if SHOW_DEBUG_LOGS:
                        sys.stdout.write(f"[{vad_result['timestamp']}][ROS_SILERO] ROSトピック送信: {status} (prob={vad_result['speech_probability']:.3f}, ratio={vad_result['speech_ratio']:.1f}%)\n")
                        sys.stdout.flush()
                    
            except Exception as e:
                if SHOW_BASIC_LOGS:
                    sys.stdout.write(f"[ERROR] SileroVAD ROSパブリッシュエラー: {e}\n")
                    sys.stdout.flush()

    def publish_vad_iterator(self):
        """VADIterator結果をROSトピックにパブリッシュ（RQT監視用）"""
        if not vad_iterator_result_queue.empty():
            try:
                iterator_result = vad_iterator_result_queue.get()
                
                # 音声状態（0=無声, 1=音声）
                speech_state_msg = Float32()
                speech_state_msg.data = 1.0 if iterator_result['is_speech'] else 0.0
                self.pub_iterator_speech_state.publish(speech_state_msg)
                
                # イベント間隔
                event_interval_msg = Float32()
                event_interval_msg.data = float(iterator_result['event_interval_ms'])
                self.pub_iterator_event_interval.publish(event_interval_msg)
                
                # 総イベント数
                total_events_msg = Float32()
                total_events_msg.data = float(iterator_result['total_events'])
                self.pub_iterator_total_events.publish(total_events_msg)
                
                # イベントタイプ
                event_type_msg = String()
                event_type_msg.data = iterator_result['result_type'] if iterator_result['result_type'] else "null"
                self.pub_iterator_event_type.publish(event_type_msg)
                
                # ステータス
                status_msg = String()
                status = "音声" if iterator_result['is_speech'] else "無声"
                if iterator_result['state_changed']:
                    status += "_変化"
                if iterator_result['speech_start']:
                    status += "_開始"
                elif iterator_result['speech_end']:
                    status += "_終了"
                status_msg.data = status
                self.pub_iterator_status.publish(status_msg)
                
                # デバッグ出力（状態変化時のみ）
                if iterator_result['state_changed']:
                    if SHOW_DEBUG_LOGS:
                        sys.stdout.write(f"[{iterator_result['timestamp']}][ROS_ITERATOR] ROSトピック送信: {status} (type={iterator_result['result_type']}, events={iterator_result['total_events']})\n")
                        sys.stdout.flush()
                    
            except Exception as e:
                if SHOW_BASIC_LOGS:
                    sys.stdout.write(f"[ERROR] VADIterator ROSパブリッシュエラー: {e}\n")
                    sys.stdout.flush()


def runROS(node):
    rclpy.spin(node)

def runTurnTaking():
    TurnTaking()  # ここでモデル実行箇所

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
    node = RosTurnTaking()

    ros = threading.Thread(target=runROS, args=(node,))
    mod = threading.Thread(target=runTurnTaking)

    ros.setDaemon(True)
    mod.setDaemon(True)

    ros.start()
    mod.start()
    shutdown()

if __name__ == '__main__':
    main()
