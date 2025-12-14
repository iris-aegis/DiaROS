import rclpy
import threading
import sys
import time
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from interfaces.msg import Idm
from interfaces.msg import Inlg
from interfaces.msg import Imm
from diaros.naturalLanguageGeneration import NaturalLanguageGeneration

class RosNaturalLanguageGeneration(Node):
    def __init__(self, naturalLanguageGeneration):
        super().__init__('natural_language_generation')
        self.naturalLanguageGeneration = naturalLanguageGeneration

        # 分散実行対応: RELIABLE QoSプロファイルを設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub_dm = self.create_subscription(Idm, 'DMtoNLG', self.dm_update, qos_profile)
        self.pub_nlg = self.create_publisher(Inlg, 'NLGtoSS', qos_profile)  # NLG→SpeechSynthesis用（QoSをRELIABLEに統一）
        # self.pub_nlg_dr = self.create_publisher(Inlg, 'NLGtoDR', 1)
        # self.pub_mm = self.create_publisher(Imm, 'MM', 1)
        self.timer = self.create_timer(0.02, self.ping)
        self.last_sent_reply = None

        # ステージ管理用フィールド
        self.current_stage = None  # 現在処理中のステージ
        self.current_request_id = None  # 現在処理中のリクエストID
        self.stage_start_timestamp_ns = 0  # ステージ開始時刻（ナノ秒）
        # ★2.5秒間隔ASR履歴（ROS2メッセージから抽出）
        self.asr_history_2_5s = []

        # ★重複リクエスト防止：現在処理中のリクエストを記録
        self.processing_request_id = None
        self.processing_stage = None

    def dm_update(self, msg):
        """DMからのリクエストを受信（非同期処理）"""
        words = list(msg.words)
        stage = getattr(msg, 'stage', 'first')  # stageフィールドを取得
        request_id = getattr(msg, 'request_id', 0)
        turn_taking_decision_timestamp_ns = getattr(msg, 'turn_taking_decision_timestamp_ns', 0)
        first_stage_backchannel_at_tt = getattr(msg, 'first_stage_backchannel_at_tt', '')  # ★TurnTaking判定時の相槌内容
        # ★2.5秒間隔ASR履歴を抽出（ROS2メッセージから）
        asr_history_2_5s = list(getattr(msg, 'asr_history_2_5s', []))
        # ★インスタンス変数に保存（NLGで使用）
        self.asr_history_2_5s = asr_history_2_5s

        # ★デバッグ：受け取ったメッセージの詳細ログ
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.get_logger().info(
            f"[{timestamp}] [NLG-DEBUG] DM受信: words={len(words)}件, stage='{stage}', request_id={request_id}, msg.stage属性={hasattr(msg, 'stage')}"
        )

        # ★修正：Second stageでは空のwordsでも処理を続ける（first_stage_responseを使用するため）
        if words or stage == 'second':
            # ★重複リクエスト防止：現在処理中のリクエストと同じ場合はスキップ
            if request_id == self.processing_request_id and stage == self.processing_stage:
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                self.get_logger().info(
                    f"[{timestamp}] [NLG] 重複リクエストをスキップ (request_id={request_id}, stage={stage})"
                )
                return

            # ★新しいリクエストの開始を記録
            if request_id != self.current_request_id or stage != self.current_stage:
                self.current_stage = stage
                self.current_request_id = request_id
                self.stage_start_timestamp_ns = time.time_ns()

                # ステージ開始ログ
                stage_name = "相槌生成" if stage == "first" else "応答生成" if stage == "second" else "不明"
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                self.get_logger().info(
                    f"[{timestamp}] [NLG] {stage_name}ステージ開始 (request_id={request_id}, 入力数={len(words)})"
                )

            # ★【重要】update() をスレッドで非同期実行
            # ROS2 コールバックをブロックせず、複数のリクエストを並列処理可能に
            # ★【修正】ラムダ関数を使ってパラメータをキャプチャ（遅延評価を防止）
            # ★処理開始前に処理中フラグを設定（重複処理を防止）
            self.processing_request_id = request_id
            self.processing_stage = stage

            update_thread = threading.Thread(
                target=lambda w=words, s=stage, t=turn_taking_decision_timestamp_ns, bc=first_stage_backchannel_at_tt, asr_2_5s=asr_history_2_5s:
                        self.naturalLanguageGeneration.update(w, stage=s, turn_taking_decision_timestamp_ns=t, first_stage_backchannel_at_tt=bc, asr_history_2_5s=asr_2_5s),
                daemon=True
            )
            update_thread.start()

    def ping(self):
        """NLGが応答を生成したら、ステージ情報と共に送信"""
        # 応答が生成されたらpublish
        if hasattr(self.naturalLanguageGeneration, "last_reply") and self.naturalLanguageGeneration.last_reply != self.last_sent_reply:
            nlg_msg = Inlg()
            nlg_msg.reply = self.naturalLanguageGeneration.last_reply

            # ★ステージ情報を設定
            nlg_msg.stage = self.current_stage if self.current_stage else "first"
            nlg_msg.request_id = self.current_request_id if self.current_request_id else 0

            # ★音声認識結果リストも送信
            if hasattr(self.naturalLanguageGeneration, "source_words"):
                nlg_msg.source_words = self.naturalLanguageGeneration.source_words
            else:
                nlg_msg.source_words = []

            # ★新しい時刻情報フィールドを送信
            nlg_msg.worker_name = getattr(self.naturalLanguageGeneration, "worker_name", "")
            nlg_msg.start_timestamp_ns = getattr(self.naturalLanguageGeneration, "start_timestamp_ns", 0)
            nlg_msg.completion_timestamp_ns = getattr(self.naturalLanguageGeneration, "completion_timestamp_ns", 0)
            nlg_msg.inference_duration_ms = getattr(self.naturalLanguageGeneration, "inference_duration_ms", 0.0)

            # ステージ開始から完了までの時間を計測
            if self.stage_start_timestamp_ns > 0:
                stage_duration_ms = (time.time_ns() - self.stage_start_timestamp_ns) / 1_000_000
            else:
                stage_duration_ms = 0.0

            # ステージ完了ログ
            stage_name = "相槌生成" if self.current_stage == "first" else "応答生成" if self.current_stage == "second" else "不明"
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self.get_logger().info(
                f"[{timestamp}] [NLG] {stage_name}ステージ完了 (request_id={self.current_request_id}, "
                f"処理時間={stage_duration_ms:.1f}ms, 応答='{nlg_msg.reply[:30]}...' {'← お疲れ様' if len(nlg_msg.reply) > 30 else ''})"
            )

            self.pub_nlg.publish(nlg_msg)  # NLG生成文とステージ情報をNLGtoSSトピックで送信
            # self.pub_nlg_dr.publish(nlg_msg)  # ← コメントアウト
            self.last_sent_reply = self.naturalLanguageGeneration.last_reply

            # ★処理中フラグをリセット（次のリクエストを受け付けるため）
            self.processing_request_id = None
            self.processing_stage = None

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
    # ★ROS2NLG参照を設定（NLGから2.5秒間隔ASR履歴を取得するため）
    naturalLanguageGeneration.rnlg_ref = rnlg

    ros = threading.Thread(target=runROS, args=(rnlg,))
    mod = threading.Thread(target=runNLG, args=(naturalLanguageGeneration,))

    ros.setDaemon(True)
    mod.setDaemon(True)

    ros.start()
    mod.start()
    shutdown()

if __name__ == '__main__':
    main()