# ============================================================
# ログレベル設定
# ============================================================
SHOW_BASIC_LOGS = True   # 基本ログ表示（メッセージ送受信、エラーなど）
SHOW_DEBUG_LOGS = False  # デバッグログ表示（詳細な処理内容、中間データなど）

import rclpy
import threading
import sys
import time
from datetime import datetime
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from interfaces.msg import Iasr
from interfaces.msg import Isa
from interfaces.msg import Iss
from interfaces.msg import Idm
from interfaces.msg import Inlg  # NLG応答用メッセージを追加
from interfaces.msg import Imm
from interfaces.msg import Itt
from interfaces.msg import Ibc  # 追加
from interfaces.msg import Iaa
from diaros.dialogManagement import DialogManagement
import sys
import os
sys.path.append(os.path.expanduser('~/DiaROS_deep_model/DiaROS_py/diaros'))
from playsound import playsound

class RosDialogManagement(Node):
    def __init__(self, dialogManagement):
        super().__init__('dialog_management')
        self.dialogManagement = dialogManagement
        self.prev_word = ""

        # 分散実行対応: RELIABLE QoSプロファイルを設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # ASRからの直接入力を受け付ける（NLUノードが起動していない場合）
        # 優先順位: ASRtoNLU（直接）> NLUtoDM（NLUを経由）
        self.sub_asr_direct = self.create_subscription(Iasr, 'ASRtoNLU', self.dm_update, qos_profile)  # ASRからの直接入力
        # self.sub_lu = self.create_subscription(Iasr, 'NLUtoDM', self.dm_update, qos_profile)  # NaturalLanguageUnderstanding2DialogManagement（nluでは処理を短絡してるのでIasrをつかう）
        self.sub_aa = self.create_subscription(Iaa, 'AAtoDM', self.aa_update, qos_profile)
        self.sub_tt = self.create_subscription(Itt, 'TTtoDM', self.tt_update, qos_profile) # TurnTaking2DialogManagement
        self.sub_bc = self.create_subscription(Ibc, 'BCtoDM', self.bc_update, qos_profile) # BackChannel2DialogManagement
        self.sub_ss = self.create_subscription(Iss, 'SStoDM', self.ss_update, qos_profile)
        self.sub_nlg = self.create_subscription(Inlg, 'NLGtoSS', self.nlg_callback, qos_profile)  # NLGからの応答を購読
        self.pub_dm = self.create_publisher(Idm, 'DMtoNLG', qos_profile)  # QoSをRELIABLEに統一
        # self.pub_mm = self.create_publisher(Imm, 'MM', 1)
        self.timer = self.create_timer(0.001, self.callback)
        self.recv_count = 0  # 受信回数カウンタ追加
        self.prev_recv_time = None  # 前回受信時刻

        # ★リクエストID生成用カウンター
        self.request_id_counter = 0  # リクエストIDのカウンター
        self.current_request_stage = None  # 現在処理中のステージ
        # ★Second stage 2.5秒間隔ASR履歴キャッシュ（NLGで使用）
        self.second_stage_asr_history_2_5s = []


    def dm_update(self, dm):
        new = { "you": dm.you, "is_final": dm.is_final, "timestamp_ns": dm.timestamp_ns }
        self.dialogManagement.updateASR(new)

        # ★デバッグ用：ASR結果受信ログ（毎回出力）
        if SHOW_DEBUG_LOGS:
            from datetime import datetime
            timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            sys.stdout.write(f"[{timestamp_str}][DM_UPDATE-ASR] 受信: '{dm.you}' (len={len(dm.you)}) | is_final: {dm.is_final}\n")
            sys.stdout.flush()
        
    def ss_update(self, ss):# test
        new = {
            "is_speaking": ss.is_speaking,
            "timestamp": ss.timestamp,
            "filename": ss.filename,  # ← 追加: 合成音声ファイル名を渡す
            "dialogue_text": ss.dialogue_text,  # ★追加: 対話生成結果を渡す
            # ★NLGタイミング情報を追加
            "request_id": ss.request_id,
            "worker_name": ss.worker_name,
            "start_timestamp_ns": ss.start_timestamp_ns,
            "completion_timestamp_ns": ss.completion_timestamp_ns,
            "inference_duration_ms": ss.inference_duration_ms,
            # TTSタイミング情報も追加（将来の拡張用）
            "tts_start_timestamp_ns": getattr(ss, 'tts_start_timestamp_ns', 0),
            "tts_completion_timestamp_ns": getattr(ss, 'tts_completion_timestamp_ns', 0)
        }
        # print(f"[SSトピック受信] is_speaking: {new['is_speaking']} / timestamp: {new['timestamp']}")  # 確認用
        self.dialogManagement.updateSS(new)

    def tt_update(self, msg):
        import datetime
        tt_receive_time = datetime.datetime.now()
        tt_receive_timestamp = tt_receive_time.strftime('%H:%M:%S.%f')[:-3]
        
        data = {
            'result': msg.result,
            'confidence': msg.confidence
        }
        self.dialogManagement.updateTT(data)

        # print(f"[{tt_receive_timestamp}][DM_TT] TT結果受信 (result={msg.result}, conf={msg.confidence:.3f})")
        # sys.stdout.flush()

    def bc_update(self, msg):
        data = {
            'result': msg.result,
            'confidence': msg.confidence
        }
        self.recv_count += 1
        now = time.time()
        if self.prev_recv_time is not None:
            elapsed_ms = (now - self.prev_recv_time) * 1000
        else:
            elapsed_ms = 0.0
        self.prev_recv_time = now
        # バーでconfidenceを表示 + 現在時刻（msまで）
        # bar_len = int(round(float(data['confidence']) * 10))
        # bar = '■' * bar_len + ' ' * (10 - bar_len)
        # now_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now*1000)%1000):03d}"
        # print(f"[ros2_dm.py] Recv#{self.recv_count} {now_str} result={data['result']} confidence={data['confidence']:.10f}")
        # sys.stdout.flush()
        self.dialogManagement.updateBC(data)  # dialogManagement.py側でupdateBCを実装しておくこと

    def nlg_callback(self, msg):
        """NLGからの応答を受信（ステージ情報付き）"""
        stage = msg.stage if hasattr(msg, 'stage') else 'first'
        reply = msg.reply
        request_id = getattr(msg, 'request_id', 0)

        # ★ステージ完了を記録
        stage_name = "相槌生成" if stage == "first" else "応答生成" if stage == "second" else "不明"
        if SHOW_BASIC_LOGS:
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self.get_logger().info(
                f"[{timestamp}] [DM] NLGから{stage_name}応答受信 (request_id={request_id}): '{reply[:30]}...'"
            )

        nlg_data = {
            'stage': stage,
            'reply': reply,
            'request_id': request_id,  # ★リクエストIDを含める
            'filename': '',  # filenameはSStoDM経由で受け取るため、ここでは空
            # ★NLGのタイミング情報も伝播
            'nlg_start_timestamp_ns': getattr(msg, 'start_timestamp_ns', 0),
            'nlg_completion_timestamp_ns': getattr(msg, 'completion_timestamp_ns', 0),
            'nlg_inference_duration_ms': getattr(msg, 'inference_duration_ms', 0.0)
        }

        self.dialogManagement.updateNLG(nlg_data)

    def callback(self):
        # First stage相槌生成リクエスト
        dm = Idm()
        pub_dm_return = self.dialogManagement.pubDM()
        words = pub_dm_return['words']
        dm_result_update = pub_dm_return['update']
        stage = pub_dm_return.get('stage', 'first')

        # ★デバッグ：pubDM() の返り値をログ（100回おきに出力）
        if not hasattr(self, 'callback_count'):
            self.callback_count = 0
        self.callback_count += 1
        if self.callback_count % 100 == 0:
            if SHOW_DEBUG_LOGS:
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                self.get_logger().info(
                    f"[{timestamp}] [DEBUG-CALLBACK] pubDM return: update={dm_result_update}, words_count={len(words)}"
                )

        if dm_result_update is True:
            # ★毎回リクエストIDをインクリメント（重複検出の精度向上）
            self.request_id_counter += 1
            if stage != self.current_request_stage:
                self.current_request_stage = stage

            dm.words = words
            dm.stage = stage
            dm.request_id = self.request_id_counter  # ★ユニークなリクエストIDを設定（毎回increment）
            dm.session_id = getattr(self.dialogManagement, 'current_session_id', '')
            # ★TurnTaking判定時刻を送信（分散実行時のNLG連携用）
            dm.turn_taking_decision_timestamp_ns = getattr(self.dialogManagement, 'turn_taking_decision_timestamp_ns', 0)
            # ★First stage用の相槌内容とASR履歴を設定
            dm.first_stage_backchannel_at_tt = getattr(self.dialogManagement, 'first_stage_backchannel_at_tt_decision', '')
            dm.asr_history_2_5s = getattr(self.dialogManagement, 'asr_history_at_tt_decision_2_5s', [])

            # ★DM→NLG送信ログ
            stage_name = "相槌生成" if stage == "first" else "応答生成" if stage == "second" else "不明"
            if SHOW_BASIC_LOGS:
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                self.get_logger().info(
                    f"[{timestamp}] [DM] {stage_name}リクエスト送信 (request_id={self.request_id_counter}, 入力数={len(words)})"
                )

            self.prev_word = words[0] if words else ""
            self.pub_dm.publish(dm)

        # Second stage応答生成リクエスト送信
        if hasattr(self.dialogManagement, 'second_stage_request_pending') and self.dialogManagement.second_stage_request_pending:
            self.dialogManagement.second_stage_request_pending = False

            # ★デバッグ：second_stageリクエスト処理開始
            if SHOW_DEBUG_LOGS:
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                self.get_logger().info(
                    f"[{timestamp}] [DEBUG] Second stage リクエスト処理開始"
                )

            dm_data_second = self.dialogManagement.pubDM_second_stage()
            if dm_data_second["update"]:
                # ★新しいステージの場合、リクエストIDを更新
                if "second" != self.current_request_stage:
                    self.request_id_counter += 1
                    self.current_request_stage = "second"

                msg = Idm()
                # ★修正：TurnTaking判定時に保存したASR履歴を送信
                # Second stageでは、このASR履歴を使用して応答を生成
                msg.words = dm_data_second.get("words", [])

                msg.stage = "second"
                msg.request_id = self.request_id_counter  # ★リクエストIDを設定
                msg.session_id = getattr(self.dialogManagement, 'current_session_id', '')
                # ★TurnTaking判定時刻を送信（pubDM_second_stage()で返されるデータに含まれている）
                msg.turn_taking_decision_timestamp_ns = dm_data_second.get("turn_taking_decision_timestamp_ns", 0)
                # ★TurnTaking判定時に再生する相槌内容を送信（Second stage用）
                msg.first_stage_backchannel_at_tt = dm_data_second.get("first_stage_backchannel_at_tt", "")
                # ★2.5秒間隔ASR履歴をメッセージに設定（ROS2メッセージで送信）
                msg.asr_history_2_5s = dm_data_second.get("asr_history_2_5s", [])
                # ★2.5秒間隔ASR履歴をキャッシュに保存（ローカル参照用）
                self.second_stage_asr_history_2_5s = dm_data_second.get("asr_history_2_5s", [])

                # ★デバッグ：送信前のメッセージ内容確認
                if SHOW_DEBUG_LOGS:
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    self.get_logger().info(
                        f"[{timestamp}] [DEBUG] Second stageメッセージ送信: stage='{msg.stage}', request_id={msg.request_id}, words={len(msg.words)}件"
                    )

                # ★ログ出力
                if SHOW_BASIC_LOGS:
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    self.get_logger().info(
                        f"[{timestamp}] [DM] 応答生成リクエスト送信 (request_id={self.request_id_counter}, 入力数={len(msg.words)})"
                    )

                self.pub_dm.publish(msg)
            else:
                # ★デバッグ：second_stage更新フラグがfalseの場合
                if SHOW_DEBUG_LOGS:
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    self.get_logger().info(
                        f"[{timestamp}] [DEBUG] Second stage 更新なし（update=False）"
                    )


    def aa_update(self, msg):
        new = {
            "prevgrad": 0.0,
            "frequency": 0.0,
            "grad": msg.grad,
            "power": msg.power,
            "zerocross": msg.zerocross
        }
        self.dialogManagement.updateSA(new)

    # def wav_play(self, msg):
    #     filename = msg.filename
    #     if filename:
    #         try:
    #             playsound(filename, True)
    #         except Exception as e:
    #             print(f"[DM] playsound error: {e}")

    # def callback(self):# Admhive wordの内容が変更されていたら対話生成していた
    #     dm = Idm()
    #     now_word = self.dialogManagement.pubDM()['word']
    #     dm.word = now_word if self.prev_word != now_word else ""
    #     self.prev_word = now_word
    #     print(dm.word)
    #     self.pub_dm.publish(dm)
    

def runROS(pub):
    rclpy.spin(pub)

def runDM(dialogManagement):
    dialogManagement.run()

def shutdown():
    while True:
        key = sys.stdin.readline().strip()
        if key == "kill":
            if SHOW_BASIC_LOGS:
                sys.stdout.write("kill command received.\n")
                sys.stdout.flush()
            sys.exit()

def main(args=None):
    dm = DialogManagement()
    rclpy.init(args=args)
    rdm = RosDialogManagement(dm)

    ros = threading.Thread(target=runROS, args=(rdm,))
    mod = threading.Thread(target=runDM, args=(dm,))

    ros.setDaemon(True)
    mod.setDaemon(True)

    ros.start()
    mod.start()
    shutdown()

if __name__ == '__main__':
    main()
