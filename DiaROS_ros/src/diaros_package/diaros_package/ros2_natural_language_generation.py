import rclpy
import threading
import sys
import time
from datetime import datetime
from rclpy.node import Node
from interfaces.msg import Idm
from interfaces.msg import Inlg
from interfaces.msg import Imm
from diaros.naturalLanguageGeneration import NaturalLanguageGeneration
from diaros.timeTracker import get_time_tracker
from diaros.timing_integration import get_timing_logger, start_timing_session, log_nlg_start, log_nlg_complete, log_nlg_send
class RosNaturalLanguageGeneration(Node):
    def __init__(self, naturalLanguageGeneration):
        super().__init__('natural_language_generation')
        self.naturalLanguageGeneration = naturalLanguageGeneration
        self.sub_dm = self.create_subscription(Idm, 'DMtoNLG', self.dm_update, 1)
        self.pub_nlg = self.create_publisher(Inlg, 'NLGtoSS', 1)  # NLG→SpeechSynthesis用
        # self.pub_nlg_dr = self.create_publisher(Inlg, 'NLGtoDR', 1)
        # self.pub_mm = self.create_publisher(Imm, 'MM', 1)
        self.timer = self.create_timer(0.0005, self.ping)
        self.last_sent_reply = None
        self.last_sent_source_words = None
        
        # タイムトラッカーの初期化
        self.time_tracker = get_time_tracker("nlg_pc")
        self.current_session_id = None
        
        # Docker統合時間計測の初期化
        self.timing_logger = get_timing_logger()
        self.nlg_start_time = None

    def dm_update(self, msg):
        words = list(msg.words)
        session_id = getattr(msg, 'session_id', None)
        
        # すべて空文字列なら送らない
        if words and any(w.strip() for w in words):
            now = datetime.now()
            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[{timestamp}][NLG] 音声認識結果受信時刻: {timestamp}\n")
            sys.stdout.write(f"[{timestamp}][NLG] 受信した音声認識結果リスト: {words}\n")
            
            # セッションIDの管理
            if session_id:
                self.current_session_id = session_id
                sys.stdout.write(f"[{timestamp}][NLG] セッションID: {session_id}\n")
                
                # NLG処理開始チェックポイント
                self.time_tracker.add_checkpoint(session_id, "nlg", "processing_start", {
                    "asr_words": words,
                    "word_count": len(words)
                })
                
                # NLGクラスにセッションIDを設定
                self.naturalLanguageGeneration.set_session_id(session_id)
            else:
                # セッションIDがない場合は新規作成
                self.current_session_id = start_timing_session()
                session_id = self.current_session_id
                sys.stdout.write(f"[{timestamp}][NLG] 新規セッションID作成: {session_id}\n")
            
            # Docker統合時間計測: NLG開始ログ
            self.nlg_start_time = time.time()
            dialogue_context = ' '.join(words)
            log_nlg_start(session_id, dialogue_context)
            
            self.naturalLanguageGeneration.update(words)
            
    def ping(self):
        # 応答が生成されたらpublish
        if (
            self.naturalLanguageGeneration.last_reply != self.last_sent_reply
            and self.naturalLanguageGeneration.last_reply != ""
        ):
            nlg_msg = Inlg()
            nlg_msg.reply = self.naturalLanguageGeneration.last_reply
            # 対話生成の元にした音声認識結果も送信
            if hasattr(self.naturalLanguageGeneration, "last_source_words") and self.naturalLanguageGeneration.last_source_words:
                nlg_msg.source_words = self.naturalLanguageGeneration.last_source_words
            else:
                nlg_msg.source_words = []
            
            now = datetime.now()
            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            
            # ★新しい時刻情報フィールドを送信
            nlg_msg.request_id = getattr(self.naturalLanguageGeneration, "request_id", 0)
            nlg_msg.worker_name = getattr(self.naturalLanguageGeneration, "worker_name", "")
            nlg_msg.start_timestamp_ns = getattr(self.naturalLanguageGeneration, "start_timestamp_ns", 0)
            nlg_msg.completion_timestamp_ns = getattr(self.naturalLanguageGeneration, "completion_timestamp_ns", 0)
            nlg_msg.inference_duration_ms = getattr(self.naturalLanguageGeneration, "inference_duration_ms", 0.0)
            
            # セッションIDを含める
            nlg_msg.session_id = self.current_session_id or ""
            
            # NLG処理完了チェックポイント
            if self.current_session_id:
                self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "processing_complete", {
                    "response": nlg_msg.reply,
                    "source_words": list(nlg_msg.source_words),
                    "inference_duration_ms": nlg_msg.inference_duration_ms
                })
                
                # Docker統合時間計測: NLG完了ログ
                if self.nlg_start_time:
                    processing_time_ms = (time.time() - self.nlg_start_time) * 1000
                    log_nlg_complete(self.current_session_id, nlg_msg.reply, processing_time_ms)
                
                # Docker統合時間計測: メッセージ送信ログ
                message_data = {
                    "reply": nlg_msg.reply,
                    "source_words": list(nlg_msg.source_words),
                    "session_id": nlg_msg.session_id,
                    "inference_duration_ms": nlg_msg.inference_duration_ms
                }
                log_nlg_send(self.current_session_id, message_data)
            
            # 対話生成タイミング情報の読みやすい出力
            if nlg_msg.start_timestamp_ns > 0 and nlg_msg.completion_timestamp_ns > 0:
                start_time = datetime.fromtimestamp(nlg_msg.start_timestamp_ns / 1_000_000_000)
                completion_time = datetime.fromtimestamp(nlg_msg.completion_timestamp_ns / 1_000_000_000)
                sys.stdout.write(f"[{timestamp}][NLG TIMING] 対話生成開始: {start_time.strftime('%H:%M:%S.%f')[:-3]}\n")
                sys.stdout.write(f"[{timestamp}][NLG TIMING] 対話生成完了: {completion_time.strftime('%H:%M:%S.%f')[:-3]}\n")
                sys.stdout.write(f"[{timestamp}][NLG TIMING] 生成時間: {nlg_msg.inference_duration_ms:.1f}ms\n")
            else:
                sys.stdout.write(f"[{timestamp}][NLG WARNING] タイムスタンプが0です: start={nlg_msg.start_timestamp_ns}, completion={nlg_msg.completion_timestamp_ns}\n")
            sys.stdout.write(f"[{timestamp}][NLG] 対話生成結果送信時刻: {timestamp}\n")
            sys.stdout.write(f"[{timestamp}][NLG] 送信する対話生成内容: {nlg_msg.reply}\n")
            sys.stdout.write(f"[{timestamp}][NLG] 送信する音声認識結果: {nlg_msg.source_words}\n")
            sys.stdout.write(f"[{timestamp}][NLG] 送信するワーカー情報: ID={nlg_msg.request_id}, Worker={nlg_msg.worker_name}, Duration={nlg_msg.inference_duration_ms:.1f}ms\n")
            self.pub_nlg.publish(nlg_msg)  # ここでNLG生成文とsource_wordsをNLGtoSSトピックで送信
            self.last_sent_reply = self.naturalLanguageGeneration.last_reply
            self.last_sent_source_words = self.naturalLanguageGeneration.last_source_words
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