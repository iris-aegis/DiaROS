# ============================================================
# ログレベル設定
# ============================================================
SHOW_BASIC_LOGS = True
SHOW_DEBUG_LOGS = True

class RosSpeechSynthesis(Node):
    def __init__(self, speechSynthesis):
        super().__init__('speech_synthesis')
        self.speechSynthesis = speechSynthesis

        # RELIABLE QoSプロファイルを定義（分散実行対応）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub_nlg = self.create_subscription(Inlg, 'NLGtoSS', self.play, qos_profile)
        self.pub_ss = self.create_publisher(Iss, 'SStoDM', qos_profile)
        # self.pub_ss = self.create_publisher(Iss, 'SStoDR', qos_profile)
        # self.pub_mm = self.create_publisher(Imm, 'MM', qos_profile)
        # self.pub_wav = self.create_publisher(SynthWav, 'SynthWav', qos_profile)  # ← 削除
        self.timer = self.create_timer(0.0005, self.send)
        self.is_speaking = False

        # 時間計測用
        self.timing_logger = get_timing_logger()
        self.current_session_id = None
        self.tts_start_time = None

    def play(self, nlg):
        if SHOW_DEBUG_LOGS:
            sys.stdout.write(f"[ROS2-SS] 音声合成リクエスト受信: '{nlg.reply}'\n")
            sys.stdout.flush()

        # ★NLGから受信したデータを辞書形式でspeechSynthesisに渡す
        nlg_data = {
            "reply": nlg.reply,
            "source_words": list(nlg.source_words),  # ★音声認識結果リストも渡す
            "request_id": nlg.request_id,  # リクエストID
            "worker_name": nlg.worker_name,  # ワーカースレッド名
            "start_timestamp_ns": nlg.start_timestamp_ns,  # 開始時刻（ナノ秒）
            "completion_timestamp_ns": nlg.completion_timestamp_ns,  # 完了時刻（ナノ秒）
            "inference_duration_ms": nlg.inference_duration_ms  # 推論時間（ミリ秒）
        }
        
        # NLG時刻情報受信完了
        
        # ★Inlgメッセージ受信確認のデバッグ出力（コメントアウト：応答時のみ表示）
        # print(f"[DEBUG][ros2_SS] Inlg受信 - request_id:{nlg.request_id}, worker:{nlg.worker_name}")
        # print(f"[DEBUG][ros2_SS] Inlg受信 - start_ns:{nlg.start_timestamp_ns}, completion_ns:{nlg.completion_timestamp_ns}")
        # print(f"[DEBUG][ros2_SS] Inlg受信 - inference_ms:{nlg.inference_duration_ms}")
        # sys.stdout.flush()
        
        self.speechSynthesis.updateNLG(nlg_data)
        
        # 時間計測: セッションID取得とTTS開始
        if self.current_session_id is None:
            sessions = self.timing_logger.session_data
            if sessions:
                self.current_session_id = list(sessions.keys())[-1]  # 最新セッション
        
        text = str(nlg.reply)
        
        # 時間計測: 音声合成開始
        if self.current_session_id:
            self.tts_start_time = time.time()
            log_tts_start(self.current_session_id, text)
        
        wav_path = self.speechSynthesis.run(text)
        
        # 時間計測: 音声合成完了
        if self.current_session_id and self.tts_start_time:
            tts_duration_ms = (time.time() - self.tts_start_time) * 1000
            log_tts_complete(self.current_session_id, wav_path or "synthesis_failed", tts_duration_ms)
        # ★合成済みファイル名を必ずlast_tts_fileにセット
        if wav_path:
            self.speechSynthesis.last_tts_file = wav_path
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
        
        # ★対話生成結果を送信
        if hasattr(self.speechSynthesis, 'txt'):
            ss.dialogue_text = self.speechSynthesis.txt if self.speechSynthesis.txt else ""
        else:
            ss.dialogue_text = ""
        
        # ★対話生成時刻情報を送信（デバッグ出力追加）
        if hasattr(self.speechSynthesis, 'request_id'):
            ss.request_id = self.speechSynthesis.request_id
        else:
            ss.request_id = 0
            
        if hasattr(self.speechSynthesis, 'worker_name'):
            ss.worker_name = self.speechSynthesis.worker_name
        else:
            ss.worker_name = ""
            
        if hasattr(self.speechSynthesis, 'start_timestamp_ns'):
            ss.start_timestamp_ns = self.speechSynthesis.start_timestamp_ns
        else:
            ss.start_timestamp_ns = 0
            
        if hasattr(self.speechSynthesis, 'completion_timestamp_ns'):
            ss.completion_timestamp_ns = self.speechSynthesis.completion_timestamp_ns
        else:
            ss.completion_timestamp_ns = 0
            
        if hasattr(self.speechSynthesis, 'inference_duration_ms'):
            ss.inference_duration_ms = self.speechSynthesis.inference_duration_ms
        else:
            ss.inference_duration_ms = 0.0
            
        # DM送信処理完了
            
        self.pub_ss.publish(ss)
        self.speechSynthesis.speak_end = False

        mm = Imm()
        mm.mod = "ss"
        # self.pub_mm.publish(mm)

def runROS(pub):
    rclpy.spin(pub)

def shutdown():
    while True:
        key = sys.stdin.readline().strip()
        if key == "kill":
            if SHOW_BASIC_LOGS:
                sys.stdout.write("kill command received.\n")
                sys.stdout.flush()
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