import rclpy
import threading
import sys
import time
from datetime import datetime
from rclpy.node import Node
from interfaces.msg import Iasr
# from interfaces.msg import Isa
from interfaces.msg import Imm
from std_msgs.msg import Float32MultiArray
from diaros.automaticSpeechRecognition import AutomaticSpeechRecognition
import numpy as np
import pygame

class RosAutomaticSpeechRecognition(Node):
    def __init__(self, automaticSpeechRecognition):
        super().__init__('automatic_speech_recognition')
        self.automaticSpeechRecognition = automaticSpeechRecognition
        self.sub_mic = self.create_subscription(Float32MultiArray, 'mic_audio_float32', self.audio_callback, 10)
        self.pub_asr = self.create_publisher(Iasr, 'ASRtoNLU', 1)  # トピック名を変更
        # self.pub_mm = self.create_publisher(Imm, 'MM', 1)
        self.timer = self.create_timer(0.005, self.callback)
        
        # 遅延測定用変数
        self.audio_receive_count = 0
        
        # ビープ音機能の初期化
        self._init_beep_sound()

    def _init_beep_sound(self):
        """ビープ音機能の初期化"""
        try:
            # pygame初期化（音再生用）
            pygame.mixer.init(frequency=22050, size=-16, channels=1, buffer=256)
            self.beep_sound = None
            self._create_beep_sound()
            sys.stdout.write('[ASR] ビープ音機能初期化完了\n')
        except Exception as e:
            sys.stdout.write(f"[WARNING] pygame初期化失敗: {e}\n")
            self.beep_sound = None

    def _create_beep_sound(self):
        """短いビープ音を生成"""
        try:
            # 660Hz (E5) のビープ音を100ms生成（他とは異なる音程）
            duration = 0.1  # 100ms
            sample_rate = 22050
            frames = int(duration * sample_rate)
            
            # サイン波生成（1次元配列でモノラル音声）
            arr = np.zeros(frames)
            for i in range(frames):
                wave = np.sin(2 * np.pi * 660 * i / sample_rate)  # 660Hz
                # フェードイン・フェードアウト
                if i < frames * 0.1:
                    wave *= i / (frames * 0.1)
                elif i > frames * 0.9:
                    wave *= (frames - i) / (frames * 0.1)
                arr[i] = wave * 0.3  # 音量調整
            
            # pygame.Soundオブジェクト作成（1次元配列をint16に変換）
            arr = (arr * 32767).astype(np.int16)
            self.beep_sound = pygame.sndarray.make_sound(arr)
            
        except Exception as e:
            sys.stdout.write(f"[WARNING] ビープ音生成失敗: {e}\n")
            self.beep_sound = None

    def _play_beep(self):
        """ビープ音を再生"""
        try:
            if self.beep_sound is not None:
                self.beep_sound.play()
                sys.stdout.write("[ASR_BEEP] 音声認識結果発行音を再生\n")
                sys.stdout.flush()
        except Exception as e:
            # ビープ音再生エラーは無視（処理継続）
            pass

    def audio_callback(self, msg):
        # ASRで音声データを受信した時刻を記録
        asr_receive_timestamp = time.time()
        audio_np = np.array(msg.data, dtype=np.float32)
        
        # データ追跡用: 先頭3サンプルの値でデータを特定
        data_id = f"{audio_np[0]:.6f},{audio_np[1]:.6f},{audio_np[2]:.6f}" if len(audio_np) >= 3 else "short_data"
        
        self.audio_receive_count += 1
        
        # マイク入力→ASR受信の遅延を表示（毎回）
        # timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        # sys.stdout.write(f"[🔊 ASR_RECEIVE] {timestamp_str} | 受信#{self.audio_receive_count} | ID:{data_id}\n")
        # sys.stdout.flush()
        
        self.automaticSpeechRecognition.update_audio(audio_np)

    def callback(self):
        asr_result = self.automaticSpeechRecognition.pubASR()
        if asr_result is not None:
            # ASR認識結果出力時刻を記録
            asr_output_timestamp = time.time()
            timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            asr = Iasr()
            asr.you = asr_result['you']
            asr.is_final = asr_result['is_final']
            self.pub_asr.publish(asr)
            
            # ASR認識結果の遅延測定ログ出力
            sys.stdout.write(f"[🧠 ASR_OUTPUT] {timestamp_str} | 認識結果: '{asr.you}' | is_final: {asr.is_final}\n")
            sys.stdout.flush()
            
            # ASR結果を発行した後にビープ音を再生
            # self._play_beep()
            
        mm = Imm()
        mm.mod = "asr"

def runROS(pub):
    rclpy.spin(pub)

def runASR(automaticSpeechRecognition):
    automaticSpeechRecognition.run()

def shutdown():
    while True:
        key = input()
        if key == "kill":
            print("kill command received.")
            sys.exit()

def main(args=None):
    rclpy.init(args=args)  # ← ここをノード生成より前に移動
    asr = AutomaticSpeechRecognition()
    rasr = RosAutomaticSpeechRecognition(asr)

    ros = threading.Thread(target=runROS, args=(rasr,))
    mod = threading.Thread(target=runASR, args=(asr,))

    ros.setDaemon(True)
    mod.setDaemon(True)

    ros.start()
    mod.start()
    shutdown()

if __name__ == '__main__':
    main()