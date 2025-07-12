import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import sys
import queue
import time
from datetime import datetime

from diaros.speechInput import stream_queue, SpeechInput
import diaros.speechInput as speechInput_module

class MicPublisher(Node):
    def __init__(self):
        super().__init__('speech_input')
        self.pub_mic = self.create_publisher(Float32MultiArray, 'mic_audio_float32', 50)
        
        # 即座に送信するための設定
        self.send_count = 0
        self.pending_data = []  # まとめ送信用のバッファ
        self.last_send_time = time.time()
        self.batch_size_threshold = 3  # 3個以上溜まったらまとめて送信
        self.time_threshold = 0.01  # 10ms以上経過したら強制送信
        
        
        # speechInput側からの通知を受けるため自身を登録
        speechInput_module.ros_publisher = self
        
        # 超高速キュー監視タイマー（1ms間隔）
        self.fast_timer = self.create_timer(0.001, self.fast_queue_check)
        # 定期的な監視タイマー（低頻度）
        self.timer = self.create_timer(0.1, self.monitor_queue)


    def fast_queue_check(self):
        """超高速キュー監視（1ms間隔）"""
        if not stream_queue.empty():
            self._process_queue_immediate()
    
    def _notify_new_data(self):
        """speechInput.pyからの通知を受けて即座に送信処理"""
        self._process_queue_immediate()
    
    def _process_queue_immediate(self):
        """キューからデータを取得して即座に送信またはバッファリング"""
        current_time = time.time()
        
        # キューから利用可能な全データを取得
        while not stream_queue.empty():
            try:
                data = stream_queue.get_nowait()
                float_array = np.frombuffer(data, dtype=np.float32)
                
                # データ追跡用: 先頭3サンプルの値でデータを特定
                data_id = f"{float_array[0]:.6f},{float_array[1]:.6f},{float_array[2]:.6f}" if len(float_array) >= 3 else "short_data"
                # timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                # sys.stdout.write(f"[📤 QUEUE_GET] {timestamp_str} | キューから取得 | ID:{data_id} | queue_size:{stream_queue.qsize()}\n")
                # sys.stdout.flush()
                
                self.pending_data.append(float_array)
            except queue.Empty:
                break
        
        # 送信条件をチェック（超高速監視では即座に送信）
        should_send = (
            len(self.pending_data) >= 1  # 1個でも即座に送信
        )
        
        if should_send:
            self._send_batched_data()
    
    def _send_batched_data(self):
        """バッファされたデータをまとめて送信"""
        if not self.pending_data:
            return
            
        # 複数のチャンクをまとめて1つのメッセージとして送信
        if len(self.pending_data) == 1:
            # 1個の場合はそのまま送信
            combined_data = self.pending_data[0]
        else:
            # 複数の場合は連結
            combined_data = np.concatenate(self.pending_data)
        
        # マイク入力タイムスタンプを記録
        mic_timestamp = time.time()
        
        msg_mic = Float32MultiArray()
        msg_mic.data = combined_data.tolist()
        self.pub_mic.publish(msg_mic)
        
        self.send_count += 1
        batch_size = len(self.pending_data)
        
        # データ追跡用: 先頭3サンプルの値でデータを特定  
        data_id = f"{combined_data[0]:.6f},{combined_data[1]:.6f},{combined_data[2]:.6f}" if len(combined_data) >= 3 else "short_data"
        
        # マイク入力遅延測定用ログ出力（毎回表示）
        timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        sys.stdout.write(f"[🎤 MIC_INPUT] {timestamp_str} | 送信#{self.send_count} | ID:{data_id} | pending:{batch_size} | combined_len:{len(combined_data)}\n")
        sys.stdout.flush()
        
        # バッファをクリア
        self.pending_data = []
        self.last_send_time = mic_timestamp

    def monitor_queue(self):
        """定期監視とフォールバック処理"""
        queue_size = stream_queue.qsize()
        current_time = time.time()
        
        # 強制フラッシュ（念のため）- 一時的にコメントアウト
        # if self.pending_data and current_time - self.last_send_time >= self.time_threshold * 2:
        #     sys.stdout.write(f"[INFO] 強制フラッシュ実行, pending={len(self.pending_data)}\n")
        #     self._send_batched_data()
        
        # キューが蓄積している場合の警告
        if queue_size >= 5:
            sys.stdout.write(f"[WARNING] stream_queue蓄積: サイズ={queue_size}\n")
            sys.stdout.flush()
            # 緊急処理
            self._process_queue_immediate()
            
        # 定期的な状況報告
        if hasattr(self, 'monitor_count'):
            self.monitor_count += 1
        else:
            self.monitor_count = 1
            
        if self.monitor_count % 50 == 0:  # 5秒ごと
            sys.stdout.write(f"[INFO] 超高速監視モード動作中, queue_size={queue_size}, pending={len(self.pending_data)}\n")
            sys.stdout.flush()


def runROS(node):
    rclpy.spin(node)

def runSpeechInput():
    speech_input = SpeechInput(16000, 160, 0)  # 10msチャンク
    try:
        while True:
            # SpeechInputは内部でマイク監視ループを持つため何もしない
            pass
    except KeyboardInterrupt:
        pass

def shutdown():
    while True:
        key = input()
        if key == "kill":
            print("kill command received.")
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    mic_publisher = MicPublisher()
    # SpeechInputを別スレッドで起動
    mic_thread = threading.Thread(target=runSpeechInput)
    mic_thread.setDaemon(True)
    mic_thread.start()
    # runROSをマルチスレッドで起動
    ros_thread = threading.Thread(target=runROS, args=(mic_publisher,))
    ros_thread.setDaemon(True)
    ros_thread.start()
    shutdown()
    mic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
