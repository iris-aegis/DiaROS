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
        batch_audio_ids = []
        batch_timestamps = []
        
        while not stream_queue.empty():
            try:
                queue_data = stream_queue.get_nowait()
                receive_timestamp = time.time()
                
                # メタデータ付きデータかチェック
                if isinstance(queue_data, dict) and 'audio_data' in queue_data:
                    # 新しいメタデータ付きフォーマット
                    audio_data = queue_data['audio_data']
                    audio_id = queue_data['audio_id']
                    capture_timestamp = queue_data['capture_timestamp']
                    sample_count = queue_data['sample_count']
                    
                    float_array = np.frombuffer(audio_data, dtype=np.float32)
                    batch_audio_ids.append(audio_id)
                    batch_timestamps.append(capture_timestamp)
                    
                    # 詳細ログ（コメントアウト）
                    # if not hasattr(self, 'detailed_log_counter'):
                    #     self.detailed_log_counter = 0
                    # self.detailed_log_counter += 1
                    # if self.detailed_log_counter % 10 == 0:
                    #     latency_ms = (receive_timestamp - capture_timestamp) * 1000
                    #     timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    #     
                    #     # データ対応確認用: 先頭5サンプルの具体的な値を表示
                    #     sample_values = float_array[:5].tolist() if len(float_array) >= 5 else float_array.tolist()
                    #     sample_str = '[' + ','.join([f'{v:.6f}' for v in sample_values]) + ']'
                    #     
                    #     # 音声データの統計値も追加
                    #     rms = np.sqrt(np.mean(float_array**2))
                    #     max_val = np.max(np.abs(float_array))
                    #     
                    #     sys.stdout.write(f"[📥 QUEUE_RECEIVE] {timestamp_str} | AudioID:{audio_id} | QueueLatency:{latency_ms:.2f}ms | Samples:{sample_count} | Data:{sample_str} | RMS:{rms:.6f} | Max:{max_val:.6f}\n")
                    #     sys.stdout.flush()
                        
                else:
                    # 旧フォーマット（バックワード互換性）
                    audio_data = queue_data
                    float_array = np.frombuffer(audio_data, dtype=np.float32)
                    data_id = f"{float_array[0]:.6f},{float_array[1]:.6f},{float_array[2]:.6f}" if len(float_array) >= 3 else "short_data"
                    batch_audio_ids.append(data_id)
                    batch_timestamps.append(receive_timestamp)
                
                self.pending_data.append(float_array)
            except queue.Empty:
                break
        
        # 送信条件をチェック（超高速監視では即座に送信）
        should_send = (
            len(self.pending_data) >= 1  # 1個でも即座に送信
        )
        
        if should_send:
            self._send_batched_data(batch_audio_ids, batch_timestamps)
    
    def _send_batched_data(self, batch_audio_ids=None, batch_timestamps=None):
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
        
        # 音声IDと遅延情報を含むログ出力（コメントアウト）
        # if batch_audio_ids and batch_timestamps:
        #     # 最も古いタイムスタンプから最新までの遅延
        #     oldest_timestamp = min(batch_timestamps)
        #     total_latency_ms = (mic_timestamp - oldest_timestamp) * 1000
        #     audio_ids_str = ','.join(batch_audio_ids[:3]) if len(batch_audio_ids) <= 3 else f"{','.join(batch_audio_ids[:2])}...(+{len(batch_audio_ids)-2})"
        #     
        #     timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        #     sys.stdout.write(f"[🚀 ROS_PUBLISH] {timestamp_str} | 送信#{self.send_count} | IDs:[{audio_ids_str}] | Latency:{total_latency_ms:.2f}ms | Batch:{batch_size} | Samples:{len(combined_data)}\n")
        # else:
        #     # フォールバック（旧形式）
        #     data_id = f"{combined_data[0]:.6f},{combined_data[1]:.6f},{combined_data[2]:.6f}" if len(combined_data) >= 3 else "short_data"
        #     timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        #     sys.stdout.write(f"[🚀 ROS_PUBLISH] {timestamp_str} | 送信#{self.send_count} | ID:{data_id} | Batch:{batch_size} | Samples:{len(combined_data)}\n")
        # 
        # sys.stdout.flush()
        
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
