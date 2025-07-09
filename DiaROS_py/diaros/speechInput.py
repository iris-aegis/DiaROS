### speechInput.py ###
import time
import pyaudio
from six.moves import queue
import sys
import rclpy
from std_msgs.msg import Float32MultiArray
import numpy as np

STREAMING_LIMIT = 10000

# グローバル共有キュー
stream_queue = queue.Queue()
# ROS Publisher のグローバル参照
ros_publisher = None

class SpeechInput:
    def __init__(self, rate, chunk_size, device):
        sys.stdout.write('speechInput start\n')
        self._rate = rate
        self.chunk_size = 160  # 10ms @ 16kHz
        self._num_channels = 1
        self._buff = queue.Queue()
        self.closed = True
        self.start_time = int(round(time.time() * 1000))
        self.restart_counter = 0
        self.audio_input = []
        self.last_audio_input = []
        self.result_end_time = 0
        self.is_final_end_time = 0
        self.final_request_end_time = 0
        self.bridging_offset = 0
        self.last_transcript_was_final = False
        self.new_stream = True
        self._audio_interface = pyaudio.PyAudio()
        self.mic_info = self._audio_interface.get_default_input_device_info()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paFloat32,
            channels=self._num_channels,
            rate=self._rate,
            input=True,
            input_device_index=self.mic_info["index"],
            frames_per_buffer=self.chunk_size,
            stream_callback=self._fill_buffer,
        )
        # acousticAnalysisの独立に伴い無効化
        # self.frequency = 0.0
        # self.grad = 0.0
        # self.power = 0.0
        # self.zerocross = 0
        # self.prevgrad = 0.0

    def __enter__(self):
        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, *args, **kwargs):
        # 音声データの具体的な値でデータ追跡用ログ
        import time
        from datetime import datetime
        import numpy as np
        
        # 音声データを浮動小数点配列に変換
        audio_np = np.frombuffer(in_data, dtype=np.float32)
        # データ識別用: 先頭3サンプルの値でデータを特定
        data_id = f"{audio_np[0]:.6f},{audio_np[1]:.6f},{audio_np[2]:.6f}" if len(audio_np) >= 3 else "short_data"
        
        # SDS音声取得タイムスタンプをログ出力（10回に1回）
        capture_timestamp = time.time()
        if not hasattr(self, 'capture_log_counter'):
            self.capture_log_counter = 0
        self.capture_log_counter += 1
        if self.capture_log_counter % 10 == 0:
            timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            sys.stdout.write(f"[🎙️ SDS_CAPTURE] {timestamp_str} | ID:{data_id} | T:{capture_timestamp:.6f}\n")
            sys.stdout.flush()
        
        # キューにデータを追加
        stream_queue.put(in_data)
        self._buff.put(in_data)
        
        # queueに追加されたことをROS側に通知
        if ros_publisher is not None:
            try:
                # 通知用の関数を呼び出し（非ブロッキング）
                ros_publisher._notify_new_data()
            except Exception as e:
                # コールバック内でのエラーはログ出力のみ
                pass
        
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            data = []
            if self.new_stream and self.last_audio_input:
                chunk_time = STREAMING_LIMIT / len(self.last_audio_input)
                if chunk_time != 0:
                    if self.bridging_offset < 0:
                        self.bridging_offset = 0
                    if self.bridging_offset > self.final_request_end_time:
                        self.bridging_offset = self.final_request_end_time
                    chunks_from_ms = round((self.final_request_end_time -
                                            self.bridging_offset) / chunk_time)
                    self.bridging_offset = (round((len(self.last_audio_input) - chunks_from_ms) * chunk_time))
                    for i in range(chunks_from_ms, len(self.last_audio_input)):
                        data.append(self.last_audio_input[i])
                self.new_stream = False

            chunk = self._buff.get()
            self.audio_input.append(chunk)

            if chunk is None:
                return
            data.append(chunk)

            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                    self.audio_input.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)

def main():
    rate = 16000
    chunk_size = 160  # 10ms @ 16kHz
    device = 0
    speech_input = SpeechInput(rate, chunk_size, device)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
