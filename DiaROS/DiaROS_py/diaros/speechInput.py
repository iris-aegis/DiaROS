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
        import time
        from datetime import datetime
        import numpy as np
        import hashlib
        
        # 音声データを浮動小数点配列に変換
        audio_np = np.frombuffer(in_data, dtype=np.float32)
        
        # 一意の音声IDを生成（先頭10サンプル + タイムスタンプのハッシュ）
        capture_timestamp = time.time()
        if len(audio_np) >= 10:
            sample_data = audio_np[:10].tobytes()
            timestamp_bytes = str(capture_timestamp).encode()
            audio_id = hashlib.md5(sample_data + timestamp_bytes).hexdigest()[:8]
        else:
            audio_id = hashlib.md5(str(capture_timestamp).encode()).hexdigest()[:8]
        
        # SDS音声取得タイムスタンプをログ出力（毎回）
        timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # データ対応確認用: 先頭5サンプルの具体的な値を表示
        sample_values = audio_np[:5].tolist() if len(audio_np) >= 5 else audio_np.tolist()
        sample_str = '[' + ','.join([f'{v:.6f}' for v in sample_values]) + ']'
        
        sys.stdout.write(f"[🎙️ MIC_CAPTURE] {timestamp_str} | {capture_timestamp:.6f} | {sample_str}\n")
        sys.stdout.flush()
        
        # 音声データにIDとタイムスタンプを付与してキューに追加
        audio_data_with_metadata = {
            'audio_data': in_data,
            'audio_id': audio_id,
            'capture_timestamp': capture_timestamp,
            'sample_count': len(audio_np)
        }
        
        # キューにメタデータ付きデータを追加
        stream_queue.put(audio_data_with_metadata)
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
