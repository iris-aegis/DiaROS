### backChannel.py ###
"""
仕様
- リアルタイム音声入力に対して相槌予測を実行
- 相槌音声を再生する機能はDialogManagementに移管済み
- 相槌後のクールダウンもDialogManagementに移管済み
- リアルタイムに相槌確率と処理時間（ms）を表示
"""

import pyaudio
import numpy as np
import torch
import wave
import time
from threading import Thread, Lock, Event
from queue import Queue
import sys
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor
import pygame
pygame.mixer.init()

# ROSから音声チャンクを受け取るためのグローバルキュー
stream_queue = Queue()
back_channel_result_queue = Queue()
THRESHOLD = 0.60

def push_audio_data(data):
    stream_queue.put(data)
    # 追加: キューサイズを表示
    # sys.stdout.write(f"[backChannel.py] push_audio_data: stream_queue size={stream_queue.qsize()}\n")
    sys.stdout.flush()

def get_audio_data():
    if not stream_queue.empty():
        data = stream_queue.get()
        # sys.stdout.write(f"[backChannel.py] get_audio_data: stream_queue size={stream_queue.qsize()}\n")
        sys.stdout.flush()
        return data
    # sys.stdout.write(f"[backChannel.py] get_audio_data: stream_queue EMPTY\n")
    sys.stdout.flush()
    return None

# 新たにグローバル変数として直近の送信済みフラグを追加
last_sent_time = 0

class RealtimeAizuchiPredictor:
    def __init__(self, model_id="SiRoZaRuPa/japanese-wav2vec2-base-backchannel-CSJ", sample_rate=16000, chunk_duration_ms=1000):
        self.sample_rate = sample_rate
        self.chunk_duration_ms = chunk_duration_ms
        self.chunk_samples = int(sample_rate * chunk_duration_ms / 1000)
        self.inference_interval_ms = 100  # 100msごとに推論
        self.inference_interval_samples = int(sample_rate * self.inference_interval_ms / 1000)
        self.samples_since_last_inference = 0
        self.buffer_lock = Lock()
        self.audio_buffer = np.zeros(0, dtype=np.float32)
        self.max_buffer_samples = self.chunk_samples * 2  # バッファ肥大化防止
        self.prediction_queue = Queue()
        self.audio_level_queue = Queue()
        self.processing_time_queue = Queue()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        sys.stdout.write(f"使用デバイス: {self.device}\n")
        sys.stdout.flush()
        
        try:
            # 学習時と同じ構成でモデルをロード
            self.model = Wav2Vec2ForSequenceClassification.from_pretrained(
                model_id, output_hidden_states=True
            ).to(self.device)
            self.model.eval()
            self.feature_extractor = Wav2Vec2FeatureExtractor.from_pretrained(
                model_id
            )
            sys.stdout.write("wav2vec2相槌予測モデルの読み込みが完了しました\n")
            sys.stdout.flush()
        except Exception as e:
            sys.stdout.write(f"wav2vec2相槌予測モデルの読み込みに失敗しました: {e}\n")
            sys.stdout.flush()
            raise

    def process_chunk(self, new_chunk):
        self.audio_buffer = np.concatenate([self.audio_buffer, new_chunk])
        self.samples_since_last_inference += len(new_chunk)
        if len(self.audio_buffer) > self.max_buffer_samples:
            self.audio_buffer = self.audio_buffer[-self.max_buffer_samples:]
        result = None
        if self.audio_buffer.shape[0] >= self.chunk_samples and self.samples_since_last_inference >= self.inference_interval_samples:
            start_time = time.time()
            audio_chunk = self.audio_buffer[-self.chunk_samples:]
            try:
                # 学習時と同じくfeature_extractorで前処理
                inputs = self.feature_extractor(
                    audio_chunk, sampling_rate=self.sample_rate, return_tensors="pt"
                )
                inputs = {k: v.to(self.device) for k, v in inputs.items()}
                with torch.no_grad():
                    output = self.model(**inputs).logits
                # 学習時と同じくシグモイドで確率化
                probability = torch.sigmoid(output).item()
                end_time = time.time()
                processing_time = (end_time - start_time) * 1000
                result = (probability, processing_time)
            except Exception as e:
                sys.stdout.write(f"処理中にエラーが発生しました: {e}\n")
                import traceback
                sys.stdout.write(traceback.format_exc())
                sys.stdout.flush()
                result = None
            self.samples_since_last_inference = 0
        return result

    def get_audio_level(self):
        if not self.audio_level_queue.empty():
            return self.audio_level_queue.get()
        return None

    def stop(self):
        # スレッド管理不要
        pass

def create_level_bar(level, width=40, threshold=0.01):
    normalized_level = min(1.0, level / threshold)
    bar_length = int(normalized_level * width)
    return '[' + '#' * bar_length + '-' * (width - bar_length) + ']'

def move_cursor_up(n):
    sys.stdout.write(f'\033[{n}A')
    sys.stdout.flush()

def clear_line():
    sys.stdout.write('\033[K')
    sys.stdout.flush()

class AudioPlayer:
    def __init__(self, audio_file):
        self.audio_file = audio_file
        self.is_playing = Event()
        
    def play_audio(self):
        if self.is_playing.is_set():
            return
            
        try:
            wf = wave.open(self.audio_file, 'rb')
            p = pyaudio.PyAudio()
            
            def callback(in_data, frame_count, time_info, status):
                data = wf.readframes(frame_count)
                return (data, pyaudio.paContinue)
            
            stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                          channels=wf.getnchannels(),
                          rate=wf.getframerate(),
                          output=True,
                          stream_callback=callback)
            
            self.is_playing.set()
            stream.start_stream()
            
            while stream.is_active():
                time.sleep(0.1)
            
            stream.stop_stream()
            stream.close()
            wf.close()
            p.terminate()
            self.is_playing.clear()
            
        except Exception as e:
            sys.stdout.write(f"音声再生中にエラーが発生しました: {e}\n")
            sys.stdout.flush()
            self.is_playing.clear()

    def play_async(self):
        Thread(target=self.play_audio).start()

def get_default_device_index():
    p = pyaudio.PyAudio()
    default_index = None
    for i in range(p.get_device_count()):
        dev = p.get_device_info_by_index(i)
        if dev['maxInputChannels'] > 0 and dev['name'] == "default":
            default_index = i
            sys.stdout.write(f"デフォルトデバイスインデックス: {default_index}\n")
            sys.stdout.flush()
            break
    p.terminate()
    if default_index is None:
        raise RuntimeError('デバイス名が"default"のマイクが見つかりません')
    return default_index

def main():
    try:
        sys.stdout.write("\n相槌予測システムを初期化中...\n")
        sys.stdout.flush()
        predictor = RealtimeAizuchiPredictor()
        
        # last_probability = None
        # waiting_for_change = False
        
        sys.stdout.write("相槌予測システムを開始します。Ctrl+Cで終了します。\n")
        sys.stdout.write(f"閾値: {THRESHOLD:.2f}\n")
        sys.stdout.write("--------------------\n")
        sys.stdout.flush()
        
        BAR_MEM = 20  # バーの長さ
        YELLOW = "\033[33m"
        RESET = "\033[0m"
        while True:
            result = None
            if not stream_queue.empty():
                new_chunk = stream_queue.get()
                result = predictor.process_chunk(new_chunk)
            if result is not None:
                probability, processing_time = result
                # 相槌確率が変化するまで次の相槌判定を停止する機能を削除
                # bar_len = int(round(probability * BAR_MEM))
                # threshold_pos = int(round(THRESHOLD * BAR_MEM))
                # bar = ""
                # for i in range(BAR_MEM):
                #     if i == threshold_pos:
                #         bar += "|"
                #     elif i < bar_len:
                #         bar += "■"
                #     else:
                #         bar += " "
                # now_str = time.strftime("%H:%M:%S", time.localtime(time.time())) + f".{int((time.time()*1000)%1000):03d}"
                # sys.stdout.write(f"|{bar}| {probability:.10f} [処理時間: {processing_time:.1f}ms] [{now_str}]\n")
                # sys.stdout.flush()
                result_flag = 1 if probability > THRESHOLD else 0
                # すべての推論結果をキューにput
                back_channel_result_queue.put((result_flag, probability))
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        sys.stdout.write("\n処理を終了します...\n")
        sys.stdout.flush()
        predictor.stop()
    except Exception as e:
        sys.stdout.write(f"エラーが発生しました: {e}\n")
        sys.stdout.flush()