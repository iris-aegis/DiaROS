#!/usr/bin/env python3
"""
SileroVAD 高速判定テスト - 10ms間隔で100ms無声区間検出
10msずらしで10ms毎に直近100msの音声/無声を判定
"""

import torch
import numpy as np
import pyaudio
import time
import threading
import queue
from datetime import datetime
import signal
import sys
from collections import deque

class CircularAudioBuffer:
    """100ms分の音声データを保持するサーキュラーバッファ"""
    def __init__(self, sample_rate=16000, buffer_duration_ms=100):
        self.sample_rate = sample_rate
        self.buffer_size = int(sample_rate * buffer_duration_ms / 1000)  # 100ms = 1600サンプル
        self.buffer = deque(maxlen=self.buffer_size)
        self.lock = threading.Lock()
    
    def add_samples(self, samples):
        """新しいサンプルを追加"""
        with self.lock:
            self.buffer.extend(samples)
    
    def get_last_100ms(self):
        """直近100ms分のデータを取得"""
        with self.lock:
            if len(self.buffer) < self.buffer_size:
                # バッファが満たない場合はゼロパディング
                padded = np.zeros(self.buffer_size, dtype=np.float32)
                if len(self.buffer) > 0:
                    padded[-len(self.buffer):] = list(self.buffer)
                return padded
            else:
                return np.array(list(self.buffer), dtype=np.float32)
    
    def is_ready(self):
        """100ms分のデータが蓄積されているか"""
        with self.lock:
            return len(self.buffer) >= self.buffer_size

class SileroVADFastTest:
    def __init__(self):
        # 音声設定（10ms間隔処理用）
        self.sample_rate = 16000
        self.chunk_size = 160  # 10ms分（16000Hz × 0.01秒）
        self.channels = 1
        self.format = pyaudio.paFloat32
        
        # PyAudio設定
        self.audio = pyaudio.PyAudio()
        self.stream = None
        
        # SileroVAD設定
        self.model = None
        self.get_speech_timestamps = None
        self.collect_chunks = None
        
        # 音声バッファ（100ms分を保持）
        self.audio_buffer = CircularAudioBuffer(self.sample_rate, 100)
        
        # 制御フラグ
        self.running = False
        self.audio_queue = queue.Queue()
        
        # VAD状態
        self.speech_detected = False
        self.last_silence_detection = None
        self.silence_100ms_detected = False
        
        # 統計
        self.total_judgments = 0
        self.speech_judgments = 0
        self.start_time = None
        
        # タイミング測定
        self.last_judgment_time = None
        self.judgment_intervals = deque(maxlen=100)  # 直近100回の判定間隔
        
    def load_silero_vad(self):
        """SileroVADモデルをロード"""
        try:
            print("[INFO] SileroVADモデルをロード中...")
            torch.set_num_threads(1)
            
            self.model, utils = torch.hub.load(
                repo_or_dir='snakers4/silero-vad',
                model='silero_vad',
                force_reload=False,
                onnx=False
            )
            
            self.get_speech_timestamps, save_audio, read_audio, VADIterator, self.collect_chunks = utils
            
            self.model.eval()
            print("[INFO] SileroVADモデルのロードが完了しました")
            return True
            
        except Exception as e:
            print(f"[ERROR] SileroVADロードに失敗: {e}")
            return False
    
    def audio_callback(self, in_data, frame_count, time_info, status):
        """PyAudioコールバック関数（10msチャンク）"""
        if status:
            print(f"[WARNING] Audio status: {status}")
        
        # numpy配列に変換
        audio_data = np.frombuffer(in_data, dtype=np.float32)
        
        # キューに追加
        try:
            self.audio_queue.put_nowait(audio_data)
        except queue.Full:
            print("[WARNING] Audio queue full, dropping frame")
        
        return (None, pyaudio.paContinue)
    
    def start_audio_stream(self):
        """音声ストリームを開始（10msチャンク）"""
        try:
            # 利用可能なデバイスを表示
            print("\n[INFO] 利用可能な音声デバイス:")
            for i in range(self.audio.get_device_count()):
                device_info = self.audio.get_device_info_by_index(i)
                if device_info['maxInputChannels'] > 0:
                    print(f"  デバイス {i}: {device_info['name']} (入力チャンネル: {device_info['maxInputChannels']})")
            
            # デフォルト入力デバイスを使用
            default_device = self.audio.get_default_input_device_info()
            print(f"\n[INFO] 使用デバイス: {default_device['name']}")
            
            # ストリーム開始（10msチャンク）
            self.stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size,
                stream_callback=self.audio_callback,
                start=False
            )
            
            self.stream.start_stream()
            print(f"[INFO] 音声ストリーム開始 (チャンクサイズ: {self.chunk_size}サンプル = {self.chunk_size/self.sample_rate*1000:.1f}ms)")
            return True
            
        except Exception as e:
            print(f"[ERROR] 音声ストリーム開始に失敗: {e}")
            return False
    
    def stop_audio_stream(self):
        """音声ストリームを停止"""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio:
            self.audio.terminate()
        print("[INFO] 音声ストリーム停止")
    
    def judge_last_100ms_silence(self):
        """直近100msが無声かどうかを判定（連続3チャンクを一括処理）"""
        if not self.audio_buffer.is_ready():
            return None, 0.0
        
        try:
            # 直近100ms分のデータを取得（1600サンプル）
            audio_100ms = self.audio_buffer.get_last_100ms()
            
            # 方法1: get_speech_timestampsを使用した一括処理（推奨）
            try:
                speech_timestamps = self.get_speech_timestamps(
                    audio_100ms,
                    self.model,
                    sampling_rate=self.sample_rate,
                    threshold=0.5,
                    min_speech_duration_ms=10,  # 短い音声も検出
                    min_silence_duration_ms=10, # 短い無声も検出
                    window_size_samples=512,    # 32msウィンドウ
                    speech_pad_ms=0,           # パディングなし
                    return_seconds=False       # サンプル単位で取得
                )
                
                # 音声区間が検出されたかで判定
                total_speech_samples = sum(segment['end'] - segment['start'] for segment in speech_timestamps)
                speech_ratio = total_speech_samples / len(audio_100ms)
                
                # 音声活動率で判定（50%以上で音声と判定）
                is_speech = speech_ratio >= 0.5
                final_prob = speech_ratio  # 0.0-1.0の範囲
                
                # デバッグ情報
                if len(speech_timestamps) > 0 and (speech_ratio > 0.8 or speech_ratio < 0.2):
                    timestamp_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    print(f"[{timestamp_str}][BULK_VAD] 音声区間: {len(speech_timestamps)}, 音声率: {speech_ratio:.3f}")
                
                return is_speech, final_prob
                
            except Exception as bulk_error:
                print(f"[WARNING] 一括処理失敗、個別処理にフォールバック: {bulk_error}")
                # フォールバック: 個別チャンク処理
                return self._judge_individual_chunks(audio_100ms)
            
        except Exception as e:
            print(f"[ERROR] 100ms判定エラー: {e}")
            return None, 0.0
    
    def _judge_individual_chunks(self, audio_100ms):
        """個別チャンク処理（フォールバック用）+ collect_chunks集約"""
        try:
            # 1600サンプルを512サンプルチャンクに分割
            chunk_size_512 = 512
            chunks = []
            speech_probs = []
            
            # 連続する3つの512サンプルチャンク（重複なし）
            for i in range(3):
                start_pos = i * chunk_size_512
                end_pos = start_pos + chunk_size_512
                if end_pos <= len(audio_100ms):
                    chunk = audio_100ms[start_pos:end_pos]
                    chunks.append(chunk)
                    
                    # 個別チャンクの確率を計算
                    with torch.no_grad():
                        chunk_tensor = torch.FloatTensor(chunk).unsqueeze(0)
                        speech_prob = self.model(chunk_tensor, self.sample_rate).item()
                        speech_probs.append(speech_prob)
            
            if not speech_probs:
                return None, 0.0
            
            # SileroVADのcollect_chunks機能を使用（利用可能な場合）
            try:
                if self.collect_chunks and len(chunks) == 3:
                    # 各チャンクの音声/無声判定結果
                    chunk_results = []
                    for i, prob in enumerate(speech_probs):
                        is_speech_chunk = prob >= 0.5
                        if is_speech_chunk:
                            chunk_results.append({
                                'start': i * chunk_size_512,
                                'end': (i + 1) * chunk_size_512,
                                'confidence': prob
                            })
                    
                    # collect_chunksで集約（詳細は実装に依存）
                    # ここでは手動集約を実装
                    speech_sample_count = sum(result['end'] - result['start'] for result in chunk_results)
                    total_samples = 3 * chunk_size_512
                    speech_ratio = speech_sample_count / total_samples
                    
                    is_speech = speech_ratio >= 0.5
                    final_prob = speech_ratio
                    
                    # デバッグ情報
                    if speech_ratio > 0.8 or speech_ratio < 0.2:
                        timestamp_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                        prob_str = f"[{speech_probs[0]:.3f}, {speech_probs[1]:.3f}, {speech_probs[2]:.3f}]"
                        print(f"[{timestamp_str}][COLLECT_CHUNKS] 個別確率: {prob_str}, 集約率: {speech_ratio:.3f}")
                    
                    return is_speech, final_prob
                    
            except Exception as collect_error:
                print(f"[WARNING] collect_chunks失敗: {collect_error}")
            
            # 従来の平均値による判定（最終フォールバック）
            avg_speech_prob = np.mean(speech_probs)
            is_speech = avg_speech_prob >= 0.5
            
            return is_speech, avg_speech_prob
            
        except Exception as e:
            print(f"[ERROR] 個別チャンク処理エラー: {e}")
            return None, 0.0
    
    def audio_processing_thread(self):
        """音声処理スレッド（10ms間隔処理）"""
        print("[INFO] 音声処理スレッド開始 (10ms間隔で100ms無声判定)")
        
        judgment_count = 0
        
        while self.running:
            try:
                # キューから10ms音声データを取得
                audio_chunk = self.audio_queue.get(timeout=0.1)
                
                # チャンクサイズの確認とパディング
                if len(audio_chunk) != self.chunk_size:
                    if len(audio_chunk) < self.chunk_size:
                        audio_chunk = np.pad(audio_chunk, (0, self.chunk_size - len(audio_chunk)))
                    else:
                        audio_chunk = audio_chunk[:self.chunk_size]
                
                # サーキュラーバッファに追加
                self.audio_buffer.add_samples(audio_chunk)
                
                # 100ms分のデータが蓄積されたら判定開始
                if self.audio_buffer.is_ready():
                    current_time = datetime.now()
                    
                    # タイミング測定
                    if self.last_judgment_time:
                        interval_ms = (current_time - self.last_judgment_time).total_seconds() * 1000
                        self.judgment_intervals.append(interval_ms)
                    self.last_judgment_time = current_time
                    
                    # 直近100msの音声/無声判定
                    is_speech, speech_prob = self.judge_last_100ms_silence()
                    
                    if is_speech is not None:
                        judgment_count += 1
                        self.total_judgments += 1
                        
                        if is_speech:
                            self.speech_judgments += 1
                        
                        timestamp_str = current_time.strftime('%H:%M:%S.%f')[:-3]
                        
                        # 状態変化検出
                        state_changed = False
                        if is_speech and not self.speech_detected:
                            # 音声開始
                            self.speech_detected = True
                            self.last_silence_detection = None
                            self.silence_100ms_detected = False
                            print(f"[{timestamp_str}][FAST_VAD] 音声開始検出 (prob={speech_prob:.3f})")
                            state_changed = True
                            
                        elif not is_speech and self.speech_detected:
                            # 音声終了→無声開始
                            self.speech_detected = False
                            self.last_silence_detection = current_time
                            print(f"[{timestamp_str}][FAST_VAD] 音声終了、無声区間開始 (prob={speech_prob:.3f})")
                            state_changed = True
                        
                        # 100ms無声継続チェック
                        if not is_speech and self.last_silence_detection and not self.silence_100ms_detected:
                            # 既に無声状態で、100ms判定も無声の場合
                            detection_latency_ms = (current_time - self.last_silence_detection).total_seconds() * 1000
                            if detection_latency_ms >= 0:  # 即座に100ms無声検出として報告
                                print(f"[{timestamp_str}][FAST_VAD] 100ms無声区間検出完了 (検出遅延: {detection_latency_ms:.1f}ms)")
                                self.silence_100ms_detected = True
                                
                                # TurnTaking実行タイミング
                                print(f"[{timestamp_str}][TURN_TAKING] TT推論実行タイミング! (高速検出)")
                        
                        # 定期的な統計表示（5秒ごと）
                        if judgment_count % 500 == 0:  # 500回判定ごと（約5秒）
                            elapsed = time.time() - self.start_time if self.start_time else 0
                            speech_ratio = (self.speech_judgments / self.total_judgments) * 100 if self.total_judgments > 0 else 0
                            avg_interval = np.mean(self.judgment_intervals) if self.judgment_intervals else 0
                            print(f"[{timestamp_str}][STATS] 経過: {elapsed:.1f}s, 判定回数: {self.total_judgments}, 音声率: {speech_ratio:.1f}%, 平均間隔: {avg_interval:.1f}ms")
                        
                        # 詳細ログ（状態変化時または極端な確率値）
                        if state_changed or speech_prob > 0.9 or speech_prob < 0.1:
                            avg_interval = np.mean(self.judgment_intervals) if self.judgment_intervals else 0
                            print(f"[{timestamp_str}][DETAIL] prob={speech_prob:.3f}, 判定={'音声' if is_speech else '無声'}, 判定間隔={avg_interval:.1f}ms")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[ERROR] 音声処理スレッドエラー: {e}")
                continue
        
        print("[INFO] 音声処理スレッド終了")
    
    def signal_handler(self, signum, frame):
        """シグナルハンドラ（Ctrl+C処理）"""
        print(f"\n[INFO] 終了シグナル受信 (signal: {signum})")
        self.stop()
    
    def start(self):
        """テスト開始"""
        print("SileroVAD 高速判定テスト (10ms間隔で100ms無声判定)")
        print("=" * 70)
        
        # シグナルハンドラ設定
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # SileroVADロード
        if not self.load_silero_vad():
            return False
        
        # 音声ストリーム開始
        if not self.start_audio_stream():
            return False
        
        # 音声処理スレッド開始
        self.running = True
        self.start_time = time.time()
        processing_thread = threading.Thread(target=self.audio_processing_thread)
        processing_thread.daemon = True
        processing_thread.start()
        
        print("\n[INFO] テスト開始! 話しかけてみてください...")
        print("[INFO] 10ms間隔で直近100msの音声/無声を判定します")
        print("[INFO] 終了するには Ctrl+C を押してください")
        print("=" * 70)
        
        # メインループ
        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        
        return True
    
    def stop(self):
        """テスト停止"""
        print("\n[INFO] テスト停止中...")
        self.running = False
        
        # 音声ストリーム停止
        self.stop_audio_stream()
        
        # 統計表示
        if self.total_judgments > 0:
            elapsed = time.time() - self.start_time if self.start_time else 0
            speech_ratio = (self.speech_judgments / self.total_judgments) * 100
            avg_interval = np.mean(self.judgment_intervals) if self.judgment_intervals else 0
            judgment_rate = self.total_judgments / elapsed if elapsed > 0 else 0
            
            print("\n" + "=" * 70)
            print("テスト結果:")
            print(f"  実行時間: {elapsed:.1f}秒")
            print(f"  判定回数: {self.total_judgments}")
            print(f"  音声判定回数: {self.speech_judgments}")
            print(f"  音声活動率: {speech_ratio:.1f}%")
            print(f"  平均判定間隔: {avg_interval:.1f}ms")
            print(f"  判定レート: {judgment_rate:.1f} judgments/sec")
            print(f"  理論値(10ms間隔): 100 judgments/sec")
            print("=" * 70)

def main():
    """メイン関数"""
    test = SileroVADFastTest()
    
    try:
        test.start()
    except Exception as e:
        print(f"[ERROR] テスト実行エラー: {e}")
    finally:
        test.stop()

if __name__ == "__main__":
    main()