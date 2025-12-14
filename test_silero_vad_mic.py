#!/usr/bin/env python3
"""
SileroVAD マイク入力リアルタイムテスト
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

class SileroVADMicTest:
    def __init__(self):
        # 音声設定
        self.sample_rate = 16000
        self.chunk_size = 512  # 32ms分
        self.channels = 1
        self.format = pyaudio.paFloat32
        
        # PyAudio設定
        self.audio = pyaudio.PyAudio()
        self.stream = None
        
        # SileroVAD設定
        self.model = None
        self.vad_iterator = None
        self.get_speech_timestamps = None
        
        # 制御フラグ
        self.running = False
        self.audio_queue = queue.Queue()
        
        # VAD状態
        self.speech_detected = False
        self.silent_start_time = None
        self.silence_100ms_detected = False
        
        # 統計
        self.total_chunks = 0
        self.speech_chunks = 0
        self.start_time = None
        
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
            
            self.get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
            
            # VADIteratorの初期化
            self.vad_iterator = VADIterator(
                self.model,
                threshold=0.5,
                sampling_rate=self.sample_rate,
                min_silence_duration_ms=100,
                speech_pad_ms=30
            )
            
            self.model.eval()
            print("[INFO] SileroVADモデルのロードが完了しました")
            return True
            
        except Exception as e:
            print(f"[ERROR] SileroVADロードに失敗: {e}")
            return False
    
    def audio_callback(self, in_data, frame_count, time_info, status):
        """PyAudioコールバック関数"""
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
        """音声ストリームを開始"""
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
            
            # ストリーム開始
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
            print(f"[INFO] 音声ストリーム開始 (サンプリングレート: {self.sample_rate}Hz, チャンクサイズ: {self.chunk_size})")
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
    
    def process_audio_direct_model(self, audio_chunk):
        """モデル直接呼び出しで音声処理"""
        try:
            with torch.no_grad():
                chunk_tensor = torch.FloatTensor(audio_chunk).unsqueeze(0)
                speech_prob = self.model(chunk_tensor, self.sample_rate).item()
            
            current_time = datetime.now()
            timestamp_str = current_time.strftime('%H:%M:%S.%f')[:-3]
            
            # 統計更新
            self.total_chunks += 1
            is_speech = speech_prob >= 0.5
            if is_speech:
                self.speech_chunks += 1
            
            # 状態変化検出
            state_changed = False
            if is_speech and not self.speech_detected:
                # 音声開始
                self.speech_detected = True
                self.silent_start_time = None
                self.silence_100ms_detected = False
                print(f"[{timestamp_str}][SILERO_VAD] 音声開始検出 (prob={speech_prob:.3f})")
                state_changed = True
                
            elif not is_speech and self.speech_detected:
                # 音声終了→無声開始
                self.speech_detected = False
                self.silent_start_time = current_time
                print(f"[{timestamp_str}][SILERO_VAD] 音声終了、無声区間開始 (prob={speech_prob:.3f})")
                state_changed = True
            
            # 100ms無声継続チェック
            if self.silent_start_time and not self.silence_100ms_detected:
                silent_duration_ms = (current_time - self.silent_start_time).total_seconds() * 1000
                if silent_duration_ms >= 100:
                    start_timestamp = self.silent_start_time.strftime('%H:%M:%S.%f')[:-3]
                    print(f"[{start_timestamp}][SILERO_VAD] 100ms無声区間検出 ({silent_duration_ms:.0f}ms継続)")
                    self.silence_100ms_detected = True
                    
                    # ここでTurnTakingモデルを実行する場合の処理
                    print(f"[{timestamp_str}][TURN_TAKING] TT推論実行タイミング!")
            
            # 定期的な状態表示（5秒ごと）
            if self.total_chunks % (5 * self.sample_rate // self.chunk_size) == 0:
                elapsed = time.time() - self.start_time
                speech_ratio = (self.speech_chunks / self.total_chunks) * 100
                print(f"[{timestamp_str}][STATS] 経過時間: {elapsed:.1f}s, 音声率: {speech_ratio:.1f}%, 現在: {'音声' if is_speech else '無声'} (prob={speech_prob:.3f})")
            
            # 詳細ログ（状態変化時のみ）
            if state_changed or speech_prob > 0.8 or (not is_speech and speech_prob < 0.2):
                print(f"[{timestamp_str}][DETAIL] prob={speech_prob:.3f}, 判定={'音声' if is_speech else '無声'}")
            
        except Exception as e:
            print(f"[ERROR] 音声処理エラー: {e}")
    
    def process_audio_vad_iterator(self, audio_chunk):
        """VADIteratorで音声処理"""
        try:
            result = self.vad_iterator(audio_chunk)
            
            if result is not None:
                current_time = datetime.now()
                timestamp_str = current_time.strftime('%H:%M:%S.%f')[:-3]
                
                print(f"[{timestamp_str}][VAD_ITERATOR] 結果: {result}")
                
                # 結果の形式を解析
                if isinstance(result, dict):
                    if 'start' in result:
                        sample_pos = result['start']
                        time_pos = sample_pos / self.sample_rate
                        print(f"[{timestamp_str}][VAD_ITERATOR] 音声開始検出 (位置: {time_pos:.3f}s)")
                    elif 'end' in result:
                        sample_pos = result['end']
                        time_pos = sample_pos / self.sample_rate
                        print(f"[{timestamp_str}][VAD_ITERATOR] 音声終了検出 (位置: {time_pos:.3f}s)")
                
        except Exception as e:
            print(f"[ERROR] VADIterator処理エラー: {e}")
    
    def audio_processing_thread(self):
        """音声処理スレッド"""
        print("[INFO] 音声処理スレッド開始")
        
        while self.running:
            try:
                # キューから音声データを取得（100msタイムアウト）
                audio_chunk = self.audio_queue.get(timeout=0.1)
                
                # チャンクサイズの確認
                if len(audio_chunk) != self.chunk_size:
                    print(f"[WARNING] 期待されるチャンクサイズ {self.chunk_size} != 受信サイズ {len(audio_chunk)}")
                    # パディングまたはトリミング
                    if len(audio_chunk) < self.chunk_size:
                        audio_chunk = np.pad(audio_chunk, (0, self.chunk_size - len(audio_chunk)))
                    else:
                        audio_chunk = audio_chunk[:self.chunk_size]
                
                # 両方の方式で処理
                self.process_audio_direct_model(audio_chunk)
                # self.process_audio_vad_iterator(audio_chunk)  # 必要に応じてコメントアウト解除
                
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
        print("SileroVAD マイク入力リアルタイムテスト")
        print("=" * 60)
        
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
        print("[INFO] 終了するには Ctrl+C を押してください")
        print("=" * 60)
        
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
        if self.total_chunks > 0:
            elapsed = time.time() - self.start_time if self.start_time else 0
            speech_ratio = (self.speech_chunks / self.total_chunks) * 100
            
            print("\n" + "=" * 60)
            print("テスト結果:")
            print(f"  実行時間: {elapsed:.1f}秒")
            print(f"  処理チャンク数: {self.total_chunks}")
            print(f"  音声チャンク数: {self.speech_chunks}")
            print(f"  音声活動率: {speech_ratio:.1f}%")
            print(f"  平均処理レート: {self.total_chunks/elapsed:.1f} chunks/sec")
            print("=" * 60)

def main():
    """メイン関数"""
    test = SileroVADMicTest()
    
    try:
        test.start()
    except Exception as e:
        print(f"[ERROR] テスト実行エラー: {e}")
    finally:
        test.stop()

if __name__ == "__main__":
    main()