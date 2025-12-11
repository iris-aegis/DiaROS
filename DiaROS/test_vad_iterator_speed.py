#!/usr/bin/env python3
"""
VADIteratorの処理速度検証用スクリプト。
指定されたWAVファイルに対してVADIteratorを実行し、処理時間を計測する。
"""

import torch
import numpy as np
import time
import sys
import torchaudio # read_audioユーティリティのために必要

class VADSpeedTest:
    def __init__(self, wav_path):
        # 音声設定
        self.sample_rate = 16000
        self.chunk_size = 512  # 32ms
        self.wav_path = wav_path
        
        # SileroVAD設定
        self.model = None
        self.vad_iterator = None
        self.read_audio_util = None # Silero VADのread_audioユーティリティ

    def load_silero_vad(self):
        """SileroVADモデルとVADIteratorをロード・初期化"""
        try:
            print("[INFO] SileroVADモデルをロード中...")
            torch.set_num_threads(1)
            
            model, utils = torch.hub.load(
                repo_or_dir='snakers4/silero-vad',
                model='silero_vad',
                force_reload=False,
                onnx=False
            )
            
            # utilsからVADIteratorとread_audioを取得
            self.read_audio_util = utils[2]
            VADIterator = utils[3]
            self.vad_iterator = VADIterator(model, sampling_rate=self.sample_rate)
            print("[INFO] SileroVADモデルのロードが完了しました。")
            return True
            
        except Exception as e:
            print(f"[ERROR] SileroVADロードに失敗: {e}", file=sys.stderr)
            return False

    def load_audio_from_file(self):
        """指定されたWAVファイルから音声を読み込む"""
        print("-" * 60)
        print(f"[INFO] 音声ファイル '{self.wav_path}' を読み込み中...")
        try:
            # Silero VADのread_audioユーティリティを使用
            audio_tensor = self.read_audio_util(self.wav_path, sampling_rate=self.sample_rate)
            print("[INFO] 音声ファイルの読み込みが完了しました。")
            return audio_tensor
        except Exception as e:
            print(f"[ERROR] 音声ファイルの読み込みに失敗: {e}", file=sys.stderr)
            return None

    def run_vad_on_data(self, audio_tensor):
        """音声データ全体に対してVAD処理を行い、時間を計測する"""
        print("-" * 60)
        print("[INFO] 読み込んだ音声データに対してVAD処理の時間を計測します...")
        
        self.vad_iterator.reset_states()
        
        # 計測開始
        start_time = time.perf_counter()
        
        num_samples = len(audio_tensor)
        for i in range(0, num_samples, self.chunk_size):
            chunk = audio_tensor[i: i + self.chunk_size]
            if len(chunk) < self.chunk_size:
                # 最後のチャンクが短ければスキップ
                continue
            self.vad_iterator(chunk) # イベント出力は無視して処理だけ行う

        # 計測終了
        end_time = time.perf_counter()
        
        processing_time_ms = (end_time - start_time) * 1000
        return processing_time_ms

    def start(self):
        """検証処理を開始"""
        if not self.load_silero_vad():
            return

        audio_data = self.load_audio_from_file()
        if audio_data is None:
            return

        audio_duration_seconds = len(audio_data) / self.sample_rate
        
        processing_time = self.run_vad_on_data(audio_data)
        
        print("-" * 60)
        print("[結果]")
        print(f"  音声ファイル長: {audio_duration_seconds:.2f}秒")
        print(f"  VAD処理時間: {processing_time:.4f} ミリ秒")
        print("-" * 60)
        
        target_ms = 10
        if processing_time < target_ms:
            print(f"結論: 処理は目標の{target_ms}ミリ秒以内に完了しました。")
        else:
            print(f"結論: 処理は目標の{target_ms}ミリ秒を超過しました。")
            print("      この処理を10msごとに行うのは現実的ではありません。")
        print("-" * 60)

def main():
    """メイン関数"""
    wav_file_path = "/workspace/DiaROS/他になにかいい案無いかな.wav"
    test = VADSpeedTest(wav_path=wav_file_path)
    test.start()

if __name__ == "__main__":
    main()