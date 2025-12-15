#!/usr/bin/env python3
"""
SileroVAD WAVファイル入力テスト
VADIteratorのロジックを参考に、10msステップ/100msウィンドウで音声区間の開始・終了を判定する。
"""

import torch
import numpy as np
import time
from datetime import datetime
import sys

class SileroVADWavTest:
    def __init__(self, wav_path, threshold=0.5, neg_threshold=0.35, min_silence_duration_ms=100):
        # 音声設定
        self.sample_rate = 16000
        self.wav_path = wav_path
        
        # VADパラメータ
        self.threshold = threshold
        self.neg_threshold = neg_threshold
        self.min_silence_duration_ms = min_silence_duration_ms
        
        # SileroVAD設定
        self.model = None
        self.read_audio = None
        
        # 状態管理
        self.triggered = False
        self.temp_end_ms = 0
        
        # 統計
        self.speeches = []
        self.current_speech = {}

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
            
            _, _, self.read_audio, _, _ = utils
            
            self.model.eval()
            print("[INFO] SileroVADモデルのロードが完了しました")
            return True
            
        except Exception as e:
            print(f"[ERROR] SileroVADロードに失敗: {e}", file=sys.stderr)
            return False

    def process_state(self, speech_prob, current_time_ms):
        """VADIteratorのロジックを模倣して状態を更新し、イベントを出力"""
        
        # 音声開始の可能性
        if speech_prob >= self.threshold:
            if self.temp_end_ms > 0:
                # 一時的な無音区間は終了
                self.temp_end_ms = 0
            
            if not self.triggered:
                self.triggered = True
                self.current_speech['start'] = current_time_ms
                print(f"[{int(current_time_ms)}ms] --- 音声開始 --- (prob={speech_prob:.4f})")

        # 音声終了の可能性
        elif speech_prob < self.neg_threshold:
            if self.triggered:
                if self.temp_end_ms == 0:
                    # 無音区間の開始時刻を記録
                    self.temp_end_ms = current_time_ms
                
                # 無音区間が規定時間続いたかチェック
                if (current_time_ms - self.temp_end_ms) >= self.min_silence_duration_ms:
                    self.current_speech['end'] = self.temp_end_ms
                    self.speeches.append(self.current_speech)
                    print(f"[{int(self.temp_end_ms)}ms] --- 音声終了 --- (無音継続: {current_time_ms - self.temp_end_ms}ms)")
                    self.current_speech = {}
                    self.triggered = False
                    self.temp_end_ms = 0
        
        # 確率をデバッグ表示したい場合は以下のコメントを解除
        # print(f"[{int(current_time_ms)}ms] prob={speech_prob:.4f}, triggered={self.triggered}")


    def start(self):
        """テスト開始"""
        title = "SileroVAD WAVテスト (VADIterator風ロジック, 10msステップ, 100msウィンドウ)"
        print(title)
        print("=" * len(title))
        
        if not self.load_silero_vad():
            return False
        
        try:
            print(f"[INFO] 音声ファイル '{self.wav_path}' を読み込み中...")
            wav_tensor = self.read_audio(self.wav_path, sampling_rate=self.sample_rate)
            wav_data = wav_tensor.numpy()
            num_samples = len(wav_data)
            print(f"[INFO] 読み込み完了。データ長: {num_samples}サンプル, 約{num_samples/self.sample_rate:.2f}秒")
        except Exception as e:
            print(f"[ERROR] 音声ファイルの読み込みに失敗: {e}", file=sys.stderr)
            return False

        step_size_ms = 10
        window_size_ms = 100
        chunk_size_samples = 512 # 32ms

        step_size_samples = int(self.sample_rate * step_size_ms / 1000)
        window_size_samples = int(self.sample_rate * window_size_ms / 1000)

        processing_start_time = time.time()
        
        print("\n[INFO] 処理開始...")
        print("=" * 60)
        
        # 10msずつ基準点をずらしながら処理
        for current_pos in range(0, num_samples + 1, step_size_samples):
            window_start = max(0, current_pos - window_size_samples)
            window_end = current_pos
            
            audio_window = wav_data[window_start:window_end]
            
            # ウィンドウ内のチャンクの平均確率を計算
            probs = []
            for i in range(0, len(audio_window), chunk_size_samples):
                chunk = audio_window[i: i + chunk_size_samples]
                if len(chunk) < chunk_size_samples:
                    continue
                with torch.no_grad():
                    chunk_tensor = torch.FloatTensor(chunk)
                    prob = self.model(chunk_tensor, self.sample_rate).item()
                    probs.append(prob)
            
            speech_prob = np.mean(probs) if probs else 0.0
            
            # 状態を更新
            current_time_ms = (current_pos / self.sample_rate) * 1000
            self.process_state(speech_prob, current_time_ms)

        # ファイルの最後に音声が残っている場合
        if self.triggered:
            self.current_speech['end'] = (num_samples / self.sample_rate) * 1000
            self.speeches.append(self.current_speech)
            print(f"[{int(self.current_speech['end'])}ms] --- 音声終了 --- (ファイル終端)")

        processing_duration = time.time() - processing_start_time
        print("=" * 60)
        print("[INFO] 処理完了")
        
        # 統計表示
        print("\n" + "=" * 60)
        print("テスト結果: 検出された音声区間")
        if not self.speeches:
            print("  (音声区間は検出されませんでした)")
        else:
            for i, speech in enumerate(self.speeches):
                start_s = speech['start'] / 1000
                end_s = speech['end'] / 1000
                duration = end_s - start_s
                print(f"  区間 {i+1}: {speech['start']:.0f}ms - {speech['end']:.0f}ms (持続時間: {duration:.2f}秒)")
        
        print("\n" + "=" * 60)
        audio_duration = num_samples / self.sample_rate
        print(f"  音声ファイル長: {audio_duration:.2f}秒")
        print(f"  処理時間: {processing_duration:.2f}秒")
        if audio_duration > 0:
            print(f"  リアルタイム比: {processing_duration / audio_duration:.4f} (1.0未満ならリアルタイム処理可能)")
        print("=" * 60)
        
        return True

def main():
    """メイン関数"""
    wav_file = "/workspace/他になにかいい案無いかな.wav"
    
    if not wav_file:
        print("エラー: WAVファイルが指定されていません。", file=sys.stderr)
        sys.exit(1)
        
    # パラメータはここで調整可能
    test = SileroVADWavTest(wav_file, 
                            threshold=0.5, 
                            neg_threshold=0.35, 
                            min_silence_duration_ms=100)
    
    try:
        test.start()
    except Exception as e:
        print(f"[ERROR] テスト実行中に予期せぬエラーが発生: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()