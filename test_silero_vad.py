#!/usr/bin/env python3
"""
SileroVAD単体テスト用スクリプト
GitHubの公式例を参考にした実装: https://github.com/snakers4/silero-vad
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
import time
from datetime import datetime
import soundfile as sf

# SileroVADのロード
def load_silero_vad():
    """SileroVADモデルをロード"""
    try:
        # PyTorchスレッド数を制限（CPUパフォーマンス最適化）
        torch.set_num_threads(1)
        
        print("[INFO] SileroVADモデルをロード中...")
        model, utils = torch.hub.load(
            repo_or_dir='snakers4/silero-vad',
            model='silero_vad',
            force_reload=False,
            onnx=False
        )
        
        get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
        print("[INFO] SileroVADモデルのロードが完了しました")
        
        return model, get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks
    except Exception as e:
        print(f"[ERROR] SileroVADロードに失敗: {e}")
        return None, None, None, None, None, None

def test_with_audio_file(audio_file_path):
    """音声ファイルを使用したテスト"""
    print(f"\n=== 音声ファイルテスト: {audio_file_path} ===")
    
    # SileroVADをロード
    model, get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = load_silero_vad()
    if model is None:
        return
    
    try:
        # 音声ファイル読み込み
        print("[INFO] 音声ファイルを読み込み中...")
        wav = read_audio(audio_file_path)
        print(f"[INFO] 音声データ: {len(wav)}サンプル, サンプリングレート: 16kHz想定")
        
        # 音声区間検出
        print("[INFO] 音声区間検出中...")
        start_time = time.time()
        
        speech_timestamps = get_speech_timestamps(
            wav,
            model,
            sampling_rate=16000,
            threshold=0.5,
            min_speech_duration_ms=250,
            min_silence_duration_ms=100,
            window_size_samples=1024,
            speech_pad_ms=30,
            return_seconds=True
        )
        
        processing_time = (time.time() - start_time) * 1000
        print(f"[INFO] 処理時間: {processing_time:.1f}ms")
        
        # 結果表示
        print(f"\n[RESULT] 検出された音声区間数: {len(speech_timestamps)}")
        for i, segment in enumerate(speech_timestamps):
            start_sec = segment['start']
            end_sec = segment['end']
            duration_ms = (end_sec - start_sec) * 1000
            print(f"  区間 {i+1}: {start_sec:.3f}s - {end_sec:.3f}s (長さ: {duration_ms:.0f}ms)")
        
        # 音声活動率計算
        total_speech_duration = sum(segment['end'] - segment['start'] for segment in speech_timestamps)
        total_duration = len(wav) / 16000
        speech_ratio = (total_speech_duration / total_duration) * 100
        print(f"\n[STATS] 音声活動率: {speech_ratio:.1f}% ({total_speech_duration:.2f}s / {total_duration:.2f}s)")
        
        return speech_timestamps
        
    except Exception as e:
        print(f"[ERROR] 音声ファイル処理エラー: {e}")
        return None

def test_realtime_vad():
    """リアルタイムVADのテスト（シミュレーション）"""
    print(f"\n=== リアルタイムVADテスト ===")
    
    # SileroVADをロード
    model, get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = load_silero_vad()
    if model is None:
        return
    
    # VADIteratorの初期化
    vad_iterator = VADIterator(
        model,
        threshold=0.5,
        sampling_rate=16000,
        min_silence_duration_ms=100,
        speech_pad_ms=30
    )
    
    print("[INFO] VADIterator初期化完了")
    print("[INFO] 512サンプルチャンクでのリアルタイム処理をシミュレーション...")
    
    # テスト用の音声データ生成（ノイズ + 正弦波）
    sample_rate = 16000
    chunk_size = 512  # 32ms分
    total_duration = 3.0  # 3秒間
    total_samples = int(sample_rate * total_duration)
    
    # 音声パターン: 無音(0.5s) -> 音声(1s) -> 無音(1s) -> 音声(0.5s)
    test_audio = np.zeros(total_samples, dtype=np.float32)
    
    # 音声区間1: 0.5s-1.5s
    start1 = int(0.5 * sample_rate)
    end1 = int(1.5 * sample_rate)
    test_audio[start1:end1] = 0.3 * np.sin(2 * np.pi * 440 * np.linspace(0, 1, end1-start1))
    
    # 音声区間2: 2.5s-3.0s
    start2 = int(2.5 * sample_rate)
    end2 = int(3.0 * sample_rate)
    test_audio[start2:end2] = 0.2 * np.sin(2 * np.pi * 880 * np.linspace(0, 0.5, end2-start2))
    
    # ノイズ追加
    test_audio += np.random.normal(0, 0.01, total_samples)
    
    print("[INFO] テスト音声パターン:")
    print("  0.0s - 0.5s: 無音")
    print("  0.5s - 1.5s: 音声 (440Hz)")
    print("  1.5s - 2.5s: 無音")
    print("  2.5s - 3.0s: 音声 (880Hz)")
    
    # チャンク単位で処理
    chunks_processed = 0
    speech_detected = False
    
    for i in range(0, len(test_audio), chunk_size):
        chunk = test_audio[i:i+chunk_size]
        if len(chunk) < chunk_size:
            # パディング
            chunk = np.pad(chunk, (0, chunk_size - len(chunk)))
        
        current_time = i / sample_rate
        
        try:
            # VAD処理
            speech_dict = vad_iterator(chunk)
            
            if speech_dict is not None:
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                
                if isinstance(speech_dict, dict):
                    if speech_dict.get('type') == 'start':
                        print(f"[{timestamp}] {current_time:.3f}s: 音声開始検出")
                        speech_detected = True
                    elif speech_dict.get('type') == 'end':
                        print(f"[{timestamp}] {current_time:.3f}s: 音声終了検出")
                        speech_detected = False
                elif isinstance(speech_dict, list):
                    for event in speech_dict:
                        if isinstance(event, dict):
                            if event.get('type') == 'start':
                                print(f"[{timestamp}] {current_time:.3f}s: 音声開始検出")
                                speech_detected = True
                            elif event.get('type') == 'end':
                                print(f"[{timestamp}] {current_time:.3f}s: 音声終了検出")
                                speech_detected = False
            
            chunks_processed += 1
            
        except Exception as e:
            print(f"[ERROR] チャンク処理エラー (時刻: {current_time:.3f}s): {e}")
            continue
    
    print(f"\n[INFO] 処理完了: {chunks_processed}チャンク処理")

def test_direct_model():
    """モデル直接呼び出しテスト"""
    print(f"\n=== モデル直接呼び出しテスト ===")
    
    # SileroVADをロード
    model, get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = load_silero_vad()
    if model is None:
        return
    
    model.eval()
    sample_rate = 16000
    chunk_size = 512  # 32ms分
    
    print("[INFO] 512サンプルチャンクでのモデル直接呼び出しテスト...")
    
    # テスト音声生成
    test_chunks = [
        np.random.normal(0, 0.01, chunk_size).astype(np.float32),  # ノイズのみ
        0.3 * np.sin(2 * np.pi * 440 * np.linspace(0, chunk_size/sample_rate, chunk_size)).astype(np.float32),  # 音声
        np.random.normal(0, 0.01, chunk_size).astype(np.float32),  # ノイズのみ
    ]
    
    labels = ["ノイズのみ", "音声(440Hz)", "ノイズのみ"]
    
    for i, (chunk, label) in enumerate(zip(test_chunks, labels)):
        try:
            with torch.no_grad():
                chunk_tensor = torch.FloatTensor(chunk).unsqueeze(0)
                speech_prob = model(chunk_tensor, sample_rate).item()
            
            is_speech = speech_prob >= 0.5
            status = "音声" if is_speech else "無声"
            
            print(f"チャンク {i+1} ({label}): 確率={speech_prob:.3f}, 判定={status}")
            
        except Exception as e:
            print(f"[ERROR] チャンク {i+1} 処理エラー: {e}")

def main():
    """メイン関数"""
    print("SileroVAD単体テストプログラム")
    print("=" * 50)
    
    # 1. モデル直接呼び出しテスト
    test_direct_model()
    
    # 2. リアルタイムVADテスト
    test_realtime_vad()
    
    # 3. 音声ファイルテスト（もしファイルがあれば）
    test_audio_files = [
        "/workspace/DiaROS_ros/static_response_source/static_response_1.wav",
        "/workspace/DiaROS_ros/start_announce.wav",
        "/workspace/DiaROS_ros/power_calibration.wav"
    ]
    
    for audio_file in test_audio_files:
        import os
        if os.path.exists(audio_file):
            test_with_audio_file(audio_file)
            break
    else:
        print("\n[INFO] テスト用音声ファイルが見つかりませんでした")
    
    print("\n" + "=" * 50)
    print("テスト完了")

if __name__ == "__main__":
    main()