#!/usr/bin/env python3
"""
SileroVAD詳細テスト - リアルタイム処理と確率出力
"""

import torch
import numpy as np
import time
from datetime import datetime
import soundfile as sf
import os

def test_chunk_by_chunk_processing():
    """実際の音声ファイルをチャンク単位で処理"""
    print("=== チャンク単位処理テスト ===")
    
    # SileroVADをロード
    torch.set_num_threads(1)
    model, utils = torch.hub.load(
        repo_or_dir='snakers4/silero-vad',
        model='silero_vad',
        force_reload=False
    )
    get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
    
    # テスト用音声ファイルを探す
    test_files = [
        "/workspace/DiaROS/DiaROS_ros/static_response_source/static_response_1.wav",
        "/workspace/DiaROS/DiaROS_ros/start_announce.wav",
        "/workspace/DiaROS/DiaROS_ros/power_calibration.wav"
    ]
    
    audio_file = None
    for f in test_files:
        if os.path.exists(f):
            audio_file = f
            break
    
    if audio_file is None:
        print("[ERROR] テスト用音声ファイルが見つかりません")
        return
    
    print(f"[INFO] 音声ファイル: {audio_file}")
    
    # 音声読み込み
    wav = read_audio(audio_file)
    print(f"[INFO] 音声長: {len(wav)/16000:.2f}秒")
    
    # チャンク単位で処理
    chunk_size = 512  # 32ms分
    sample_rate = 16000
    
    print(f"\n[INFO] {chunk_size}サンプル（{chunk_size/sample_rate*1000:.1f}ms）ずつ処理...")
    
    # 方法1: モデル直接呼び出し
    print("\n--- モデル直接呼び出し結果 ---")
    model.eval()
    
    speech_probabilities = []
    silent_start_time = None
    speech_detected = False
    
    for i in range(0, len(wav), chunk_size):
        chunk = wav[i:i+chunk_size]
        if len(chunk) < chunk_size:
            chunk = np.pad(chunk, (0, chunk_size - len(chunk)))
        
        current_time = i / sample_rate
        
        try:
            with torch.no_grad():
                chunk_tensor = torch.FloatTensor(chunk).unsqueeze(0)
                speech_prob = model(chunk_tensor, sample_rate).item()
            
            speech_probabilities.append(speech_prob)
            is_speech = speech_prob >= 0.5
            
            # 状態変化を検出
            if is_speech and not speech_detected:
                print(f"  {current_time:.3f}s: 音声開始 (prob={speech_prob:.3f})")
                speech_detected = True
                silent_start_time = None
            elif not is_speech and speech_detected:
                print(f"  {current_time:.3f}s: 音声終了 (prob={speech_prob:.3f})")
                speech_detected = False
                silent_start_time = current_time
            
            # 100ms無声継続チェック
            if silent_start_time is not None and not speech_detected:
                silent_duration = current_time - silent_start_time
                if silent_duration >= 0.1:  # 100ms
                    print(f"  {current_time:.3f}s: 100ms無声区間検出 ({silent_duration*1000:.0f}ms継続)")
                    silent_start_time = None  # 一度だけ報告
            
            if i % (chunk_size * 10) == 0:  # 10チャンクごとに詳細表示
                print(f"  {current_time:.3f}s: prob={speech_prob:.3f}, status={'音声' if is_speech else '無声'}")
                
        except Exception as e:
            print(f"  ERROR at {current_time:.3f}s: {e}")
            continue
    
    # 統計表示
    total_chunks = len(speech_probabilities)
    speech_chunks = sum(1 for p in speech_probabilities if p >= 0.5)
    avg_prob = np.mean(speech_probabilities)
    max_prob = max(speech_probabilities)
    min_prob = min(speech_probabilities)
    
    print(f"\n--- 統計 ---")
    print(f"総チャンク数: {total_chunks}")
    print(f"音声チャンク数: {speech_chunks} ({speech_chunks/total_chunks*100:.1f}%)")
    print(f"平均確率: {avg_prob:.3f}")
    print(f"最大確率: {max_prob:.3f}")
    print(f"最小確率: {min_prob:.3f}")
    
    # 方法2: get_speech_timestampsとの比較
    print(f"\n--- get_speech_timestamps結果（比較用） ---")
    speech_timestamps = get_speech_timestamps(
        wav,
        model,
        sampling_rate=sample_rate,
        threshold=0.5,
        min_speech_duration_ms=50,
        min_silence_duration_ms=100,
        return_seconds=True
    )
    
    print(f"検出された音声区間数: {len(speech_timestamps)}")
    for i, segment in enumerate(speech_timestamps):
        print(f"  区間 {i+1}: {segment['start']:.3f}s - {segment['end']:.3f}s")

def test_vad_iterator_detailed():
    """VADIteratorの詳細テスト"""
    print("\n=== VADIterator詳細テスト ===")
    
    # SileroVADをロード
    torch.set_num_threads(1)
    model, utils = torch.hub.load(
        repo_or_dir='snakers4/silero-vad',
        model='silero_vad',
        force_reload=False
    )
    get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
    
    # VADIteratorの初期化
    vad_iterator = VADIterator(
        model,
        threshold=0.5,
        sampling_rate=16000,
        min_silence_duration_ms=100,
        speech_pad_ms=30
    )
    
    # テスト用音声ファイル
    test_files = [
        "/workspace/DiaROS/DiaROS_ros/static_response_source/static_response_1.wav",
        "/workspace/DiaROS/DiaROS_ros/start_announce.wav"
    ]
    
    audio_file = None
    for f in test_files:
        if os.path.exists(f):
            audio_file = f
            break
    
    if audio_file is None:
        print("[ERROR] テスト用音声ファイルが見つかりません")
        return
    
    print(f"[INFO] 音声ファイル: {audio_file}")
    wav = read_audio(audio_file)
    
    # チャンク単位でVADIterator処理
    chunk_size = 512
    sample_rate = 16000
    
    print(f"[INFO] VADIteratorで{chunk_size}サンプルずつ処理...")
    
    events = []
    for i in range(0, len(wav), chunk_size):
        chunk = wav[i:i+chunk_size]
        if len(chunk) < chunk_size:
            chunk = np.pad(chunk, (0, chunk_size - len(chunk)))
        
        current_time = i / sample_rate
        
        try:
            result = vad_iterator(chunk)
            if result is not None:
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                print(f"[{timestamp}] {current_time:.3f}s: VADIterator結果 = {result}")
                events.append((current_time, result))
                
        except Exception as e:
            print(f"[ERROR] {current_time:.3f}s: {e}")
            continue
    
    print(f"\n[INFO] 検出されたイベント数: {len(events)}")
    for time_point, event in events:
        print(f"  {time_point:.3f}s: {event}")

def main():
    """メイン関数"""
    print("SileroVAD詳細テストプログラム")
    print("=" * 60)
    
    # 詳細なチャンク処理テスト
    test_chunk_by_chunk_processing()
    
    # VADIterator詳細テスト
    test_vad_iterator_detailed()
    
    print("\n" + "=" * 60)
    print("詳細テスト完了")

if __name__ == "__main__":
    main()