#!/usr/bin/env python3
"""
WebRTC VAD: バッチ処理 vs ストリーミング処理の比較検証
1. 判定結果が同じかどうか
2. 処理速度の違い
"""

import numpy as np
import webrtcvad
import librosa
import time
from scipy.io import wavfile

# 設定
sample_rate = 16000
frame_duration = 30  # ms
frame_samples = int(sample_rate * frame_duration / 1000)  # 480サンプル

# WAVファイル読み込み
wav_file = '/workspace/script1.wav'
print(f"WAVファイル読み込み: {wav_file}")
print("=" * 80)

audio_data, orig_sr = librosa.load(wav_file, sr=sample_rate, mono=True)
audio_data_int16 = np.array(audio_data * 32767, dtype=np.int16)

print(f"サンプリングレート: {sample_rate}Hz")
print(f"音声長: {len(audio_data_int16) / sample_rate:.2f}秒 ({len(audio_data_int16)}サンプル)")
print(f"フレームサイズ: {frame_duration}ms ({frame_samples}サンプル)")

total_frames = len(audio_data_int16) // frame_samples
print(f"総フレーム数: {total_frames}")
print("=" * 80)

# WebRTC VAD初期化
vad_batch = webrtcvad.Vad()
vad_batch.set_mode(3)

vad_stream = webrtcvad.Vad()
vad_stream.set_mode(3)

print("\n【方法1: バッチ処理（一気に全フレーム処理）】")
print("-" * 80)

batch_results = []
batch_times = []

batch_start = time.perf_counter()

for frame_idx in range(total_frames):
    start_sample = frame_idx * frame_samples
    end_sample = start_sample + frame_samples

    if end_sample > len(audio_data_int16):
        break

    frame_audio = audio_data_int16[start_sample:end_sample]
    frame_bytes = frame_audio.tobytes()

    # 処理時間計測
    frame_start = time.perf_counter()
    is_speech = vad_batch.is_speech(frame_bytes, sample_rate)
    frame_end = time.perf_counter()

    frame_time_us = (frame_end - frame_start) * 1e6  # マイクロ秒

    batch_results.append(is_speech)
    batch_times.append(frame_time_us)

batch_total_time = time.perf_counter() - batch_start

print(f"総処理時間: {batch_total_time * 1000:.2f}ms")
print(f"平均処理時間/フレーム: {np.mean(batch_times):.2f}μs")
print(f"最小処理時間: {np.min(batch_times):.2f}μs")
print(f"最大処理時間: {np.max(batch_times):.2f}μs")
print(f"標準偏差: {np.std(batch_times):.2f}μs")
print(f"音声フレーム数: {sum(batch_results)} / {len(batch_results)}")

print("\n【方法2: ストリーミング処理（1フレームずつ逐次処理）】")
print("-" * 80)

stream_results = []
stream_times = []

stream_start = time.perf_counter()

for frame_idx in range(total_frames):
    start_sample = frame_idx * frame_samples
    end_sample = start_sample + frame_samples

    if end_sample > len(audio_data_int16):
        break

    frame_audio = audio_data_int16[start_sample:end_sample]
    frame_bytes = frame_audio.tobytes()

    # リアルタイム処理をシミュレート（実際にはsleep不要、単純に1フレームずつ処理）
    # time.sleep(0.03)  # 30msウェイト（コメントアウト: 純粋な処理速度測定）

    # 処理時間計測
    frame_start = time.perf_counter()
    is_speech = vad_stream.is_speech(frame_bytes, sample_rate)
    frame_end = time.perf_counter()

    frame_time_us = (frame_end - frame_start) * 1e6  # マイクロ秒

    stream_results.append(is_speech)
    stream_times.append(frame_time_us)

stream_total_time = time.perf_counter() - stream_start

print(f"総処理時間: {stream_total_time * 1000:.2f}ms")
print(f"平均処理時間/フレーム: {np.mean(stream_times):.2f}μs")
print(f"最小処理時間: {np.min(stream_times):.2f}μs")
print(f"最大処理時間: {np.max(stream_times):.2f}μs")
print(f"標準偏差: {np.std(stream_times):.2f}μs")
print(f"音声フレーム数: {sum(stream_results)} / {len(stream_results)}")

# 結果比較
print("\n【判定結果の比較】")
print("=" * 80)

if batch_results == stream_results:
    print("✓ バッチ処理とストリーミング処理の判定結果は完全に一致")
    print("  → WebRTC VADはステートレス（フレーム間で状態を保持しない）")
else:
    print("✗ 判定結果に差異あり")
    diff_count = sum(1 for b, s in zip(batch_results, stream_results) if b != s)
    print(f"  差異フレーム数: {diff_count} / {len(batch_results)}")

    # 差異の詳細表示（最初の10件）
    print("\n  差異の詳細（最初の10件）:")
    diff_shown = 0
    for idx, (b, s) in enumerate(zip(batch_results, stream_results)):
        if b != s:
            print(f"    フレーム#{idx} ({idx*30}ms): バッチ={b}, ストリーム={s}")
            diff_shown += 1
            if diff_shown >= 10:
                break

# 処理速度比較
print("\n【処理速度の比較】")
print("=" * 80)

speed_ratio = batch_total_time / stream_total_time
print(f"バッチ処理総時間: {batch_total_time * 1000:.2f}ms")
print(f"ストリーム処理総時間: {stream_total_time * 1000:.2f}ms")
print(f"速度比: {speed_ratio:.2f}x (バッチ/ストリーム)")

if speed_ratio > 1.1:
    print("→ バッチ処理の方が遅い（予想外）")
elif speed_ratio < 0.9:
    print("→ バッチ処理の方が速い")
else:
    print("→ ほぼ同じ速度")

# リアルタイム性の確認
print("\n【リアルタイム性の確認】")
print("=" * 80)

real_time_duration_ms = total_frames * frame_duration  # 実際の音声時間
avg_processing_time_us = np.mean(stream_times)
avg_processing_time_ms = avg_processing_time_us / 1000

print(f"音声の実時間: {real_time_duration_ms}ms ({real_time_duration_ms/1000:.2f}秒)")
print(f"1フレームの処理時間: {avg_processing_time_ms:.3f}ms")
print(f"1フレームの実時間: {frame_duration}ms")
print(f"リアルタイム係数: {avg_processing_time_ms / frame_duration:.4f}x")

if avg_processing_time_ms < frame_duration:
    margin = frame_duration - avg_processing_time_ms
    print(f"✓ リアルタイム処理可能！（余裕: {margin:.3f}ms = {margin/frame_duration*100:.1f}%）")
else:
    delay = avg_processing_time_ms - frame_duration
    print(f"✗ リアルタイム処理不可（遅延: {delay:.3f}ms）")

# 処理時間のヒストグラム（簡易版）
print("\n【ストリーム処理時間の分布】")
print("=" * 80)

bins = [0, 10, 20, 50, 100, 200, 500, 1000, float('inf')]
bin_labels = ['0-10μs', '10-20μs', '20-50μs', '50-100μs', '100-200μs', '200-500μs', '500-1000μs', '>1000μs']

hist, _ = np.histogram(stream_times, bins=bins)

for label, count in zip(bin_labels, hist):
    percentage = count / len(stream_times) * 100
    bar_length = int(percentage / 2)  # 50% = 25文字
    bar = '█' * bar_length
    print(f"{label:12s}: {count:4d} ({percentage:5.1f}%) {bar}")

print("\n【詳細統計（ストリーム処理）】")
print("=" * 80)
print(f"25パーセンタイル: {np.percentile(stream_times, 25):.2f}μs")
print(f"50パーセンタイル(中央値): {np.percentile(stream_times, 50):.2f}μs")
print(f"75パーセンタイル: {np.percentile(stream_times, 75):.2f}μs")
print(f"95パーセンタイル: {np.percentile(stream_times, 95):.2f}μs")
print(f"99パーセンタイル: {np.percentile(stream_times, 99):.2f}μs")
