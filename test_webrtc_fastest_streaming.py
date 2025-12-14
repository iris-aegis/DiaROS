#!/usr/bin/env python3
"""
WebRTC VAD 最速設定（10msフレーム）でのストリーミング処理
全フレームの出力を表示して、100ms無声検出のタイミングを確認
"""

import numpy as np
import webrtcvad
import librosa
import time

# 設定: 最速の10msフレーム
sample_rate = 16000
frame_duration = 10  # ms（WebRTC VADの最小フレームサイズ）
frame_samples = int(sample_rate * frame_duration / 1000)  # 160サンプル

# WAVファイル読み込み
wav_file = '/workspace/DiaROS/script1.wav'
print(f"WAVファイル読み込み: {wav_file}")
print("=" * 80)

audio_data, orig_sr = librosa.load(wav_file, sr=sample_rate, mono=True)
audio_data_int16 = np.array(audio_data * 32767, dtype=np.int16)

print(f"サンプリングレート: {sample_rate}Hz")
print(f"音声長: {len(audio_data_int16) / sample_rate:.2f}秒 ({len(audio_data_int16)}サンプル)")
print(f"フレームサイズ: {frame_duration}ms ({frame_samples}サンプル) ← 最速設定")
total_frames = len(audio_data_int16) // frame_samples
print(f"総フレーム数: {total_frames}")
print("=" * 80)

# WebRTC VAD初期化（mode 3 = 最も厳しい判定）
vad = webrtcvad.Vad()
vad.set_mode(3)

# 状態管理変数（mic_input_original.pyと同じロジック）
sound_available = False
sound_count = 0
silent_count = 0
sound_buffer = np.empty(0, dtype=np.float32)

# 検出イベント記録
events = []
frame_results = []  # 全フレームの結果を記録

print("\n【ストリーミング処理: 全フレーム出力】")
print("-" * 100)
print("時刻(ms) | フレーム# | VAD判定 | sound_count | silent_count | 処理時間(μs) | イベント")
print("-" * 100)

# ストリーミング処理
for frame_idx in range(total_frames):
    start_sample = frame_idx * frame_samples
    end_sample = start_sample + frame_samples

    if end_sample > len(audio_data_int16):
        break

    # フレームデータ取得
    frame_audio = audio_data_int16[start_sample:end_sample]
    frame_bytes = frame_audio.tobytes()
    frame_float = audio_data[start_sample:end_sample]

    # 音声バッファに追加
    sound_buffer = np.concatenate([sound_buffer, frame_float])

    # 5.1秒に制限
    if sound_buffer.shape[0] >= int(5.1 * sample_rate):
        sound_buffer = sound_buffer[-int(5.1 * sample_rate):]

    # VAD判定（処理時間計測）
    frame_start = time.perf_counter()
    is_speech = vad.is_speech(frame_bytes, sample_rate)
    frame_end = time.perf_counter()
    processing_time_us = (frame_end - frame_start) * 1e6

    current_time_ms = frame_idx * frame_duration
    event_str = ""

    # 音声/無音判定ロジック（mic_input_original.pyと同じ）
    if is_speech:
        # 音声セグメント
        silent_count = 0
        sound_count += 1

        # 200ms音声継続で音声区間確定
        if sound_count >= (200 / frame_duration):  # 200/10 = 20フレーム
            if not sound_available:
                sound_available = True
                event_str = "✓ 音声区間開始（200ms達成）"
                events.append({
                    'time_ms': current_time_ms,
                    'frame': frame_idx,
                    'event': '音声区間開始',
                    'sound_count': sound_count,
                    'silent_count': silent_count
                })

    elif sound_buffer.shape[0] >= 5 * sample_rate:
        # 無音セグメント（5秒以上の音声がある場合のみ）
        sound_count = 0
        silent_count += 1

        # 100ms無声検出
        if silent_count >= (100 / frame_duration):  # 100/10 = 10フレーム
            if sound_available:
                actual_silence_ms = silent_count * frame_duration
                event_str = f"★ 100ms無声検出！（実際: {actual_silence_ms}ms, {silent_count}フレーム目）"
                events.append({
                    'time_ms': current_time_ms,
                    'frame': frame_idx,
                    'event': '100ms無声検出',
                    'sound_count': sound_count,
                    'silent_count': silent_count,
                    'actual_silence_ms': actual_silence_ms
                })
                sound_available = False

    # 結果を記録
    frame_results.append({
        'time_ms': current_time_ms,
        'frame': frame_idx,
        'is_speech': is_speech,
        'sound_count': sound_count,
        'silent_count': silent_count,
        'processing_time_us': processing_time_us,
        'event': event_str
    })

    # 出力（重要なフレームと定期的なサンプル）
    should_print = (
        event_str != "" or  # イベントがある
        frame_idx < 50 or  # 最初の50フレーム
        frame_idx % 50 == 0 or  # 50フレームごと
        (silent_count > 0 and silent_count <= 15)  # 無声開始から15フレーム
    )

    if should_print:
        vad_label = "音声" if is_speech else "無音"
        print(f"{current_time_ms:8d} | {frame_idx:9d} | {vad_label:4s} | "
              f"{sound_count:11d} | {silent_count:12d} | {processing_time_us:12.2f} | {event_str}")

print("=" * 100)

# イベントまとめ
print("\n【検出イベントまとめ】")
print("-" * 100)
for event in events:
    print(f"{event['time_ms']:8d}ms (フレーム#{event['frame']:5d}): {event['event']}")
    if 'actual_silence_ms' in event:
        print(f"             → 無声継続時間: {event['actual_silence_ms']}ms "
              f"(フレーム数: {event['silent_count']})")
        print(f"             → 設定値100msに対して実際の検出: {event['actual_silence_ms']}ms")

# 統計情報
print("\n【統計情報】")
print("=" * 100)

speech_frames = sum(1 for r in frame_results if r['is_speech'])
silence_frames = len(frame_results) - speech_frames
processing_times = [r['processing_time_us'] for r in frame_results]

print(f"総フレーム数: {len(frame_results)}")
print(f"音声フレーム数: {speech_frames} ({speech_frames/len(frame_results)*100:.1f}%)")
print(f"無音フレーム数: {silence_frames} ({silence_frames/len(frame_results)*100:.1f}%)")
print()
print(f"平均処理時間: {np.mean(processing_times):.2f}μs")
print(f"最小処理時間: {np.min(processing_times):.2f}μs")
print(f"最大処理時間: {np.max(processing_times):.2f}μs")
print(f"中央値: {np.median(processing_times):.2f}μs")
print(f"95パーセンタイル: {np.percentile(processing_times, 95):.2f}μs")

# 100ms無声検出の詳細分析
print("\n【100ms無声検出の詳細】")
print("=" * 100)

silence_detections = [e for e in events if e['event'] == '100ms無声検出']
if silence_detections:
    print(f"検出回数: {len(silence_detections)}回")
    print()

    for idx, detection in enumerate(silence_detections, 1):
        print(f"[検出#{idx}]")
        print(f"  時刻: {detection['time_ms']}ms (フレーム#{detection['frame']})")
        print(f"  無声継続時間: {detection['actual_silence_ms']}ms")
        print(f"  無声フレーム数: {detection['silent_count']}フレーム")

        # この検出の直前フレームを確認
        frame_num = detection['frame']
        print(f"  直前10フレームの状態:")
        for i in range(max(0, frame_num - 10), frame_num + 1):
            if i < len(frame_results):
                r = frame_results[i]
                vad_label = "音声" if r['is_speech'] else "無音"
                marker = " ← 検出" if i == frame_num else ""
                print(f"    フレーム#{i:5d} ({r['time_ms']:5d}ms): {vad_label}, silent_count={r['silent_count']}{marker}")
        print()

    avg_detection_time = np.mean([d['actual_silence_ms'] for d in silence_detections])
    print(f"平均検出時間: {avg_detection_time:.1f}ms")
else:
    print("100ms無声検出イベントなし")

# リアルタイム性能
print("\n【リアルタイム性能】")
print("=" * 100)

real_time_duration_ms = len(frame_results) * frame_duration
total_processing_time_ms = sum(processing_times) / 1000

print(f"音声の実時間: {real_time_duration_ms}ms ({real_time_duration_ms/1000:.2f}秒)")
print(f"総処理時間: {total_processing_time_ms:.3f}ms")
print(f"リアルタイム係数: {total_processing_time_ms / real_time_duration_ms:.6f}x")
print()

avg_processing_ms = np.mean(processing_times) / 1000
margin_ms = frame_duration - avg_processing_ms
print(f"1フレームの実時間: {frame_duration}ms")
print(f"1フレームの処理時間: {avg_processing_ms:.4f}ms")
print(f"処理余裕: {margin_ms:.4f}ms ({margin_ms/frame_duration*100:.2f}%)")

if avg_processing_ms < frame_duration:
    print(f"✓ リアルタイム処理可能（余裕率: {margin_ms/frame_duration*100:.2f}%）")
else:
    print(f"✗ リアルタイム処理不可")

# 最速設定の利点
print("\n【10msフレーム設定の利点】")
print("=" * 100)
print("100ms無声検出に必要なフレーム数:")
print(f"  10msフレーム: {int(np.ceil(100/10))}フレーム = {int(np.ceil(100/10))*10}ms")
print(f"  20msフレーム: {int(np.ceil(100/20))}フレーム = {int(np.ceil(100/20))*20}ms")
print(f"  30msフレーム: {int(np.ceil(100/30))}フレーム = {int(np.ceil(100/30))*30}ms")
print()
print("→ 10msフレームが最も100ms設定に近い検出が可能（誤差: 0ms）")
