#!/usr/bin/env python3
"""
WebRTC VADの無声区間検出タイミング検証スクリプト
100ms無声で検出できるか、それとも200ms必要なのかを詳細にログ出力
"""

import numpy as np
import webrtcvad
import librosa
from scipy.io import wavfile
import sys

# 設定
sample_rate = 16000
frame_duration = 30  # ms
frame_samples = int(sample_rate * frame_duration / 1000)  # 480サンプル

# WAVファイル読み込み
wav_file = '/workspace/script1.wav'
print(f"WAVファイル読み込み: {wav_file}")
print("=" * 80)

# librosaで読み込み（サンプルレート変換対応）
audio_data, orig_sr = librosa.load(wav_file, sr=sample_rate, mono=True)
audio_data_int16 = np.array(audio_data * 32767, dtype=np.int16)

print(f"サンプリングレート: {sample_rate}Hz")
print(f"音声長: {len(audio_data_int16) / sample_rate:.2f}秒 ({len(audio_data_int16)}サンプル)")
print(f"フレームサイズ: {frame_duration}ms ({frame_samples}サンプル)")
print("=" * 80)

# WebRTC VAD初期化
vad = webrtcvad.Vad()
vad.set_mode(3)  # 最も厳しいモード（mic_input_original.pyと同じ）

# 状態管理変数
sound_available = False
sound_count = 0
silent_count = 0
sound_buffer = np.empty(0, dtype=np.float32)

# 検出イベント記録
events = []

# フレームごとに処理
total_frames = len(audio_data_int16) // frame_samples
print(f"総フレーム数: {total_frames}\n")
print("時刻(ms)  | フレーム# | VAD判定 | sound_count | silent_count | イベント")
print("-" * 80)

for frame_idx in range(total_frames):
    # 現在のフレームを取得
    start_sample = frame_idx * frame_samples
    end_sample = start_sample + frame_samples

    if end_sample > len(audio_data_int16):
        break

    frame_audio = audio_data_int16[start_sample:end_sample]
    frame_bytes = frame_audio.tobytes()

    # 音声バッファに追加
    frame_float = audio_data[start_sample:end_sample]
    sound_buffer = np.concatenate([sound_buffer, frame_float])

    # 5.1秒に制限
    if sound_buffer.shape[0] >= int(5.1 * sample_rate):
        sound_buffer = sound_buffer[-int(5.1 * sample_rate):]

    # VAD判定
    current_time_ms = frame_idx * frame_duration
    is_speech = vad.is_speech(frame_bytes, sample_rate)

    event_str = ""

    if is_speech:
        # 音声セグメント
        silent_count = 0
        sound_count += 1

        # 200msより短い音声区間ならノイズとして無視
        if sound_count >= (200 / frame_duration):  # 6.67 → 7フレーム = 210ms
            if not sound_available:
                sound_available = True
                event_str = "✓ 音声区間開始（200ms達成）"
                events.append({
                    'time_ms': current_time_ms,
                    'event': '音声区間開始',
                    'sound_count': sound_count,
                    'silent_count': silent_count
                })

    elif sound_buffer.shape[0] >= 5 * sample_rate:
        # 無音セグメント（5秒以上の音声がある場合のみ）
        sound_count = 0
        silent_count += 1

        # 100ms無声検出
        if silent_count >= (100 / frame_duration):  # 3.33 → 4フレーム = 120ms
            if sound_available:
                event_str = f"★ 100ms無声検出！（実際: {silent_count * frame_duration}ms）"
                events.append({
                    'time_ms': current_time_ms,
                    'event': '100ms無声検出',
                    'sound_count': sound_count,
                    'silent_count': silent_count,
                    'actual_silence_ms': silent_count * frame_duration
                })
                sound_available = False

    # ログ出力（重要なフレームのみ）
    if is_speech or silent_count > 0 or event_str:
        vad_label = "音声" if is_speech else "無音"
        print(f"{current_time_ms:7d} | {frame_idx:9d} | {vad_label:4s} | "
              f"{sound_count:11d} | {silent_count:12d} | {event_str}")

print("=" * 80)
print("\n【検出イベントまとめ】")
print("-" * 80)
for event in events:
    print(f"{event['time_ms']:7d}ms: {event['event']}")
    if 'actual_silence_ms' in event:
        print(f"           → 無声継続時間: {event['actual_silence_ms']}ms "
              f"(フレーム数: {event['silent_count']})")
        print(f"           → 設定値100msに対して実際の検出: {event['actual_silence_ms']}ms")

print("\n【結論】")
print("-" * 80)
silence_detections = [e for e in events if e['event'] == '100ms無声検出']
if silence_detections:
    avg_actual_ms = np.mean([e['actual_silence_ms'] for e in silence_detections])
    print(f"100ms無声検出の実際の平均時間: {avg_actual_ms:.1f}ms")
    print(f"フレーム離散化による誤差: {avg_actual_ms - 100:.1f}ms")
    print()
    print("✓ WebRTC VADは100ms設定で動作している")
    print(f"✓ ただし、{frame_duration}msフレーム単位で判定するため、")
    print(f"  実際には ceil(100/{frame_duration}) = {int(np.ceil(100/frame_duration))}フレーム")
    print(f"  = {int(np.ceil(100/frame_duration)) * frame_duration}ms の無声が必要")
else:
    print("✗ 100ms無声検出イベントが発生しませんでした")
    print("  音声ファイルに100ms以上の無声区間がない可能性があります")
