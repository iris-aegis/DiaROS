#!/usr/bin/env python3
"""
script1.wavをストリーミング処理して実際の処理時間を計測
- プログラム開始～終了までの全体時間を計測
- 1フレームごとの平均処理時間を計算
"""

import numpy as np
import soundfile as sf
import time
import webrtcvad
import sys
import datetime # datetimeモジュールをインポート

def benchmark_streaming_realtime(audio_file):
    """script1.wavをストリーミング処理して時間を計測"""
    print("=" * 80)
    print("リアルタイムストリーミング処理ベンチマーク")
    print("=" * 80)

    # 音声ファイル読み込み
    print(f"\n[INFO] 音声ファイル読み込み中: {audio_file}")
    audio_data, sr = sf.read(audio_file)
    print(f"  サンプリングレート: {sr}Hz")
    print(f"  音声長: {len(audio_data)}サンプル ({len(audio_data)/sr:.2f}秒)")

    # モノラル変換
    if audio_data.ndim > 1:
        audio_data = np.mean(audio_data, axis=1)

    # float32に変換
    audio_data = audio_data.astype(np.float32)

    # VAD設定
    sample_rate = 16000
    frame_duration = 10  # ms
    CHUNK = int(sample_rate * frame_duration / 1000)  # 160サンプル

    # webRTC VAD初期化（Mode 3）
    vad = webrtcvad.Vad()
    vad.set_mode(3)

    print(f"\n[INFO] VAD設定:")
    print(f"  - フレーム長: {frame_duration}ms ({CHUNK}サンプル)")
    print(f"  - VADモード: 3 (VERY_AGGRESSIVE)")
    print(f"  - サンプリングレート: {sample_rate}Hz")

    # 状態管理
    sound_count = 0
    silent_count = 0
    silent_start_time_seconds = None
    speech_detected_for_200ms = False
    sound = np.empty(0, dtype='float32')
    previous_is_speech = None  # 前フレームの状態

    # 総フレーム数とVAD判定結果を記録
    total_frames = 0
    speech_frames = 0
    silence_frames = 0

    # 処理時間計測用リスト
    frame_processing_times = []

    print(f"\n[INFO] ストリーミング処理開始...")
    print("-" * 80)

    # プログラム全体の開始時刻
    program_start_time = time.time()

    # 10msチャンク単位でストリーミング処理
    total_chunks = (len(audio_data) + CHUNK - 1) // CHUNK

    for chunk_idx in range(total_chunks):
        # チャンク処理開始時刻（フレームごと）
        frame_start_time = time.time()

        # 相対時刻を計算（秒）
        current_time_sec = (chunk_idx * frame_duration) / 1000.0

        # 10msチャンクを取得
        start_idx = chunk_idx * CHUNK
        end_idx = min(start_idx + CHUNK, len(audio_data))
        audiodata = audio_data[start_idx:end_idx]

        # 不足分をゼロパディング
        if len(audiodata) < CHUNK:
            audiodata = np.pad(audiodata, (0, CHUNK - len(audiodata)))

        # 音声バッファに追加
        sound = np.concatenate([sound, audiodata])

        # webRTC VAD判定
        audio_int16 = (audiodata * 32767).astype(np.int16)
        audio_bytes = audio_int16.tobytes()

        try:
            is_speech = vad.is_speech(audio_bytes, sample_rate)
            total_frames += 1

            # 状態遷移の検出（音声→無音、無音→音声）
            if previous_is_speech is not None and previous_is_speech != is_speech:
                if not is_speech:
                    # 音声から無音へ
                    print(f"[{current_time_sec:.3f}s] 音声終了 → 無音開始")
                    silent_start_time_seconds = current_time_sec
                else:
                    # 無音から音声へ
                    if silent_start_time_seconds is not None:
                        silent_duration = current_time_sec - silent_start_time_seconds
                        print(f"[{current_time_sec:.3f}s] 無音終了 → 音声開始 (無音継続: {silent_duration*1000:.0f}ms)")
                    else:
                        print(f"[{current_time_sec:.3f}s] 音声開始")
                    silent_start_time_seconds = None

            if not is_speech:
                silence_frames += 1
                if silent_start_time_seconds is None and sound_count > 0:
                    silent_start_time_seconds = current_time_sec
                silent_count += 1
                sound_count = 0

                # 100ms無音検出（100msに達した時のみ出力）
                if silent_count == int(100 / frame_duration) and speech_detected_for_200ms:
                    if silent_start_time_seconds is not None:
                        silent_duration = current_time_sec - silent_start_time_seconds
                        print(f"[{current_time_sec:.3f}s] ✓ 100ms無音検出完了（継続: {silent_duration*1000:.0f}ms）")
            else:
                speech_frames += 1
                if silent_start_time_seconds is not None:
                    silent_start_time_seconds = None
                    silent_count = 0

                sound_count += 1

                # 200ms音声検出
                if sound_count >= (200 / frame_duration):
                    if not speech_detected_for_200ms:
                        print(f"[{current_time_sec:.3f}s] 200ms以上の音声を検出")
                    speech_detected_for_200ms = True

            previous_is_speech = is_speech

        except Exception as e:
            print(f"[ERROR] VAD処理エラー at {current_time_sec:.3f}s: {e}")
            continue

        # チャンク処理終了時刻
        frame_end_time = time.time()
        frame_processing_time_ms = (frame_end_time - frame_start_time) * 1000
        frame_processing_times.append(frame_processing_time_ms)

        # フレーム処理時間をリアルタイム出力
        print(f"[{current_time_sec:.3f}s] 処理時間: {frame_processing_time_ms:.4f}ms")

        # 進捗表示
        if (chunk_idx + 1) % (total_chunks // 10) == 0:
            progress = ((chunk_idx + 1) / total_chunks) * 100
            sys.stdout.write(f"\r[INFO] 処理中... {progress:.0f}%")
            sys.stdout.flush()

    # プログラム全体の終了時刻
    program_end_time = time.time()

    print("\r" + " " * 40 + "\r", end="")  # 進捗表示をクリア
    print("-" * 80)

    # 計測結果
    # frame_processing_timesは各フレームの処理時間のリスト
    if len(frame_processing_times) > 0:
        total_processing_time = sum(frame_processing_times)  # ms
    else:
        total_processing_time = 0

    program_total_time = (program_end_time - program_start_time) * 1000  # ms

    print(f"\n[結果] 計測情報:")
    print(f"  総フレーム数: {total_frames}")
    print(f"  音声フレーム: {speech_frames} ({speech_frames/total_frames*100:.1f}%)")
    print(f"  無音フレーム: {silence_frames} ({silence_frames/total_frames*100:.1f}%)")

    print(f"\n[結果] 処理時間:")
    print(f"  ✅ 全フレーム処理時間の合計: {total_processing_time:.2f}ms")
    print(f"  📊 プログラム全体時間: {program_total_time:.2f}ms")
    print(f"  ⏱️ 実音声時間: {len(audio_data)/sample_rate*1000:.2f}ms")

    # 1フレームあたりの平均処理時間
    avg_time_per_frame = total_processing_time / total_frames if total_frames > 0 else 0

    print(f"\n[結果] フレーム単位の処理時間統計:")
    print(f"  1フレーム（10ms）あたりの平均処理時間: {avg_time_per_frame:.4f}ms")
    print(f"  相対処理負荷: {avg_time_per_frame/10*100:.2f}% (リアルタイム要件: < 1%)")

    # 詳細な統計情報
    if len(frame_processing_times) > 0:
        min_frame_time = min(frame_processing_times)
        max_frame_time = max(frame_processing_times)
        median_frame_time = np.median(frame_processing_times)
        p95_frame_time = np.percentile(frame_processing_times, 95)
        p99_frame_time = np.percentile(frame_processing_times, 99)

        print(f"\n[詳細] フレーム処理時間の分布:")
        print(f"  最小: {min_frame_time:.4f}ms")
        print(f"  最大: {max_frame_time:.4f}ms")
        print(f"  中央値: {median_frame_time:.4f}ms")
        print(f"  P95: {p95_frame_time:.4f}ms")
        print(f"  P99: {p99_frame_time:.4f}ms")

    # 1秒あたりの推定処理時間
    estimated_1sec = avg_time_per_frame * 100  # 100フレーム = 1秒

    print(f"\n[推定] 1秒間の処理時間:")
    print(f"  推定処理時間: {estimated_1sec:.2f}ms")
    print(f"  相対処理負荷: {estimated_1sec/1000*100:.2f}% (リアルタイム要件: < 1%)")

    # リアルタイム判定
    print(f"\n[判定]")
    if avg_time_per_frame < 0.1:
        print(f"  ✅ リアルタイム処理可能（余裕度: {(10 - avg_time_per_frame):.3f}ms/フレーム）")
    else:
        print(f"  ⚠️ リアルタイム処理に課題あり（超過: {(avg_time_per_frame - 10):.3f}ms/フレーム）")

    # 比較
    print(f"\n[比較] ベンチマーク結果との比較:")
    print(f"  webRTC VAD (Mode 3単体): 0.0011ms/フレーム")
    print(f"  本スクリプト（全処理含む）: {avg_time_per_frame:.4f}ms/フレーム")
    print(f"  オーバーヘッド: {avg_time_per_frame - 0.0011:.4f}ms/フレーム")

    return {
        'audio_file': audio_file,
        'total_frames': total_frames,
        'speech_frames': speech_frames,
        'silence_frames': silence_frames,
        'total_processing_time_ms': total_processing_time,
        'program_total_time_ms': program_total_time,
        'avg_time_per_frame_ms': avg_time_per_frame,
        'estimated_1sec_ms': estimated_1sec,
        'audio_duration_sec': len(audio_data) / sample_rate,
        'realtime_capable': avg_time_per_frame < 0.1,
        'frame_processing_times': frame_processing_times
    }

def main():
    start_time = datetime.datetime.now() # 開始時刻を取得
    audio_file = '/workspace/script1.wav'
    result = benchmark_streaming_realtime(audio_file)
    end_time = datetime.datetime.now() # 終了時刻を取得
    
    total_time = end_time - start_time # 差分を計算
    total_ms = total_time.total_seconds() * 1000
    total_frames = result['total_frames']

    print("\n" + "=" * 80)
    print("終了")
    print("=" * 80)
    print(f"プログラム全体の実行時間: {total_ms:.2f} milliseconds") # 差分をミリ秒で出力

    if total_frames > 0:
        avg_time = total_ms / total_frames
        print(f"1フレーム(10ms)あたりの平均処理時間 (全体): {avg_time:.4f} milliseconds")

if __name__ == "__main__":
    main()
