#!/usr/bin/env python3
"""
script1.wavをストリーミング処理してSilero VADの処理時間を計測
- プログラム開始～終了までの全体時間を計測
- 1フレーム(32ms)ごとの平均処理時間を計算
"""

import numpy as np
import soundfile as sf
import time
import torch
import sys
import warnings
import datetime

# 警告を抑制
warnings.filterwarnings("ignore")

def load_silero_vad():
    """SileroVADモデルをロード"""
    try:
        # PyTorchスレッド数を制限（CPUパフォーマンス最適化）
        torch.set_num_threads(1)

        print("[INFO] SileroVADモデルをロード中... (V5 ONNX)")
        # ローカルキャッシュを使うように設定（ダウンロード済みの場合）
        model, utils = torch.hub.load(
            repo_or_dir='snakers4/silero-vad',
            model='silero_vad',
            force_reload=False,
            onnx=True,  # ★V5 ONNX に変更
            trust_repo=True
        )

        get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
        print("[INFO] SileroVADモデルのロードが完了しました (V5 ONNX)")
        
        return model, VADIterator
    except Exception as e:
        print(f"[ERROR] SileroVADロードに失敗: {e}")
        sys.exit(1)

def benchmark_silero_realtime(audio_file):
    start_time = datetime.datetime.now() # 開始時刻を取得
    """script1.wavをストリーミング処理して時間を計測"""
    print("=" * 80)
    print("Silero VAD リアルタイムストリーミング処理ベンチマーク")
    print("=" * 80)

    # 音声ファイル読み込み
    print(f"\n[INFO] 音声ファイル読み込み中: {audio_file}")
    try:
        audio_data, sr = sf.read(audio_file)
    except Exception as e:
        print(f"[ERROR] 音声ファイル読み込みエラー: {e}")
        sys.exit(1)

    print(f"  サンプリングレート: {sr}Hz")
    print(f"  音声長: {len(audio_data)}サンプル ({len(audio_data)/sr:.2f}秒)")

    # サンプリングレート確認
    if sr != 16000:
        print("[WARNING] サンプリングレートが16000Hzではありません。Silero VADは16000Hzまたは8000Hzを推奨します。")
        # 簡易的なリサンプリング（間引き）
        if sr == 48000:
            audio_data = audio_data[::3]
            sr = 16000
            print("  -> 16000Hzにダウンサンプリングしました")
        elif sr == 32000:
            audio_data = audio_data[::2]
            sr = 16000
            print("  -> 16000Hzにダウンサンプリングしました")

    # モノラル変換
    if audio_data.ndim > 1:
        audio_data = np.mean(audio_data, axis=1)

    # float32に変換
    audio_data = audio_data.astype(np.float32)

    # Silero VAD ロード
    model, VADIterator = load_silero_vad()

    # VAD設定
    # Silero VADは 512, 1024, 1536 samples のウィンドウサイズをサポート
    # 16kHzの場合: 512=32ms, 1024=64ms, 1536=96ms
    frame_duration = 32  # ms
    CHUNK = 512 # 32ms at 16kHz

    # VADIterator初期化
    vad_iterator = VADIterator(model, threshold=0.5, sampling_rate=sr)

    print(f"\n[INFO] VAD設定:")
    print(f"  - フレーム長: {frame_duration}ms ({CHUNK}サンプル)")
    print(f"  - サンプリングレート: {sr}Hz")

    # 総フレーム数とVAD判定結果を記録
    total_frames = 0
    speech_frames = 0
    silence_frames = 0

    # 状態管理
    previous_has_speech = None  # 前フレームが音声を含んでいたか
    silent_start_time_seconds = None  # 無音開始時刻
    speech_start_time_seconds = None  # 音声開始時刻
    sound_count = 0  # 連続音声フレームカウント
    silent_count = 0  # 連続無音フレームカウント

    # 処理時間計測用リスト
    frame_processing_times = []

    print(f"\n[INFO] ストリーミング処理開始...")
    print("-" * 80)

    # プログラム全体の開始時刻
    program_start_time = time.time()

    # チャンク単位でストリーミング処理
    # 端数は切り捨てまたはパディング（Sileroは固定長を好むためパディング推奨）
    total_chunks = (len(audio_data) + CHUNK - 1) // CHUNK

    for chunk_idx in range(total_chunks):
        # チャンク処理開始時刻
        frame_start_time = time.time()

        # 相対時刻を計算（秒）
        current_time_sec = (chunk_idx * frame_duration) / 1000.0

        # チャンクを取得
        start_idx = chunk_idx * CHUNK
        end_idx = min(start_idx + CHUNK, len(audio_data))
        audiodata = audio_data[start_idx:end_idx]

        # 不足分をゼロパディング
        if len(audiodata) < CHUNK:
            audiodata = np.pad(audiodata, (0, CHUNK - len(audiodata)))

        # Torch Tensorに変換
        chunk_tensor = torch.from_numpy(audiodata)

        try:
            # Silero VAD処理
            # vad_iteratorは音声区間の開始/終了辞書を返す
            # {'start': timestamp} or {'end': timestamp}
            speech_dict = vad_iterator(chunk_tensor, return_seconds=True)

            # 音声確率を取得（VADIteratorの内部で計算）
            # ここでは簡易的に、speech_dictが返された（音声検出）かどうかで判定
            has_speech = speech_dict is not None and (('start' in speech_dict) or ('end' in speech_dict))

            # 状態遷移の検出
            if previous_has_speech is not None and previous_has_speech != has_speech:
                if not has_speech:
                    # 音声から無音へ
                    print(f"[{current_time_sec:.3f}s] 音声終了 → 無音開始")
                    silent_start_time_seconds = current_time_sec
                    silent_count = 0
                else:
                    # 無音から音声へ
                    if silent_start_time_seconds is not None:
                        silent_duration = current_time_sec - silent_start_time_seconds
                        print(f"[{current_time_sec:.3f}s] 無音終了 → 音声開始 (無音継続: {silent_duration*1000:.0f}ms)")
                    else:
                        print(f"[{current_time_sec:.3f}s] 音声開始")
                    silent_start_time_seconds = None
                    sound_count = 0

            # VAD詳細イベントの出力
            if speech_dict is not None:
                if 'start' in speech_dict:
                    speech_start_time_seconds = current_time_sec
                if 'end' in speech_dict:
                    if speech_start_time_seconds is not None:
                        speech_duration = current_time_sec - speech_start_time_seconds
                        print(f"[{current_time_sec:.3f}s] ✓ 音声区間確定（継続: {speech_duration*1000:.0f}ms）")

            if has_speech:
                speech_frames += 1
                sound_count += 1
            else:
                silence_frames += 1
                silent_count += 1

                # 100ms無音検出（100msに達した時のみ出力）
                if silent_count == int(100 / frame_duration):
                    if silent_start_time_seconds is not None:
                        silent_duration = current_time_sec - silent_start_time_seconds
                        print(f"[{current_time_sec:.3f}s] ✓ 100ms無音検出完了（継続: {silent_duration*1000:.0f}ms）")

            total_frames += 1
            previous_has_speech = has_speech

        except Exception as e:
            print(f"[ERROR] VAD処理エラー at {current_time_sec:.3f}s: {e}")
            continue

        # 進捗表示
        if (chunk_idx + 1) % (max(1, total_chunks // 10)) == 0:
            progress = ((chunk_idx + 1) / total_chunks) * 100
            sys.stdout.write(f"\r[INFO] 処理中... {progress:.0f}%")
            sys.stdout.flush()

        # チャンク処理終了時刻
        frame_end_time = time.time()
        frame_processing_time_ms = (frame_end_time - frame_start_time) * 1000
        frame_processing_times.append(frame_processing_time_ms)

        # フレーム処理時間をリアルタイム出力
        print(f"[{current_time_sec:.3f}s] 処理時間: {frame_processing_time_ms:.4f}ms")

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
    print(f"  音声フレーム: {speech_frames} ({speech_frames/total_frames*100:.1f}%)" if total_frames > 0 else "  音声フレーム: 0 (0.0%)")
    print(f"  無音フレーム: {silence_frames} ({silence_frames/total_frames*100:.1f}%)" if total_frames > 0 else "  無音フレーム: 0 (0.0%)")

    print(f"\n[結果] 処理時間:")
    print(f"  ✅ 全フレーム処理時間の合計: {total_processing_time:.2f}ms")
    print(f"  📊 プログラム全体時間: {program_total_time:.2f}ms")
    print(f"  ⏱️ 実音声時間: {len(audio_data)/sr*1000:.2f}ms")

    # 1フレームあたりの平均処理時間
    avg_time_per_frame = total_processing_time / total_frames if total_frames > 0 else 0

    print(f"\n[結果] フレーム単位の処理時間統計:")
    print(f"  1フレーム（{frame_duration}ms）あたりの平均処理時間: {avg_time_per_frame:.4f}ms")
    print(f"  相対処理負荷: {avg_time_per_frame/frame_duration*100:.2f}% (リアルタイム要件: < 1%)")

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
    # 1秒間に含まれるフレーム数 = 1000ms / frame_duration_ms
    frames_per_sec = 1000 / frame_duration
    estimated_1sec = avg_time_per_frame * frames_per_sec

    print(f"\n[推定] 1秒間の処理時間:")
    print(f"  推定処理時間: {estimated_1sec:.2f}ms")
    print(f"  相対処理負荷: {estimated_1sec/1000*100:.2f}% (リアルタイム要件: < 1%)")

    # リアルタイム判定
    print(f"\n[判定]")
    # 処理時間がフレーム長より短ければリアルタイム可能
    # さらに余裕を持って 1/10 (10%) 以下なら安全圏
    if avg_time_per_frame < frame_duration * 0.1:
        print(f"  ✅ リアルタイム処理可能（余裕度: {(frame_duration - avg_time_per_frame):.3f}ms/フレーム）")
    elif avg_time_per_frame < frame_duration:
        print(f"  ⚠️ リアルタイム処理可能だが負荷が高い（余裕度: {(frame_duration - avg_time_per_frame):.3f}ms/フレーム）")
    else:
        print(f"  ❌ リアルタイム処理不可（超過: {(avg_time_per_frame - frame_duration):.3f}ms/フレーム）")

    end_time = datetime.datetime.now() # 終了時刻を取得
    total_time = end_time - start_time # 差分を計算
    total_ms = total_time.total_seconds() * 1000

    return {
        'audio_file': audio_file,
        'total_frames': total_frames,
        'speech_frames': speech_frames,
        'silence_frames': silence_frames,
        'total_processing_time_ms': total_processing_time,
        'program_total_time_ms': program_total_time,
        'avg_time_per_frame_ms': avg_time_per_frame,
        'estimated_1sec_ms': estimated_1sec,
        'audio_duration_sec': len(audio_data) / sr,
        'realtime_capable': avg_time_per_frame < frame_duration,
        'frame_processing_times': frame_processing_times
    }



def main():
    start_time = datetime.datetime.now()  # 開始時刻を取得

    audio_file = '/workspace/script1.wav'

    # 音声ファイルの存在確認
    import os
    if not os.path.exists(audio_file):
        print(f"[ERROR] 音声ファイルが見つかりません: {audio_file}")
        return

    result = benchmark_silero_realtime(audio_file)

    end_time = datetime.datetime.now()  # 終了時刻を取得
    total_time = end_time - start_time  # 差分を計算
    total_ms = total_time.total_seconds() * 1000
    total_frames = result['total_frames']

    print("\n" + "=" * 80)
    print("終了")
    print("=" * 80)
    print(f"プログラム全体の実行時間: {total_ms:.2f} milliseconds")  # 差分をミリ秒で出力

    if total_frames > 0:
        avg_time_overall = total_ms / total_frames
        print(f"1フレーム(32ms)あたりの平均処理時間 (全体): {avg_time_overall:.4f} milliseconds")

if __name__ == "__main__":
    main()
