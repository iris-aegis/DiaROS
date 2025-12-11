#!/usr/bin/env python3
"""
VAD処理時間ベンチマーク
- SileroVAD: 32ms チャンクの推論時間
- VADIterator: チャンク処理時間（32ms単位）
- webRTC VAD: 10ms フレームの判定時間

複数回実行して平均時間、最小時間、最大時間を計測
"""

import torch
import numpy as np
import time
import sys
import warnings
import webrtcvad

warnings.filterwarnings("ignore")

# SileroVADのロード
print("[INFO] SileroVADをロード中...")
try:
    torch.set_num_threads(1)
    silero_vad_model, utils = torch.hub.load(
        repo_or_dir='snakers4/silero-vad',
        model='silero_vad',
        force_reload=False,
        onnx=False
    )
    get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
    print("[INFO] SileroVAD ロード完了")
except Exception as e:
    print(f"[ERROR] SileroVADのロードに失敗: {e}")
    sys.exit(1)

def benchmark_silero_vad_32ms(num_iterations=1000):
    """
    SileroVAD: 32msチャンクの推論時間を計測
    """
    print("\n" + "=" * 80)
    print("【SileroVAD】32msチャンク推論ベンチマーク")
    print("=" * 80)

    sample_rate = 16000
    chunk_32ms_samples = int(sample_rate * 0.032)  # 512サンプル

    # テスト用音声データ生成（ランダムノイズ）
    test_audio = np.random.randn(chunk_32ms_samples).astype(np.float32) * 0.1

    print(f"[INFO] テスト設定:")
    print(f"  - チャンク: 32ms（{chunk_32ms_samples}サンプル）")
    print(f"  - 反復回数: {num_iterations}")
    print(f"  - デバイス: {next(silero_vad_model.parameters()).device}")

    times = []

    print(f"\n[INFO] ベンチマーク実行中...", end="", flush=True)
    silero_vad_model.eval()

    with torch.no_grad():
        # ウォームアップ（最初の10回は除外）
        for _ in range(10):
            chunk_tensor = torch.FloatTensor(test_audio).unsqueeze(0)
            _ = silero_vad_model(chunk_tensor, sample_rate)

        # 計測
        for i in range(num_iterations):
            if (i + 1) % (num_iterations // 10) == 0:
                print(".", end="", flush=True)

            chunk_tensor = torch.FloatTensor(test_audio).unsqueeze(0)

            start_time = time.perf_counter()
            output = silero_vad_model(chunk_tensor, sample_rate)
            elapsed = (time.perf_counter() - start_time) * 1000  # ms

            times.append(elapsed)

    print(" 完了")

    # 統計情報
    times = np.array(times)
    print(f"\n[結果] SileroVAD 32ms推論時間:")
    print(f"  平均: {np.mean(times):.4f} ms")
    print(f"  最小: {np.min(times):.4f} ms")
    print(f"  最大: {np.max(times):.4f} ms")
    print(f"  標準偏差: {np.std(times):.4f} ms")
    print(f"  中央値: {np.median(times):.4f} ms")
    print(f"  P95: {np.percentile(times, 95):.4f} ms")
    print(f"  P99: {np.percentile(times, 99):.4f} ms")

    return {
        'method': 'SileroVAD',
        'chunk_ms': 32,
        'mean': np.mean(times),
        'min': np.min(times),
        'max': np.max(times),
        'std': np.std(times),
        'median': np.median(times),
        'p95': np.percentile(times, 95),
        'p99': np.percentile(times, 99),
        'iterations': num_iterations
    }

def benchmark_vad_iterator_32ms(num_iterations=1000):
    """
    VADIterator: 32msチャンク処理時間を計測
    """
    print("\n" + "=" * 80)
    print("【VADIterator】32msチャンク処理ベンチマーク")
    print("=" * 80)

    sample_rate = 16000
    chunk_32ms_samples = int(sample_rate * 0.032)  # 512サンプル

    # VADIterator初期化
    vad_iterator = VADIterator(
        silero_vad_model,
        threshold=0.5,
        sampling_rate=sample_rate,
        min_silence_duration_ms=100,
        speech_pad_ms=32
    )

    # テスト用音声データ生成
    test_audio = np.random.randn(chunk_32ms_samples).astype(np.float32) * 0.1

    print(f"[INFO] テスト設定:")
    print(f"  - チャンク: 32ms（{chunk_32ms_samples}サンプル）")
    print(f"  - 反復回数: {num_iterations}")
    print(f"  - 設定: min_silence=100ms, speech_pad=32ms")

    times = []

    print(f"\n[INFO] ベンチマーク実行中...", end="", flush=True)

    # ウォームアップ
    for _ in range(10):
        _ = vad_iterator(test_audio)

    # 計測
    for i in range(num_iterations):
        if (i + 1) % (num_iterations // 10) == 0:
            print(".", end="", flush=True)

        start_time = time.perf_counter()
        result = vad_iterator(test_audio)
        elapsed = (time.perf_counter() - start_time) * 1000  # ms

        times.append(elapsed)

    print(" 完了")

    # 統計情報
    times = np.array(times)
    print(f"\n[結果] VADIterator 32ms処理時間:")
    print(f"  平均: {np.mean(times):.4f} ms")
    print(f"  最小: {np.min(times):.4f} ms")
    print(f"  最大: {np.max(times):.4f} ms")
    print(f"  標準偏差: {np.std(times):.4f} ms")
    print(f"  中央値: {np.median(times):.4f} ms")
    print(f"  P95: {np.percentile(times, 95):.4f} ms")
    print(f"  P99: {np.percentile(times, 99):.4f} ms")

    return {
        'method': 'VADIterator',
        'chunk_ms': 32,
        'mean': np.mean(times),
        'min': np.min(times),
        'max': np.max(times),
        'std': np.std(times),
        'median': np.median(times),
        'p95': np.percentile(times, 95),
        'p99': np.percentile(times, 99),
        'iterations': num_iterations
    }

def benchmark_webrtc_vad_10ms(num_iterations=1000):
    """
    webRTC VAD: 10msフレームの判定時間を計測
    """
    print("\n" + "=" * 80)
    print("【webRTC VAD】10msフレーム判定ベンチマーク")
    print("=" * 80)

    sample_rate = 16000
    frame_10ms_samples = int(sample_rate * 0.01)  # 160サンプル

    # webRTC VAD初期化（各モード）
    modes = {
        0: "QUALITY（最敏感）",
        1: "NORMAL（通常）",
        2: "AGGRESSIVE（やや厳格）",
        3: "VERY_AGGRESSIVE（最も厳格）"
    }

    results = []

    for mode, mode_name in modes.items():
        print(f"\n[INFO] モード {mode}: {mode_name}")

        vad = webrtcvad.Vad()
        vad.set_mode(mode)

        # テスト用音声データ生成（int16）
        test_audio_float = np.random.randn(frame_10ms_samples).astype(np.float32) * 0.1
        test_audio_int16 = (test_audio_float * 32767).astype(np.int16)
        test_audio_bytes = test_audio_int16.tobytes()

        print(f"  テスト設定:")
        print(f"    - フレーム: 10ms（{frame_10ms_samples}サンプル）")
        print(f"    - 反復回数: {num_iterations}")

        times = []

        print(f"  ベンチマーク実行中...", end="", flush=True)

        # ウォームアップ
        for _ in range(10):
            _ = vad.is_speech(test_audio_bytes, sample_rate)

        # 計測
        for i in range(num_iterations):
            if (i + 1) % (num_iterations // 10) == 0:
                print(".", end="", flush=True)

            start_time = time.perf_counter()
            is_speech = vad.is_speech(test_audio_bytes, sample_rate)
            elapsed = (time.perf_counter() - start_time) * 1000  # ms

            times.append(elapsed)

        print(" 完了")

        # 統計情報
        times = np.array(times)
        print(f"  結果:")
        print(f"    平均: {np.mean(times):.4f} ms")
        print(f"    最小: {np.min(times):.4f} ms")
        print(f"    最大: {np.max(times):.4f} ms")
        print(f"    標準偏差: {np.std(times):.4f} ms")
        print(f"    中央値: {np.median(times):.4f} ms")
        print(f"    P95: {np.percentile(times, 95):.4f} ms")
        print(f"    P99: {np.percentile(times, 99):.4f} ms")

        results.append({
            'method': 'webRTC VAD',
            'mode': mode,
            'mode_name': mode_name,
            'chunk_ms': 10,
            'mean': np.mean(times),
            'min': np.min(times),
            'max': np.max(times),
            'std': np.std(times),
            'median': np.median(times),
            'p95': np.percentile(times, 95),
            'p99': np.percentile(times, 99),
            'iterations': num_iterations
        })

    return results

def main():
    print("\n" + "=" * 80)
    print("VAD処理時間ベンチマーク")
    print("=" * 80)

    num_iterations = 1000
    print(f"[INFO] 各手法を{num_iterations}回実行して処理時間を計測します\n")

    # 各VAD方式のベンチマーク実行
    silero_result = benchmark_silero_vad_32ms(num_iterations)
    vad_iter_result = benchmark_vad_iterator_32ms(num_iterations)
    webrtc_results = benchmark_webrtc_vad_10ms(num_iterations)

    # 比較表
    print("\n" + "=" * 80)
    print("【処理時間比較表】")
    print("=" * 80)

    print("\n【SileroVAD vs VADIterator】（32msチャンク）")
    print("-" * 80)
    print(f"{'手法':<20} {'平均':<12} {'最小':<12} {'最大':<12} {'中央値':<12} {'P99':<12}")
    print("-" * 80)
    print(f"{'SileroVAD':<20} {silero_result['mean']:<11.4f}ms {silero_result['min']:<11.4f}ms {silero_result['max']:<11.4f}ms {silero_result['median']:<11.4f}ms {silero_result['p99']:<11.4f}ms")
    print(f"{'VADIterator':<20} {vad_iter_result['mean']:<11.4f}ms {vad_iter_result['min']:<11.4f}ms {vad_iter_result['max']:<11.4f}ms {vad_iter_result['median']:<11.4f}ms {vad_iter_result['p99']:<11.4f}ms")

    ratio = vad_iter_result['mean'] / silero_result['mean']
    print(f"\n[INFO] VADIterator/SileroVAD: {ratio:.2f}x")

    print("\n【webRTC VAD】（10msフレーム）")
    print("-" * 80)
    print(f"{'モード':<20} {'平均':<12} {'最小':<12} {'最大':<12} {'中央値':<12} {'P99':<12}")
    print("-" * 80)
    for result in webrtc_results:
        print(f"{'Mode ' + str(result['mode']) + ' ' + result['mode_name']:<20} {result['mean']:<11.4f}ms {result['min']:<11.4f}ms {result['max']:<11.4f}ms {result['median']:<11.4f}ms {result['p99']:<11.4f}ms")

    # DiaROS環境での処理時間推定
    print("\n" + "=" * 80)
    print("【DiaROS環境での実行時間推定】")
    print("=" * 80)

    print("\n【シナリオ】1秒間の音声処理")
    print("-" * 80)

    # SileroVAD: 32msチャンク → 1秒 = 31.25チャンク
    silero_1s = silero_result['mean'] * (1000 / 32)
    print(f"SileroVAD (32ms):")
    print(f"  チャンク数: {1000 / 32:.1f}回")
    print(f"  推定時間: {silero_1s:.2f}ms")

    # VADIterator: 32msチャンク → 1秒 = 31.25チャンク
    vad_iter_1s = vad_iter_result['mean'] * (1000 / 32)
    print(f"\nVADIterator (32ms):")
    print(f"  チャンク数: {1000 / 32:.1f}回")
    print(f"  推定時間: {vad_iter_1s:.2f}ms")

    # webRTC VAD: 10msフレーム → 1秒 = 100フレーム
    webrtc_mode3_result = [r for r in webrtc_results if r['mode'] == 3][0]
    webrtc_1s = webrtc_mode3_result['mean'] * 100
    print(f"\nwebRTC VAD Mode 3 (10ms):")
    print(f"  フレーム数: {1000 / 10}回")
    print(f"  推定時間: {webrtc_1s:.2f}ms")

    print("\n[結論]")
    print(f"  - webRTC VADが最も高速（1秒あたり {webrtc_1s:.2f}ms）")
    print(f"  - SileroVADは中程度（1秒あたり {silero_1s:.2f}ms）")
    print(f"  - VADIteratorがやや遅い（1秒あたり {vad_iter_1s:.2f}ms）")

    # リアルタイム判定
    print("\n【リアルタイム処理可能性】")
    print("-" * 80)
    print(f"webRTC VAD Mode 3: {webrtc_1s:.2f}ms / 1000ms = {webrtc_1s/1000*100:.1f}% ✅ リアルタイム可能")
    print(f"SileroVAD: {silero_1s:.2f}ms / 1000ms = {silero_1s/1000*100:.1f}% ✅ リアルタイム可能")
    print(f"VADIterator: {vad_iter_1s:.2f}ms / 1000ms = {vad_iter_1s/1000*100:.1f}% ✅ リアルタイム可能")

if __name__ == "__main__":
    main()
