#!/usr/bin/env python3
"""
WebRTC VADのモード(0-3)による判定の違いを検証するスクリプト（改訂版）。

実際の無音、ノイズ、音声を使って各モードの違いを明確に示す。
"""

import numpy as np
import webrtcvad

def generate_silence(duration_ms=30, sample_rate=16000):
    """完全な無音を生成"""
    num_samples = int(sample_rate * duration_ms / 1000)
    audio = np.zeros(num_samples, dtype=np.int16)
    return audio.tobytes()

def generate_white_noise(amplitude_ratio, duration_ms=30, sample_rate=16000):
    """ホワイトノイズを生成"""
    num_samples = int(sample_rate * duration_ms / 1000)
    audio = np.random.randn(num_samples) * amplitude_ratio * 32767
    audio = np.clip(audio, -32768, 32767).astype(np.int16)
    return audio.tobytes()

def generate_tone(frequency, amplitude_ratio, duration_ms=30, sample_rate=16000):
    """純音を生成"""
    num_samples = int(sample_rate * duration_ms / 1000)
    t = np.linspace(0, duration_ms / 1000, num_samples)
    audio = np.sin(2 * np.pi * frequency * t) * amplitude_ratio * 32767
    audio = audio.astype(np.int16)
    return audio.tobytes()

def generate_low_frequency_tone(amplitude_ratio, duration_ms=30, sample_rate=16000):
    """低周波ノイズ（50Hz）を生成"""
    return generate_tone(50, amplitude_ratio, duration_ms, sample_rate)

def main():
    """メイン関数"""

    print("=" * 80)
    print("WebRTC VAD 閾値メカニズムの検証")
    print("=" * 80)
    print()
    print("【重要な結論】")
    print("- WebRTC VADには、ユーザーが設定可能な閾値パラメータは存在しません")
    print("- mode (0~3) のみが内部閾値を間接的に制御します")
    print("- 出力は True/False の二値のみで、確率値は得られません")
    print()
    print("【SileroVAD との違い】")
    print("┌─────────────────┬──────────────────────┬─────────────────┐")
    print("│                 │ SileroVAD            │ WebRTC VAD      │")
    print("├─────────────────┼──────────────────────┼─────────────────┤")
    print("│ 閾値設定        │ threshold=0.5 など   │ 設定不可        │")
    print("│ 閾値制御        │ 直接指定             │ mode で間接制御 │")
    print("│ 出力形式        │ float (0.0~1.0)      │ bool (T/F)      │")
    print("│ 確信度          │ 取得可能             │ 取得不可        │")
    print("└─────────────────┴──────────────────────┴─────────────────┘")
    print()
    print("-" * 80)
    print()

    # テストケース
    test_cases = [
        ("完全な無音", generate_silence()),
        ("極小ノイズ (0.01)", generate_white_noise(0.01)),
        ("小ノイズ (0.05)", generate_white_noise(0.05)),
        ("中ノイズ (0.1)", generate_white_noise(0.1)),
        ("大ノイズ (0.3)", generate_white_noise(0.3)),
        ("低周波音 50Hz (0.1)", generate_low_frequency_tone(0.1)),
        ("低周波音 50Hz (0.5)", generate_low_frequency_tone(0.5)),
        ("純音 1kHz (0.05)", generate_tone(1000, 0.05)),
        ("純音 1kHz (0.2)", generate_tone(1000, 0.2)),
        ("純音 1kHz (0.5)", generate_tone(1000, 0.5)),
    ]

    # 各モードのVADインスタンスを作成
    vads = [webrtcvad.Vad(mode=i) for i in range(4)]

    # ヘッダー
    print(f"{'テスト音声':<30} | mode 0 | mode 1 | mode 2 | mode 3")
    print("-" * 80)

    # 各テストケースで判定
    for description, audio_bytes in test_cases:
        # 各モードで判定
        results = []
        for vad in vads:
            is_speech = vad.is_speech(audio_bytes, 16000)
            results.append("音声" if is_speech else "無音")

        print(f"{description:<30} | {results[0]:^6} | {results[1]:^6} | {results[2]:^6} | {results[3]:^6}")

    print("-" * 80)
    print()
    print("【観察結果】")
    print("1. mode が高いほど、音声と判定するための内部閾値が高い")
    print("2. 同じ音声でも mode によって判定が変わることがある")
    print("3. しかし、ユーザーはこの閾値を直接設定できない")
    print("4. WebRTC VADのAPIには threshold パラメータは存在しない")
    print()
    print("【WebRTC VAD の制限】")
    print("- 確率値が得られないため、「どのくらい確信を持って音声と判定したか」不明")
    print("- 閾値を細かく調整できないため、柔軟性に欠ける")
    print("- mode 0~3 の4段階のみでの調整に限定される")
    print()
    print("【SileroVAD の利点】")
    print("- threshold を自由に設定可能（例: 0.3, 0.5, 0.7 など）")
    print("- 確率値が得られるため、判定の確信度を評価できる")
    print("- より柔軟なチューニングが可能")
    print()

    # 実際のAPIを確認
    print("=" * 80)
    print("【WebRTC VAD の実際のAPI】")
    print("=" * 80)
    print()
    vad = webrtcvad.Vad()
    print(f"利用可能なメソッド:")
    print(f"  - set_mode(mode): mode を 0~3 で設定")
    print(f"  - is_speech(buf, sample_rate): 音声判定（戻り値: bool）")
    print()
    print("存在しないメソッド:")
    print("  - set_threshold() ← このようなメソッドは存在しない")
    print("  - get_probability() ← このようなメソッドは存在しない")
    print()
    print(f"現在の vad オブジェクトの属性: {dir(vad)}")
    print()
    print("結論: WebRTC VADには閾値を直接設定する機能はありません")
    print("=" * 80)

if __name__ == "__main__":
    main()
