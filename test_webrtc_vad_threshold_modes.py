#!/usr/bin/env python3
"""
WebRTC VADのモード(0-3)による判定の違いを検証するスクリプト。

WebRTC VADはユーザーが設定可能な閾値パラメータを持たず、
mode (0=最も寛容 ~ 3=最も厳格) が内部的に閾値を制御する。
"""

import numpy as np
import webrtcvad
import struct

def generate_test_audio(amplitude_ratio, duration_ms=30, sample_rate=16000):
    """
    指定された振幅比のテスト音声を生成

    Args:
        amplitude_ratio: 振幅比 (0.0~1.0)
        duration_ms: 音声長 (ミリ秒)
        sample_rate: サンプリングレート

    Returns:
        bytes: PCM音声データ
    """
    num_samples = int(sample_rate * duration_ms / 1000)
    # 1kHzのサイン波を生成
    t = np.linspace(0, duration_ms / 1000, num_samples)
    audio = np.sin(2 * np.pi * 1000 * t) * amplitude_ratio * 32767
    audio = audio.astype(np.int16)
    return audio.tobytes()

def test_all_modes_with_amplitudes():
    """様々な振幅レベルの音声に対して全モードの判定結果を表示"""

    print("=" * 80)
    print("WebRTC VAD モード別判定テスト")
    print("=" * 80)
    print()
    print("【説明】")
    print("- WebRTC VADには、ユーザーが設定可能な閾値パラメータはありません")
    print("- mode (0~3) が内部的に閾値を制御します")
    print("  - mode 0: 最も寛容（音声と判定しやすい）")
    print("  - mode 3: 最も厳格（無音と判定しやすい）")
    print("- 出力は True/False のみで、確率値は得られません")
    print()
    print("【SileroVAD との違い】")
    print("- SileroVAD: threshold=0.5 のように閾値を直接指定可能")
    print("- SileroVAD: 0.0~1.0 の確率値を出力")
    print("- WebRTC VAD: mode のみで間接的に制御、True/False のみ出力")
    print()
    print("-" * 80)
    print()

    # テストする振幅レベル
    amplitude_levels = [
        (1.0, "最大振幅（通常の音声）"),
        (0.5, "中程度の振幅"),
        (0.2, "小さい振幅"),
        (0.1, "非常に小さい振幅"),
        (0.05, "ほぼノイズレベル"),
        (0.01, "極小振幅"),
    ]

    # 各モードのVADインスタンスを作成
    vads = [webrtcvad.Vad(mode=i) for i in range(4)]

    # ヘッダー
    print(f"{'振幅レベル':<30} | mode 0 | mode 1 | mode 2 | mode 3")
    print("-" * 80)

    # 各振幅レベルでテスト
    for amplitude, description in amplitude_levels:
        audio_bytes = generate_test_audio(amplitude)

        # 各モードで判定
        results = []
        for vad in vads:
            is_speech = vad.is_speech(audio_bytes, 16000)
            results.append("音声" if is_speech else "無音")

        print(f"{description:<30} | {results[0]:^6} | {results[1]:^6} | {results[2]:^6} | {results[3]:^6}")

    print("-" * 80)
    print()
    print("【結論】")
    print("1. WebRTC VADは振幅レベルに応じて音声/無音を判定")
    print("2. mode が高いほど、音声と判定するための閾値が高い（厳格）")
    print("3. ユーザーが直接閾値を設定することはできず、mode で間接的に制御")
    print("4. 出力は常に True/False の二値のみ")
    print()

def test_output_type():
    """WebRTC VADの出力が確率値ではなく二値であることを確認"""

    print("=" * 80)
    print("WebRTC VAD 出力形式の確認")
    print("=" * 80)
    print()

    vad = webrtcvad.Vad(mode=2)
    audio = generate_test_audio(0.5)
    result = vad.is_speech(audio, 16000)

    print(f"返り値の型: {type(result)}")
    print(f"返り値: {result}")
    print(f"取りうる値: True または False のみ")
    print()
    print("【SileroVAD との比較】")
    print("- SileroVAD: float型の確率値 (例: 0.734)")
    print("- WebRTC VAD: bool型の二値 (True or False)")
    print()
    print("このため、WebRTC VADでは:")
    print("- 「どのくらい音声らしいか」の度合いは得られない")
    print("- 「音声である/ない」の判定のみ")
    print("=" * 80)
    print()

def main():
    """メイン関数"""
    test_all_modes_with_amplitudes()
    print()
    test_output_type()

if __name__ == "__main__":
    main()
