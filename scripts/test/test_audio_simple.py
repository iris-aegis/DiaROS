#!/usr/bin/env python3
"""
簡易音声デバイステストスクリプト
Docker環境内で音声入力が正しく動作するかを確認します。
"""

import pyaudio
import numpy as np
import time
import sys

def test_audio():
    """音声入力デバイスのテスト"""
    print("🎤 音声入力デバイステスト")
    print("=" * 40)
    
    # PyAudioを初期化
    p = pyaudio.PyAudio()
    
    # 利用可能なデバイスを表示
    print("\n📋 利用可能な音声入力デバイス:")
    device_count = p.get_device_count()
    input_devices = []
    
    for i in range(device_count):
        info = p.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:
            input_devices.append(i)
            print(f"  [{i}] {info['name']} (入力ch: {info['maxInputChannels']})")
    
    if not input_devices:
        print("❌ 音声入力デバイスが見つかりません")
        print("\nトラブルシューティング:")
        print("1. ホスト側でPulseAudioが起動しているか確認")
        print("2. docker-compose.ymlでPULSE_SERVERが設定されているか確認")
        p.terminate()
        return
    
    # デフォルトデバイスを取得
    try:
        default_input = p.get_default_input_device_info()
        print(f"\n🎯 デフォルト入力デバイス: [{default_input['index']}] {default_input['name']}")
    except:
        print("\n⚠️  デフォルト入力デバイスが設定されていません")
        default_input = None
    
    # 音声レベルのテスト
    print("\n🔊 音声レベルテスト (3秒間)")
    print("マイクに向かって話してください...")
    
    try:
        # 最初の利用可能なデバイスでテスト
        test_device = input_devices[0]
        if default_input:
            test_device = default_input['index']
            
        stream = p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=16000,
            input=True,
            input_device_index=test_device,
            frames_per_buffer=160
        )
        
        start_time = time.time()
        max_level = 0
        
        while time.time() - start_time < 3:
            data = stream.read(160, exception_on_overflow=False)
            audio_data = np.frombuffer(data, dtype=np.float32)
            level = np.abs(audio_data).mean()
            max_level = max(max_level, level)
            
            # 簡易レベルメーター
            bar_length = int(level * 100)
            bar = '█' * bar_length
            print(f"\rレベル: {bar:<50} {level:.4f}", end='', flush=True)
        
        print(f"\n\n📊 最大レベル: {max_level:.4f}")
        
        if max_level > 0.001:
            print("✅ 音声入力が正常に動作しています！")
        else:
            print("⚠️  音声が検出されませんでした")
            
        stream.stop_stream()
        stream.close()
        
    except Exception as e:
        print(f"\n❌ エラー: {e}")
        print("\n考えられる原因:")
        print("- PulseAudioサーバーに接続できない")
        print("- マイクの権限が付与されていない")
        print("- デバイスが使用中")
    
    finally:
        p.terminate()

if __name__ == "__main__":
    test_audio()