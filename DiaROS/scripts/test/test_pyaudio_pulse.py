#!/usr/bin/env python3
"""
PyAudioのPulseAudio接続テストスクリプト
"""

import os
import sys

# 環境変数を設定してPyAudioがPulseAudioを使用するようにする
os.environ['PULSE_LATENCY_MSEC'] = '30'

# PyAudioをインポート前に設定
import pyaudio
import numpy as np

def list_audio_devices():
    """利用可能な音声デバイスをリスト表示"""
    print("🎤 PyAudio デバイステスト")
    print("=" * 50)
    
    try:
        # PyAudioの初期化
        p = pyaudio.PyAudio()
        
        print(f"✅ PyAudioが初期化されました")
        print(f"デバイス数: {p.get_device_count()}")
        
        # ホストAPIの情報を表示
        print("\n📋 ホストAPI情報:")
        for i in range(p.get_host_api_count()):
            api_info = p.get_host_api_info_by_index(i)
            print(f"  [{i}] {api_info['name']} - デバイス数: {api_info['deviceCount']}")
        
        # デバイス情報を表示
        print("\n📋 音声デバイス一覧:")
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"\n入力デバイス [{i}]:")
                print(f"  名前: {info['name']}")
                print(f"  入力チャンネル: {info['maxInputChannels']}")
                print(f"  サンプルレート: {info['defaultSampleRate']} Hz")
                print(f"  ホストAPI: {p.get_host_api_info_by_index(info['hostApi'])['name']}")
        
        # デフォルトデバイスの情報
        try:
            default_input = p.get_default_input_device_info()
            print(f"\n🎯 デフォルト入力デバイス:")
            print(f"  インデックス: {default_input['index']}")
            print(f"  名前: {default_input['name']}")
        except:
            print("\n⚠️  デフォルト入力デバイスが見つかりません")
        
        p.terminate()
        
    except Exception as e:
        print(f"❌ エラー: {e}")
        print("\n考えられる原因:")
        print("1. PulseAudioサーバーに接続できない")
        print("2. ALSAのPulseAudioプラグインがインストールされていない")
        print("3. 環境設定が正しくない")
        
def test_recording():
    """簡単な録音テスト"""
    print("\n🔊 録音テスト（3秒間）")
    print("マイクに向かって話してください...")
    
    try:
        p = pyaudio.PyAudio()
        
        # デバイス選択
        device_index = None
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                device_index = i
                print(f"使用デバイス: [{i}] {info['name']}")
                break
        
        if device_index is None:
            print("❌ 入力デバイスが見つかりません")
            return
        
        # ストリームを開く
        stream = p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=16000,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=1024
        )
        
        # 録音
        frames = []
        for _ in range(0, int(16000 / 1024 * 3)):
            data = stream.read(1024, exception_on_overflow=False)
            frames.append(data)
            
            # レベル表示
            audio_data = np.frombuffer(data, dtype=np.float32)
            level = np.abs(audio_data).mean()
            bar = '█' * int(level * 100)
            print(f"\r音量: {bar:<50} {level:.4f}", end='', flush=True)
        
        print("\n✅ 録音完了")
        
        stream.stop_stream()
        stream.close()
        p.terminate()
        
    except Exception as e:
        print(f"\n❌ 録音エラー: {e}")

if __name__ == "__main__":
    # ALSA設定の確認
    print("🔧 ALSA設定確認:")
    if os.path.exists(os.path.expanduser("~/.asoundrc")):
        print("✅ ~/.asoundrc が存在します")
        with open(os.path.expanduser("~/.asoundrc"), 'r') as f:
            print("内容:")
            print(f.read())
    else:
        print("⚠️  ~/.asoundrc が存在しません")
    
    print("\n" + "=" * 50 + "\n")
    
    # デバイスリスト
    list_audio_devices()
    
    # 録音テスト
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        test_recording()