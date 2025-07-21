#!/usr/bin/env python3
"""
DiaROS用デフォルトマイクデバイス設定スクリプト

このスクリプトは利用可能な音声入力デバイスをリストアップし、
ユーザーが選択したデバイスを環境変数として設定します。
"""

import pyaudio
import sys
import os
import numpy as np
import threading
import time
import queue

def list_audio_devices():
    """利用可能な音声入力デバイスをリストアップ"""
    p = pyaudio.PyAudio()
    
    print("\n🎤 利用可能な音声入力デバイス:")
    print("-" * 60)
    
    input_devices = []
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:
            input_devices.append((i, info))
            print(f"[{i}] {info['name']}")
            print(f"    チャンネル数: {info['maxInputChannels']}")
            print(f"    サンプルレート: {info['defaultSampleRate']} Hz")
            if 'pulse' in info['name'].lower():
                print("    📡 PulseAudioデバイス")
            print()
    
    p.terminate()
    return input_devices

def test_audio_device(device_index, duration=3):
    """指定されたデバイスで音声入力をテスト"""
    p = pyaudio.PyAudio()
    
    try:
        # デバイス情報を取得
        info = p.get_device_info_by_index(device_index)
        print(f"\n🔊 デバイス [{device_index}] {info['name']} でテスト中...")
        
        # 音声ストリームを開く
        stream = p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=16000,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=160
        )
        
        print(f"📊 {duration}秒間の音声レベルを表示します:")
        print("    話しかけてみてください...")
        
        # 音声レベルを表示
        start_time = time.time()
        max_level = 0.0
        
        while time.time() - start_time < duration:
            try:
                data = stream.read(160, exception_on_overflow=False)
                audio_data = np.frombuffer(data, dtype=np.float32)
                level = np.abs(audio_data).mean()
                max_level = max(max_level, level)
                
                # レベルメーターを表示
                bar_length = int(level * 200)
                bar = '█' * min(bar_length, 50)
                print(f"\r    レベル: {bar:<50} {level:.4f}", end='', flush=True)
                
            except Exception as e:
                print(f"\n    ⚠️  読み取りエラー: {e}")
                break
        
        print(f"\n    最大レベル: {max_level:.4f}")
        
        stream.stop_stream()
        stream.close()
        
        if max_level > 0.001:
            print("    ✅ 音声入力が検出されました！")
            return True
        else:
            print("    ⚠️  音声入力が検出されませんでした")
            return False
            
    except Exception as e:
        print(f"    ❌ エラー: {e}")
        return False
    finally:
        p.terminate()

def save_device_config(device_index):
    """選択されたデバイスを設定ファイルに保存"""
    config_dir = "/workspace/config"
    os.makedirs(config_dir, exist_ok=True)
    
    config_file = os.path.join(config_dir, "audio_device.conf")
    with open(config_file, 'w') as f:
        f.write(f"AUDIO_DEVICE_INDEX={device_index}\n")
    
    print(f"\n✅ デバイス設定を保存しました: {config_file}")
    print(f"   AUDIO_DEVICE_INDEX={device_index}")
    
    # 環境変数としても設定
    os.environ['AUDIO_DEVICE_INDEX'] = str(device_index)
    
    # bashrcに追加する設定を表示
    print("\n📝 以下の行を ~/.bashrc に追加することで永続化できます:")
    print(f"   export AUDIO_DEVICE_INDEX={device_index}")
    
    # DiaROS起動用のスクリプトを作成
    launch_script = os.path.join(config_dir, "launch_diaros_with_mic.sh")
    with open(launch_script, 'w') as f:
        f.write(f"""#!/bin/bash
# DiaROS起動スクリプト（設定済みマイクデバイス使用）

export AUDIO_DEVICE_INDEX={device_index}
echo "🎤 音声デバイス {device_index} を使用してDiaROSを起動します..."

cd /workspace/DiaROS_ros
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

ros2 launch diaros_package sdsmod.launch.py
""")
    
    os.chmod(launch_script, 0o755)
    print(f"\n🚀 DiaROS起動スクリプトを作成しました: {launch_script}")
    print("   実行: /workspace/config/launch_diaros_with_mic.sh")

def main():
    print("=" * 60)
    print("🎙️  DiaROS デフォルトマイクデバイス設定")
    print("=" * 60)
    
    # デバイスをリストアップ
    devices = list_audio_devices()
    
    if not devices:
        print("❌ 音声入力デバイスが見つかりません")
        sys.exit(1)
    
    # デバイスを選択
    while True:
        try:
            print("\n設定したいデバイスの番号を入力してください")
            print("（qで終了、aで自動検出モード）: ", end='')
            choice = input().strip()
            
            if choice.lower() == 'q':
                print("終了します")
                sys.exit(0)
            
            if choice.lower() == 'a':
                print("\n🔍 自動検出モードを選択しました")
                print("   DiaROSは起動時に最適なデバイスを自動選択します")
                # 設定ファイルを削除
                config_file = "/workspace/config/audio_device.conf"
                if os.path.exists(config_file):
                    os.remove(config_file)
                    print(f"   設定ファイルを削除しました: {config_file}")
                break
            
            device_index = int(choice)
            
            # 有効なデバイス番号か確認
            valid_indices = [d[0] for d in devices]
            if device_index not in valid_indices:
                print(f"⚠️  無効なデバイス番号です。{valid_indices}から選択してください")
                continue
            
            # デバイスをテスト
            print(f"\n選択されたデバイス: [{device_index}]")
            test = input("このデバイスをテストしますか？ (Y/n): ").strip().lower()
            
            if test != 'n':
                success = test_audio_device(device_index)
                if not success:
                    retry = input("\n別のデバイスを選択しますか？ (Y/n): ").strip().lower()
                    if retry != 'n':
                        continue
            
            # 設定を保存
            save_device_config(device_index)
            
            print("\n✨ 設定が完了しました！")
            print("\nDiaROSを起動する方法:")
            print("1. 設定済みスクリプトを使用: /workspace/config/launch_diaros_with_mic.sh")
            print("2. または環境変数を設定してから起動:")
            print(f"   export AUDIO_DEVICE_INDEX={device_index}")
            print("   ros2 launch diaros_package sdsmod.launch.py")
            
            break
            
        except ValueError:
            print("⚠️  数値を入力してください")
        except Exception as e:
            print(f"❌ エラーが発生しました: {e}")

if __name__ == "__main__":
    main()