#!/usr/bin/env python3
"""
音声再生機能のテストスクリプト
"""

import os
import sys
import time
import subprocess
from pydub import AudioSegment

def test_afplay():
    """afplayコマンドのテスト"""
    print("=== afplayコマンドテスト ===")
    
    # テスト音声ファイルのパスを探す
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "../.."))
    
    # 相槌ファイルをテスト
    backchannel_file = os.path.join(root_dir, "DiaROS_ros/static_back_channel_1.wav")
    
    if os.path.exists(backchannel_file):
        print(f"テストファイル: {backchannel_file}")
        print(f"ファイルサイズ: {os.path.getsize(backchannel_file)} bytes")
        
        # AudioSegmentで長さを確認
        try:
            audio = AudioSegment.from_wav(backchannel_file)
            duration = len(audio) / 1000.0
            print(f"音声長: {duration:.2f}秒")
        except Exception as e:
            print(f"AudioSegmentエラー: {e}")
        
        # afplayで再生
        print("\n音声を再生します...")
        start_time = time.time()
        result = os.system(f"afplay '{backchannel_file}'")
        end_time = time.time()
        
        print(f"再生時間: {end_time - start_time:.2f}秒")
        print(f"終了コード: {result}")
        
        if result == 0:
            print("✅ 音声再生成功！")
        else:
            print("❌ 音声再生失敗")
    else:
        print(f"❌ テストファイルが見つかりません: {backchannel_file}")

def test_voicevox_files():
    """VOICEVOXで生成された音声ファイルのテスト"""
    print("\n=== VOICEVOX生成ファイルテスト ===")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "../.."))
    tmp_dir = os.path.join(root_dir, "DiaROS_ros/tmp")
    
    if os.path.exists(tmp_dir):
        wav_files = sorted([f for f in os.listdir(tmp_dir) if f.endswith('.wav')])
        
        if wav_files:
            latest_file = os.path.join(tmp_dir, wav_files[-1])
            print(f"最新のVOICEVOX音声: {latest_file}")
            print(f"ファイルサイズ: {os.path.getsize(latest_file)} bytes")
            
            print("\n音声を再生します...")
            result = os.system(f"afplay '{latest_file}'")
            
            if result == 0:
                print("✅ VOICEVOX音声再生成功！")
            else:
                print("❌ VOICEVOX音声再生失敗")
        else:
            print("VOICEVOXで生成された音声ファイルがありません")
    else:
        print(f"tmpディレクトリが存在しません: {tmp_dir}")

def test_playsound():
    """playsoundライブラリのテスト"""
    print("\n=== playsoundライブラリテスト ===")
    
    try:
        from playsound import playsound
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        root_dir = os.path.abspath(os.path.join(script_dir, "../.."))
        backchannel_file = os.path.join(root_dir, "DiaROS_ros/static_back_channel_2.wav")
        
        if os.path.exists(backchannel_file):
            print(f"playsoundで再生: {backchannel_file}")
            playsound(backchannel_file, True)
            print("✅ playsound再生成功！")
        else:
            print("テストファイルが見つかりません")
    except Exception as e:
        print(f"❌ playsoundエラー: {e}")

def main():
    print("DiaROS音声再生テスト")
    print("=" * 50)
    
    # macOS環境の確認
    if sys.platform == "darwin":
        print("環境: macOS")
        print(f"Python: {sys.version}")
        
        # afplayの存在確認
        result = subprocess.run(["which", "afplay"], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"afplayパス: {result.stdout.strip()}")
        else:
            print("❌ afplayが見つかりません")
            return
    else:
        print(f"環境: {sys.platform}")
    
    # 各テストを実行
    test_afplay()
    test_voicevox_files()
    test_playsound()
    
    print("\nテスト完了")

if __name__ == "__main__":
    main()