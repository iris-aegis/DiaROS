#!/usr/bin/env python3
"""
macOSネイティブ環境でDiaROSを実行するスクリプト（ROS2なし）
MPSを活用して高速な音声対話を実現
"""
import os
import sys
import subprocess
import signal
import time
from pathlib import Path

# DiaROS_pyディレクトリをPythonパスに追加
diaros_py_path = Path(__file__).parent.parent / "DiaROS_py"
sys.path.insert(0, str(diaros_py_path))

def check_dependencies():
    """依存関係の確認"""
    print("🔍 依存関係を確認中...")
    
    # PyTorchとMPSの確認
    try:
        import torch
        mps_available = torch.backends.mps.is_available() if hasattr(torch.backends, 'mps') else False
        print(f"✅ PyTorch: {torch.__version__}")
        print(f"✅ MPS利用可能: {mps_available}")
        if mps_available:
            print("   🚀 Apple Silicon GPUを使用します")
    except ImportError:
        print("❌ PyTorchがインストールされていません")
        print("   pip install torch torchvision torchaudio")
        return False
    
    # その他の依存関係
    required_packages = [
        "transformers", "numpy", "scipy", "librosa", 
        "soundfile", "pyaudio", "aubio", "requests"
    ]
    
    missing = []
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            missing.append(package)
    
    if missing:
        print(f"❌ 不足パッケージ: {', '.join(missing)}")
        print(f"   pip install {' '.join(missing)}")
        return False
    
    print("✅ すべての依存関係が満たされています")
    return True

def check_voicevox():
    """VOICEVOXの起動確認"""
    print("\n🔍 VOICEVOX Engineを確認中...")
    try:
        import requests
        response = requests.get("http://localhost:50021/version", timeout=1)
        if response.status_code == 200:
            print("✅ VOICEVOX Engine が起動しています")
            return True
    except:
        pass
    
    print("⚠️  VOICEVOX Engine が起動していません")
    print("   別ターミナルでVOICEVOX Engineを起動してください")
    print("   https://github.com/VOICEVOX/voicevox_engine/releases")
    return False

def set_environment():
    """環境変数の設定"""
    print("\n⚙️  環境変数を設定中...")
    
    # MPSを優先使用
    os.environ["DIAROS_DEVICE"] = "mps"
    
    # CPU最適化
    os.environ["OMP_NUM_THREADS"] = "8"
    os.environ["TORCH_NUM_THREADS"] = "8"
    
    # HuggingFaceトークン（設定されている場合）
    if "HF_TOKEN" in os.environ:
        print("✅ HuggingFaceトークンが設定されています")
    else:
        print("⚠️  HuggingFaceトークンが未設定です（一部モデルが使用できない可能性があります）")
    
    print("✅ 環境変数の設定完了")

def run_diaros_standalone():
    """スタンドアロンモードでDiaROSを実行"""
    print("\n🚀 DiaROSをスタンドアロンモードで起動します...")
    print("   （ROS2を使用しない簡易実行モード）")
    
    # 各モジュールを直接インポートして実行
    try:
        # 必要なモジュールをインポート
        from diaros.speechInput import SpeechInput
        from diaros.automaticSpeechRecognition import runASR
        from diaros.dialogManagement import DialogManagement
        from diaros.speechSynthesis import runSpeechSynthesis
        
        print("\n✨ DiaROSモジュールの読み込み完了")
        print("📢 音声対話を開始します（Ctrl+Cで終了）")
        print("-" * 50)
        
        # ここで簡易的な対話ループを実装
        # （実際の実装は各モジュールの統合が必要）
        print("\n⚠️  注意: スタンドアロンモードは開発中です")
        print("   完全な機能にはROS2環境が必要です")
        
        # 簡易デモ：音声入力デバイスの確認
        speech_input = SpeechInput(16000, 160)
        print(f"\n🎤 音声入力デバイス: {speech_input.mic_info['name']}")
        print("   話しかけてください...")
        
        # 実際の対話ループはここに実装
        # ...
        
    except ImportError as e:
        print(f"❌ モジュールのインポートエラー: {e}")
        print("   DiaROS_pyディレクトリが正しい場所にあるか確認してください")
        return False
    except Exception as e:
        print(f"❌ エラーが発生しました: {e}")
        return False
    
    return True

def main():
    """メイン処理"""
    print("=" * 60)
    print("🤖 DiaROS macOSネイティブ実行スクリプト")
    print("=" * 60)
    
    # 依存関係チェック
    if not check_dependencies():
        print("\n❌ 依存関係を満たしてから再実行してください")
        return 1
    
    # VOICEVOX確認
    if not check_voicevox():
        print("\n⚠️  VOICEVOXなしで続行します（音声合成は利用できません）")
        time.sleep(2)
    
    # 環境設定
    set_environment()
    
    # DiaROS実行
    try:
        run_diaros_standalone()
    except KeyboardInterrupt:
        print("\n\n👋 DiaROSを終了します")
        return 0
    except Exception as e:
        print(f"\n❌ 予期しないエラー: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())