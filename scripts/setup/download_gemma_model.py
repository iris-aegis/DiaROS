#!/usr/bin/env python3
"""
Gemma 2モデルを事前にダウンロードするスクリプト
"""

import os
import sys
from pathlib import Path
from transformers import AutoTokenizer, AutoModelForCausalLM
from huggingface_hub import HfFolder

def get_hf_token():
    """HuggingFaceトークンを取得（環境変数またはCLIログインから）"""
    # 1. 環境変数から取得
    token = os.environ.get("HF_TOKEN") or os.environ.get("HUGGINGFACE_TOKEN")
    
    # 2. huggingface-cli loginのトークンを取得
    if not token:
        try:
            token = HfFolder.get_token()
        except:
            pass
    
    return token

def download_gemma_model():
    """Gemma 2モデルをダウンロード"""
    
    model_name = "google/gemma-2-2b-it"
    
    print(f"=== {model_name} モデルダウンロード ===")
    print("")
    
    # HuggingFaceトークンの取得
    hf_token = get_hf_token()
    
    if not hf_token:
        print("⚠️  警告: HuggingFaceトークンが見つかりません")
        print("")
        print("以下のいずれかの方法でログインしてください：")
        print("")
        print("方法1: HuggingFace CLIでログイン（推奨）")
        print("  huggingface-cli login")
        print("")
        print("方法2: 環境変数でトークンを設定")
        print("  export HF_TOKEN=your_huggingface_token")
        print("")
        return False
    
    print("✅ HuggingFaceトークンを検出しました")
    
    try:
        print("\nトークナイザーをダウンロード中...")
        tokenizer = AutoTokenizer.from_pretrained(
            model_name,
            token=hf_token,
            trust_remote_code=True
        )
        print("✅ トークナイザーのダウンロード完了")
        
        print("\nモデルをダウンロード中... (約5GB)")
        print("これには数分かかる場合があります")
        model = AutoModelForCausalLM.from_pretrained(
            model_name,
            token=hf_token,
            trust_remote_code=True,
            torch_dtype="auto",
            low_cpu_mem_usage=True
        )
        print("✅ モデルのダウンロード完了")
        
        # キャッシュの場所を表示
        cache_dir = os.path.expanduser("~/.cache/huggingface/hub")
        print(f"\nモデルは以下の場所にキャッシュされました：")
        print(f"  {cache_dir}")
        
        return True
        
    except Exception as e:
        error_str = str(e)
        print(f"\n❌ エラー: {error_str}")
        
        # エラーメッセージから原因を判定
        if "403" in error_str or "restricted" in error_str:
            print("\n原因: Gemma 2モデルへのアクセス許可がありません")
            print("\n解決方法：")
            print("1. 以下のURLにアクセス:")
            print("   https://huggingface.co/google/gemma-2-2b-it")
            print("")
            print("2. HuggingFaceアカウントでログイン")
            print("")
            print("3. 「Agree and access repository」ボタンをクリック")
            print("")
            print("4. このスクリプトを再実行")
        elif "connection" in error_str.lower() or "timeout" in error_str.lower():
            print("\n原因: インターネット接続の問題")
            print("安定した接続環境で再実行してください")
        elif "disk" in error_str.lower() or "space" in error_str.lower():
            print("\n原因: ディスク容量不足")
            print("5GB以上の空き容量を確保してください")
        else:
            print("\n考えられる原因：")
            print("1. HuggingFaceでGemma 2モデルへのアクセス許可がない")
            print("2. インターネット接続の問題")
            print("3. ディスク容量不足（5GB以上必要）")
        
        return False

def main():
    print("Gemma 2モデルの事前ダウンロードツール")
    print("=" * 50)
    
    # Pixi環境の確認
    if not os.environ.get("CONDA_PREFIX"):
        print("⚠️  警告: Pixi環境で実行してください")
        print("実行方法:")
        print("  cd ~/_data/_DiaROS_mac/DiaROS_pixi/diaros_workspace")
        print("  pixi shell")
        print("  python3 ../DiaROS_imamoto/scripts/setup/download_gemma_model.py")
        return
    
    success = download_gemma_model()
    
    if success:
        print("\n✅ 準備完了！")
        print("DiaROSを起動できます：")
        print("  ./scripts/launch/launch_diaros_local.sh")
    else:
        print("\n❌ ダウンロードに失敗しました")
        print("上記のエラーメッセージを確認してください")

if __name__ == "__main__":
    main()