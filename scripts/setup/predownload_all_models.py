#!/usr/bin/env python3
"""
音声対話システム用のモデル事前ダウンロードスクリプト
システム起動前にすべての必要なモデルをダウンロードしておくためのツール
"""

import os
import sys
import time
from pathlib import Path
from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
from huggingface_hub import HfFolder
import torch

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

def check_disk_space(required_gb=20):
    """必要なディスク容量をチェック（4ビット量子化モデル用）"""
    import shutil
    
    cache_dir = os.path.expanduser("~/.cache/huggingface/hub")
    os.makedirs(cache_dir, exist_ok=True)
    
    free_space = shutil.disk_usage(cache_dir).free / (1024**3)  # GB
    
    if free_space < required_gb:
        print(f"⚠️  警告: ディスク容量が不足しています")
        print(f"必要容量: {required_gb}GB（4ビット量子化モデル用）, 利用可能: {free_space:.1f}GB")
        return False
    
    print(f"✅ ディスク容量チェック OK ({free_space:.1f}GB利用可能)")
    return True

def download_model_safely(model_name, model_type="auto", custom_cache_dir=None, use_4bit=False):
    """安全にモデルをダウンロード（4ビット量子化対応）"""
    hf_token = get_hf_token()
    
    try:
        print(f"\n📥 {model_name} をダウンロード中...")
        if use_4bit:
            print(f"  🔧 4ビット量子化を使用します")
        
        kwargs = {
            "token": hf_token,
            "trust_remote_code": True,
            "low_cpu_mem_usage": True
        }
        
        # 4ビット量子化設定
        if use_4bit:
            quantization_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_compute_dtype=torch.float16,
                bnb_4bit_use_double_quant=True,
                bnb_4bit_quant_type="nf4"
            )
            kwargs["quantization_config"] = quantization_config
            kwargs["torch_dtype"] = torch.float16
        else:
            kwargs["torch_dtype"] = "auto"
        
        if custom_cache_dir:
            kwargs["cache_dir"] = custom_cache_dir
        
        start_time = time.time()
        
        # トークナイザーをダウンロード
        tokenizer = AutoTokenizer.from_pretrained(model_name, **{k: v for k, v in kwargs.items() if k not in ["quantization_config", "torch_dtype"]})
        print(f"  ✅ トークナイザーダウンロード完了")
        
        # モデルをダウンロード
        model = AutoModelForCausalLM.from_pretrained(model_name, **kwargs)
        
        download_time = time.time() - start_time
        print(f"  ✅ モデルダウンロード完了 ({download_time:.1f}秒)")
        
        # メモリ解放
        del model
        del tokenizer
        
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        
        return True
        
    except Exception as e:
        error_str = str(e)
        print(f"  ❌ エラー: {error_str}")
        
        # エラーメッセージから原因を判定
        if "403" in error_str or "restricted" in error_str:
            print(f"  原因: {model_name}へのアクセス許可がありません")
            print(f"  解決方法: https://huggingface.co/{model_name} でアクセス許可を取得")
        elif "connection" in error_str.lower() or "timeout" in error_str.lower():
            print(f"  原因: インターネット接続の問題")
        elif "disk" in error_str.lower() or "space" in error_str.lower():
            print(f"  原因: ディスク容量不足")
        elif "bitsandbytes" in error_str.lower():
            print(f"  原因: bitsandbytesライブラリが不足")
            print(f"  解決方法: pip install bitsandbytes")
        elif "argument of type 'NoneType' is not iterable" in error_str:
            print(f"  原因: 内部エラー（モデルは正常にダウンロードされた可能性があります）")
            print(f"  確認: キャッシュディレクトリでモデルファイルを確認してください")
        
        return False

def download_nlg_models():
    """自然言語生成用モデルをダウンロード（Gemma 3 4ビット量子化）"""
    print("\n🤖 自然言語生成モデルのダウンロード（Gemma 3 - 4ビット量子化）")
    print("=" * 60)
    
    # カスタムキャッシュディレクトリ
    custom_cache_dir = "/workspace/models"
    os.makedirs(custom_cache_dir, exist_ok=True)
    
    models = [
        {
            "name": "google/gemma-3-27b-it",
            "description": "Gemma 3 27Bモデル（4ビット量子化）",
            "size": "約14GB（量子化後）"
        },
        {
            "name": "google/gemma-3-12b-it", 
            "description": "Gemma 3 12Bモデル（4ビット量子化）",
            "size": "約6GB（量子化後）"
        }
    ]
    
    success_count = 0
    
    for model in models:
        print(f"\n📋 {model['description']}: {model['name']} ({model['size']})")
        
        if download_model_safely(model["name"], "auto", custom_cache_dir, use_4bit=True):
            success_count += 1
        else:
            print(f"  ⚠️  {model['name']} のダウンロードをスキップ")
    
    return success_count

def download_audio_models():
    """音声処理用モデルをダウンロード（不要のためスキップ）"""
    print("\n🎤 音声処理モデルのダウンロード")
    print("=" * 50)
    print("ℹ️  音声処理モデルは不要のためスキップします")
    
    return 0  # 音声モデルはダウンロードしない

def show_cache_info():
    """キャッシュ情報を表示"""
    print("\n📁 モデルキャッシュ情報")
    print("=" * 30)
    
    # デフォルトキャッシュ
    default_cache = os.path.expanduser("~/.cache/huggingface/hub")
    if os.path.exists(default_cache):
        print(f"デフォルトキャッシュ: {default_cache}")
    
    # カスタムキャッシュ
    custom_cache = "/workspace/models"
    if os.path.exists(custom_cache):
        print(f"カスタムキャッシュ: {custom_cache}")
        
        # モデル一覧を表示
        models = [d for d in os.listdir(custom_cache) if d.startswith("models--")]
        if models:
            print("\nダウンロード済みモデル:")
            for model in sorted(models):
                model_name = model.replace("models--", "").replace("--", "/")
                print(f"  ✅ {model_name}")

def main():
    print("🚀 DiaROS音声対話システム - モデル事前ダウンロードツール")
    print("=" * 60)
    print("Gemma 3 モデル（27B/12B）を4ビット量子化でダウンロードし、")
    print("システム起動時間を短縮します。")
    print("")
    
    # HuggingFaceトークンの確認
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
        print("設定後、このスクリプトを再実行してください。")
        return False
    
    print("✅ HuggingFaceトークンを検出しました")
    
    # ディスク容量チェック（4ビット量子化モデル用）
    if not check_disk_space(20):
        print("\nディスク容量を確保してから再実行してください。")
        return False
    
    # GPU情報表示
    if torch.cuda.is_available():
        print(f"✅ CUDA対応GPU検出: {torch.cuda.get_device_name()}")
    else:
        print("ℹ️  CPU環境で実行します")
    
    print("\n⏰ ダウンロード開始...")
    start_time = time.time()
    
    # NLGモデルダウンロード
    nlg_success = download_nlg_models()
    
    # 音声処理モデルダウンロード
    audio_success = download_audio_models()
    
    total_time = time.time() - start_time
    
    # 結果表示
    print("\n" + "=" * 60)
    print("📊 ダウンロード結果")
    print("=" * 60)
    print(f"Gemma 3 モデル（4ビット量子化）: {nlg_success}/2 成功")
    print(f"総所要時間: {total_time:.1f}秒")
    
    # キャッシュ情報表示
    show_cache_info()
    
    if nlg_success > 0:
        print("\n✅ モデルダウンロード完了！")
        print("\nこれで音声対話システムを高速起動できます：")
        print("  ./scripts/launch/launch_diaros_local.sh")
        print("  ./scripts/launch/launch_diaros_chatgpt.sh")
        return True
    else:
        print("\n❌ 必要なモデルのダウンロードに失敗しました")
        print("上記のエラーメッセージを確認してください")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)