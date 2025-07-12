#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_api_nlg.py - APIキーとNLG（応答生成）の動作テスト

このスクリプトは、DiaROSの応答生成機能が正しく動作するかをテストします。
APIキーの設定状況を確認し、実際に応答を生成してみます。
"""

import os
import sys
import time
from pathlib import Path

# DiaROSモジュールのパスを追加
diaros_path = Path(__file__).parent.parent.parent / "DiaROS_py"
sys.path.insert(0, str(diaros_path))

try:
    from diaros.naturalLanguageGeneration import NaturalLanguageGeneration
except ImportError as e:
    print(f"❌ モジュールのインポートエラー: {e}")
    print(f"   DiaROSパス: {diaros_path}")
    sys.exit(1)

def check_api_keys():
    """APIキーの設定状況を確認"""
    print("🔍 APIキー設定状況を確認中...")
    print("-" * 50)
    
    openai_key = os.environ.get("OPENAI_API_KEY", "")
    anthropic_key = os.environ.get("ANTHROPIC_API_KEY", "")
    
    if openai_key:
        print(f"✅ OPENAI_API_KEY: 設定済み ({openai_key[:7]}...)")
    else:
        print("❌ OPENAI_API_KEY: 未設定")
    
    if anthropic_key:
        print(f"✅ ANTHROPIC_API_KEY: 設定済み ({anthropic_key[:7]}...)")
    else:
        print("❌ ANTHROPIC_API_KEY: 未設定")
    
    print("-" * 50)
    return bool(openai_key or anthropic_key)

def test_response_generation():
    """応答生成のテスト"""
    print("\n🧪 応答生成テストを実行中...")
    print("-" * 50)
    
    # NLGモジュールを初期化
    nlg = NaturalLanguageGeneration()
    
    # テスト用の対話履歴
    test_histories = [
        ["明日の天気を教えてください"],
        ["今日は何曜日ですか"],
        ["おはようございます"],
    ]
    
    for i, history in enumerate(test_histories, 1):
        print(f"\n📝 テスト {i}: 入力「{history[0]}」")
        
        # 応答生成
        start_time = time.time()
        response = nlg.nlg(history)
        elapsed_time = (time.time() - start_time) * 1000
        
        if response:
            print(f"✅ 応答: {response}")
            print(f"⏱️  処理時間: {elapsed_time:.0f}ms")
            
            # APIの種類を判定
            if elapsed_time < 1500:
                print("🚀 高速API（ChatGPT/Claude）を使用中")
            else:
                print("🐌 ローカルモデルを使用中（APIキー未設定の可能性）")
        else:
            print("❌ 応答生成失敗")
    
    print("-" * 50)

def main():
    print("=" * 50)
    print("DiaROS 応答生成（NLG）テストツール")
    print("=" * 50)
    
    # APIキーの確認
    has_api_key = check_api_keys()
    
    if not has_api_key:
        print("\n⚠️  警告: APIキーが設定されていません")
        print("以下のコマンドでAPIキーを設定してください:")
        print("")
        print("export OPENAI_API_KEY=\"sk-your-openai-api-key\"")
        print("")
        print("または setup_api.sh を使用:")
        print("./scripts/setup/setup_api.sh")
        print("")
        print("ローカルモデルで動作しますが、品質と速度が劣ります。")
    
    # 応答生成テスト
    try:
        test_response_generation()
    except Exception as e:
        print(f"\n❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n✅ テスト完了")

if __name__ == "__main__":
    main()