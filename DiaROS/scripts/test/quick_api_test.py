#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
quick_api_test.py - シンプルなOpenAI APIテスト
"""

import os
import sys
import time

def test_openai_api():
    """OpenAI APIの簡単なテスト"""
    api_key = os.environ.get("OPENAI_API_KEY", "")
    
    print("=" * 50)
    print("OpenAI API クイックテスト")
    print("=" * 50)
    
    if not api_key:
        print("❌ OPENAI_API_KEY が設定されていません")
        print("\n以下のコマンドでAPIキーを設定してください:")
        print('export OPENAI_API_KEY="sk-your-api-key"')
        return False
    
    print(f"✅ APIキー検出: {api_key[:7]}...")
    
    try:
        import openai
        print("✅ openaiモジュール: インポート成功")
    except ImportError:
        print("❌ openaiモジュールが見つかりません")
        print("pip install openai を実行してください")
        return False
    
    # APIテスト
    print("\n🧪 API接続テスト中...")
    try:
        from openai import OpenAI
        client = OpenAI(api_key=api_key)
        
        start_time = time.time()
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "user", "content": "こんにちは"}
            ],
            max_tokens=50,
            temperature=0.7
        )
        elapsed = (time.time() - start_time) * 1000
        
        result = response.choices[0].message.content
        print(f"✅ API応答成功: {result}")
        print(f"⏱️  応答時間: {elapsed:.0f}ms")
        
        if elapsed < 1500:
            print("🚀 高速応答 - 対話に適しています")
        else:
            print("⚠️  応答が遅い - ネットワークを確認してください")
            
        return True
        
    except Exception as e:
        print(f"❌ APIエラー: {e}")
        print("\nAPIキーが正しいか確認してください")
        return False

if __name__ == "__main__":
    success = test_openai_api()
    sys.exit(0 if success else 1)