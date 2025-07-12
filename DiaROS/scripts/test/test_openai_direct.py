#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_openai_direct.py - OpenAI APIの直接テスト（最小限の依存関係）
"""

import os
import sys
import time

api_key = os.environ.get("OPENAI_API_KEY", "")

print("=" * 50)
print("OpenAI API 直接テスト")
print("=" * 50)

if not api_key:
    print("❌ OPENAI_API_KEY が設定されていません")
    sys.exit(1)

print(f"✅ APIキー検出: {api_key[:20]}...")

try:
    import requests
    
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }
    
    data = {
        "model": "gpt-3.5-turbo",
        "messages": [
            {"role": "user", "content": "明日の天気は？"}
        ],
        "max_tokens": 100,
        "temperature": 0.7
    }
    
    print("\n🧪 API呼び出し中...")
    start_time = time.time()
    
    response = requests.post(
        "https://api.openai.com/v1/chat/completions",
        headers=headers,
        json=data
    )
    
    elapsed = (time.time() - start_time) * 1000
    
    if response.status_code == 200:
        result = response.json()
        content = result['choices'][0]['message']['content']
        print(f"✅ 応答成功: {content}")
        print(f"⏱️  応答時間: {elapsed:.0f}ms")
        
        if elapsed < 1500:
            print("🚀 高速応答 - DiaROSでの使用に適しています")
    else:
        print(f"❌ APIエラー: {response.status_code}")
        print(f"詳細: {response.text}")
        
except Exception as e:
    print(f"❌ エラー: {e}")
    import traceback
    traceback.print_exc()