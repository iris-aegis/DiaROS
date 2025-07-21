#!/usr/bin/env python3
"""
直接OllamaAPIでGPU使用を確認
"""

import requests
import time
import json

def test_direct_ollama():
    print("🔧 直接Ollama API テスト")
    print("="*50)
    
    # 現在ロードされているモデルを確認
    ps_response = requests.get("http://localhost:11434/api/ps")
    print("📋 ロード済みモデル:")
    print(json.dumps(ps_response.json(), indent=2))
    
    # シンプルなプロンプトでテスト
    simple_payload = {
        "model": "gemma3:12b",
        "prompt": "こんにちは",
        "stream": False,
        "options": {
            "num_gpu": -1,  # 全GPUレイヤー
        }
    }
    
    print("\n🚀 シンプルプロンプトテスト:")
    start_time = time.time()
    response = requests.post("http://localhost:11434/api/generate", json=simple_payload)
    end_time = time.time()
    
    result = response.json()
    duration = (end_time - start_time) * 1000
    
    print(f"応答時間: {duration:.1f}ms")
    print(f"生成時間: {result.get('total_duration', 0) / 1000000:.1f}ms")
    print(f"応答: {result.get('response', '')}")
    
    # 複雑なプロンプトでテスト
    complex_payload = {
        "model": "gemma3:12b",
        "prompt": """音声認識結果から自然な発話を推定し、親しみやすく応答してください。

ルール:
- [雑音][無音]は無視
- 重複や欠落を自然に補正
- 20文字程度で友達口調で応答

認識結果1: 今日はいい天気ですね
認識結果2: [雑音]気持ちいいです

応答:""",
        "stream": False,
        "options": {
            "num_gpu": -1,
        }
    }
    
    print("\n🔄 複雑プロンプトテスト:")
    start_time = time.time()
    response = requests.post("http://localhost:11434/api/generate", json=complex_payload)
    end_time = time.time()
    
    result = response.json()
    duration = (end_time - start_time) * 1000
    
    print(f"応答時間: {duration:.1f}ms")
    print(f"生成時間: {result.get('total_duration', 0) / 1000000:.1f}ms")
    print(f"応答: {result.get('response', '')}")

if __name__ == "__main__":
    test_direct_ollama()