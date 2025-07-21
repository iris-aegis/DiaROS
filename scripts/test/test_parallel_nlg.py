#!/usr/bin/env python3
"""
並列処理NLGのテストスクリプト
160ms間隔でのリクエスト送信をテスト
"""

import sys
import os
import time
import threading
from datetime import datetime

# DiaROSモジュールのパスを追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../DiaROS_py'))

from diaros.naturalLanguageGeneration import NaturalLanguageGeneration

def test_parallel_nlg():
    print("🔧 並列処理NLGテスト開始")
    print("=" * 50)
    
    # NLGインスタンス作成
    nlg = NaturalLanguageGeneration()
    
    # バックグラウンドでrun()実行
    def run_nlg():
        nlg.run()
    
    thread = threading.Thread(target=run_nlg, daemon=True)
    thread.start()
    
    print("📋 160ms間隔での並列リクエストテスト")
    
    # テストクエリリスト
    test_queries = [
        ['今日はいい天気ですね', '[雑音]気持ちいいです'],
        ['お疲れ様です', 'プロジェクトの調子はどうですか'],
        ['ありがとうございます', '手伝ってもらえますか'],
        ['こんにちは', 'お元気ですか'],
        ['今度会いましょう', '[雑音]楽しみにしています']
    ]
    
    # 161ms間隔でリクエスト送信
    for i, query in enumerate(test_queries):
        print(f"\n📤 リクエスト {i+1}: {query}")
        nlg.update(query)
        
        if i < len(test_queries) - 1:
            time.sleep(0.161)  # 161ms待機（並列処理を発生させる）
    
    # 結果待機
    print("\n⏳ 処理完了待機中...")
    time.sleep(3.0)
    
    # 最終結果表示
    print(f"\n✅ 最新の応答: {nlg.last_reply}")
    print(f"📝 使用した音声認識結果: {nlg.last_source_words}")
    
    print("\n🏁 テスト完了")

if __name__ == "__main__":
    test_parallel_nlg()