#!/usr/bin/env python3
"""
ROS2 bag記録用フィールドのテストスクリプト
更新されたInlgメッセージの新しいフィールドをテスト
"""

import sys
import os
import time
import threading
from datetime import datetime

# DiaROSモジュールのパスを追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../DiaROS_py'))

from diaros.naturalLanguageGeneration import NaturalLanguageGeneration

def test_bag_fields():
    print("🔧 ROS2 bag記録用フィールドテスト開始")
    print("=" * 50)
    
    # NLGインスタンス作成
    nlg = NaturalLanguageGeneration()
    
    # バックグラウンドでrun()実行
    def run_nlg():
        nlg.run()
    
    thread = threading.Thread(target=run_nlg, daemon=True)
    thread.start()
    
    print("📋 ワーカー情報と時刻記録のテスト")
    
    # テストクエリ送信
    test_query = ['こんにちは', 'お元気ですか']
    print(f"\n📤 テストクエリ送信: {test_query}")
    nlg.update(test_query)
    
    # 結果待機
    print("\n⏳ 処理完了待機中...")
    time.sleep(3.0)
    
    # bag記録用フィールドを確認
    print("\n📊 ROS2 bag記録用フィールド:")
    print(f"  🆔 Request ID: {nlg.last_request_id}")
    print(f"  👷 Worker Name: {nlg.last_worker_name}")
    print(f"  ⏰ Start Timestamp (ns): {nlg.last_start_timestamp_ns}")
    print(f"  🏁 Completion Timestamp (ns): {nlg.last_completion_timestamp_ns}")
    print(f"  ⚡ Inference Duration (ms): {nlg.last_inference_duration_ms:.1f}")
    
    # 時刻をナノ秒から可読形式に変換
    if nlg.last_start_timestamp_ns > 0:
        start_time = datetime.fromtimestamp(nlg.last_start_timestamp_ns / 1_000_000_000)
        completion_time = datetime.fromtimestamp(nlg.last_completion_timestamp_ns / 1_000_000_000)
        print(f"\n🕐 開始時刻: {start_time.strftime('%H:%M:%S.%f')[:-3]}")
        print(f"🕕 完了時刻: {completion_time.strftime('%H:%M:%S.%f')[:-3]}")
        duration = (nlg.last_completion_timestamp_ns - nlg.last_start_timestamp_ns) / 1_000_000
        print(f"📏 実際の処理時間: {duration:.1f}ms")
    
    print(f"\n✅ 最新の応答: {nlg.last_reply}")
    print(f"📝 使用した音声認識結果: {nlg.last_source_words}")
    
    print("\n🏁 テスト完了")
    print("\n📦 ROS2 bag記録時に以下の情報が含まれます:")
    print("  - Request ID: リクエストの一意識別子")
    print("  - Worker Name: 処理したワーカースレッド名")
    print("  - Start/Completion Timestamp: 開始・完了時刻（ナノ秒精度）")
    print("  - Inference Duration: 推論時間（ミリ秒）")

if __name__ == "__main__":
    test_bag_fields()