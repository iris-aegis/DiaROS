#!/usr/bin/env python3
"""
DiaROS統合タイムスタンプシステムのテストスクリプト
"""

import sys
import time
import json
from datetime import datetime

# パスを追加
sys.path.insert(0, '/workspace/DiaROS_py')

from diaros.timeTracker import get_time_tracker

def test_timing_system():
    """タイミングシステムのテスト"""
    print("🧪 DiaROS統合タイムスタンプシステムのテスト開始")
    
    # タイムトラッカーを初期化
    tracker = get_time_tracker("test_pc")
    
    # セッションを作成
    session_id = tracker.create_session("test_dialogue")
    print(f"✅ セッション作成: {session_id}")
    
    # 各種チェックポイントを追加
    tracker.add_checkpoint(session_id, "nlg", "processing_start", {
        "test_data": "音声認識結果",
        "timestamp": datetime.now().isoformat()
    })
    
    # 少し待機
    time.sleep(0.1)
    
    tracker.add_checkpoint(session_id, "nlg", "llm_start", {
        "model": "gemma3:27b",
        "prompt_type": "test"
    })
    
    # 少し待機
    time.sleep(0.2)
    
    tracker.add_checkpoint(session_id, "nlg", "llm_complete", {
        "model": "gemma3:27b",
        "response": "テスト応答",
        "duration_ms": 200
    })
    
    tracker.add_checkpoint(session_id, "nlg", "processing_complete", {
        "final_response": "テスト応答",
        "total_duration_ms": 300
    })
    
    # セッションデータを取得
    session_data = tracker.get_session_data(session_id)
    print(f"✅ セッションデータ取得: {len(session_data['checkpoints'])} checkpoints")
    
    # データを表示
    print("\n📊 チェックポイント詳細:")
    for i, checkpoint in enumerate(session_data['checkpoints']):
        timestamp = datetime.fromtimestamp(checkpoint['timestamp_ns'] / 1_000_000_000)
        print(f"  {i+1}. {checkpoint['component']}.{checkpoint['event']} @ {timestamp.strftime('%H:%M:%S.%f')[:-3]}")
    
    # ファイルに保存
    tracker.save_session(session_id)
    print(f"✅ セッションデータ保存: /tmp/diaros_timing_test_pc.json")
    
    # ファイル内容を確認
    try:
        with open('/tmp/diaros_timing_test_pc.json', 'r') as f:
            lines = f.readlines()
        print(f"✅ 保存されたデータ: {len(lines)} sessions")
        
        # 最後の行をパース
        if lines:
            last_session = json.loads(lines[-1])
            print(f"   最新セッション: {last_session['session_id'][:8]}...")
    except Exception as e:
        print(f"❌ ファイル読み込みエラー: {e}")
    
    # クリーンアップ
    tracker.cleanup_session(session_id)
    print("✅ セッションクリーンアップ完了")
    
    print("\n🎉 タイミングシステムテスト完了!")

if __name__ == "__main__":
    test_timing_system()