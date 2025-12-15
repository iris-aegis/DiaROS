#!/usr/bin/env python3
"""
Gemma3-12Bモデルでの並列リクエストテスト
DiaROSのNLGクラスを直接使用して、実際の性能を測定
"""

import sys
import os
import time
import threading
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed

# DiaROSのパスを追加
sys.path.append('/workspace/DiaROS_py')
from diaros.naturalLanguageGeneration import NaturalLanguageGeneration

def test_nlg_parallel_requests():
    """NLGクラスでの並列リクエストテスト"""
    print("🚀 Gemma3-12B並列リクエストテスト開始")
    print("="*60)
    
    # NLGクラスのインスタンス作成
    try:
        nlg = NaturalLanguageGeneration()
        print("✅ NLGクラス初期化完了")
        
        # 少し待機してOllamaモデルのロードを待つ
        time.sleep(2)
        
    except Exception as e:
        print(f"❌ NLGクラス初期化エラー: {e}")
        return
    
    # テスト用の音声認識結果リスト
    test_asr_lists = [
        ["今日はいい天気ですね", "気持ちいい日です"],
        ["お疲れ様でした", "一日お疲れ様"],
        ["ありがとうございます", "感謝します"],
        ["こんにちは", "こんにちわ"],
        ["元気ですか？", "調子はどうですか"],
        ["おはようございます", "朝ですね"],
        ["お疲れ様です", "働いてますね"],
        ["こんばんは", "夜になりました"]
    ]
    
    print(f"📋 テストケース数: {len(test_asr_lists)}")
    print("🔄 順次実行テスト開始...")
    
    # 1. 順次実行テスト
    sequential_start = time.time()
    sequential_results = []
    
    for i, asr_list in enumerate(test_asr_lists[:4]):  # 最初の4つを順次実行
        start_time = time.time()
        nlg.update(asr_list)
        
        # 結果を待機（最大5秒）
        wait_count = 0
        last_reply = ""
        while wait_count < 500:  # 5秒間待機
            if nlg.last_reply != last_reply and nlg.last_reply != "":
                last_reply = nlg.last_reply
                break
            time.sleep(0.01)
            wait_count += 1
        
        end_time = time.time()
        duration = (end_time - start_time) * 1000
        
        sequential_results.append({
            'asr_list': asr_list,
            'response': last_reply,
            'duration_ms': duration
        })
        
        print(f"  順次 {i+1}: {duration:.1f}ms - '{last_reply}'")
    
    sequential_end = time.time()
    sequential_total = (sequential_end - sequential_start) * 1000
    
    print(f"\n📊 順次実行結果:")
    print(f"  総時間: {sequential_total:.1f}ms")
    print(f"  平均時間: {sequential_total/4:.1f}ms")
    
    # 少し待機
    time.sleep(2)
    
    print("\n⚡ 並列実行テスト開始...")
    
    # 2. 並列実行テスト
    def send_parallel_request(asr_list, request_id):
        """並列リクエスト送信"""
        start_time = time.time()
        
        nlg.update(asr_list)
        
        # 結果を待機（最大5秒）
        wait_count = 0
        last_reply = ""
        while wait_count < 500:  # 5秒間待機
            if nlg.last_reply != last_reply and nlg.last_reply != "":
                last_reply = nlg.last_reply
                break
            time.sleep(0.01)
            wait_count += 1
        
        end_time = time.time()
        duration = (end_time - start_time) * 1000
        
        return {
            'request_id': request_id,
            'asr_list': asr_list,
            'response': last_reply,
            'duration_ms': duration,
            'start_time': start_time,
            'end_time': end_time
        }
    
    parallel_start = time.time()
    
    # 4つのリクエストを並列送信
    with ThreadPoolExecutor(max_workers=4) as executor:
        futures = []
        for i, asr_list in enumerate(test_asr_lists[4:8]):  # 次の4つを並列実行
            future = executor.submit(send_parallel_request, asr_list, i+1)
            futures.append(future)
            time.sleep(0.1)  # 100ms間隔で送信
        
        parallel_results = []
        for future in as_completed(futures):
            result = future.result()
            parallel_results.append(result)
    
    parallel_end = time.time()
    parallel_total = (parallel_end - parallel_start) * 1000
    
    # 並列結果をリクエストID順にソート
    parallel_results.sort(key=lambda x: x['request_id'])
    
    print(f"\n📊 並列実行結果:")
    print(f"  総時間: {parallel_total:.1f}ms")
    print(f"  リクエスト数: {len(parallel_results)}")
    
    for result in parallel_results:
        print(f"  並列 {result['request_id']}: {result['duration_ms']:.1f}ms - '{result['response']}'")
    
    # 重複実行の確認
    overlapping_count = 0
    sorted_results = sorted(parallel_results, key=lambda x: x['start_time'])
    
    for i in range(len(sorted_results) - 1):
        current = sorted_results[i]
        next_req = sorted_results[i + 1]
        
        if current['end_time'] > next_req['start_time']:
            overlapping_count += 1
    
    print(f"\n🔄 並列実行分析:")
    print(f"  重複実行数: {overlapping_count}")
    print(f"  並列実行率: {overlapping_count/(len(sorted_results)-1)*100:.1f}%")
    
    # 効率性評価
    expected_parallel_time = sequential_total / 4 * len(parallel_results)
    efficiency = (expected_parallel_time / parallel_total) * 100
    
    print(f"\n🎯 効率性評価:")
    print(f"  理論的順次実行時間: {expected_parallel_time:.1f}ms")
    print(f"  実際の並列実行時間: {parallel_total:.1f}ms")
    print(f"  並列化効率: {efficiency:.1f}%")
    
    if efficiency > 200:
        print("✅ 優秀: Gemma3-12Bは並列リクエストを効率的に処理しています")
    elif efficiency > 120:
        print("🟡 良好: 並列処理の効果はありますが、改善の余地があります")
    else:
        print("❌ 問題: 並列処理の効果が限定的です")
    
    print("\n🏁 テスト完了")

if __name__ == "__main__":
    test_nlg_parallel_requests()