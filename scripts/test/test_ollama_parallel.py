#!/usr/bin/env python3
"""
Ollama並列リクエスト処理テストプログラム
複数スレッドから同時にgemma3:27bモデルにリクエストを送信し、
Ollamaの並列処理機能を検証する
"""

import threading
import time
import json
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed
from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser
import statistics

class OllamaParallelTester:
    def __init__(self):
        self.model = ChatOllama(
            model="gemma3:27b",
            max_tokens=100,
            temperature=0.8,
            top_p=0.9
        )
        self.results = []
        self.lock = threading.Lock()
        
    def send_request(self, thread_id, request_text, request_id):
        """単一のリクエストを送信"""
        start_time = time.time()
        start_timestamp = datetime.now()
        
        try:
            # プロンプトの構築
            prompt = ChatPromptTemplate.from_messages([
                ("system", "あなたは親しみやすいアシスタントです。15文字以内で応答してください。"),
                ("human", request_text)
            ])
            
            chain = prompt | self.model | StrOutputParser()
            
            print(f"[{start_timestamp.strftime('%H:%M:%S.%f')[:-3]}] Thread-{thread_id} リクエスト{request_id}開始: '{request_text}'")
            
            # リクエスト実行
            response = chain.invoke({})
            
            end_time = time.time()
            end_timestamp = datetime.now()
            duration = (end_time - start_time) * 1000  # ミリ秒
            
            print(f"[{end_timestamp.strftime('%H:%M:%S.%f')[:-3]}] Thread-{thread_id} リクエスト{request_id}完了: '{response}' ({duration:.1f}ms)")
            
            # 結果を記録
            with self.lock:
                self.results.append({
                    'thread_id': thread_id,
                    'request_id': request_id,
                    'request_text': request_text,
                    'response': response,
                    'start_time': start_timestamp,
                    'end_time': end_timestamp,
                    'duration_ms': duration
                })
            
            return {
                'thread_id': thread_id,
                'request_id': request_id,
                'success': True,
                'response': response,
                'duration_ms': duration
            }
            
        except Exception as e:
            end_time = time.time()
            end_timestamp = datetime.now()
            duration = (end_time - start_time) * 1000
            
            print(f"[{end_timestamp.strftime('%H:%M:%S.%f')[:-3]}] Thread-{thread_id} リクエスト{request_id}エラー: {str(e)} ({duration:.1f}ms)")
            
            with self.lock:
                self.results.append({
                    'thread_id': thread_id,
                    'request_id': request_id,
                    'request_text': request_text,
                    'response': None,
                    'error': str(e),
                    'start_time': start_timestamp,
                    'end_time': end_timestamp,
                    'duration_ms': duration
                })
            
            return {
                'thread_id': thread_id,
                'request_id': request_id,
                'success': False,
                'error': str(e),
                'duration_ms': duration
            }

    def test_sequential_requests(self, num_requests=5):
        """シーケンシャルリクエストテスト"""
        print("\n" + "="*60)
        print("🔄 シーケンシャルリクエストテスト")
        print("="*60)
        
        test_phrases = [
            "今日はいい天気ですね",
            "お疲れ様でした",
            "ありがとうございます",
            "こんにちは",
            "元気ですか？"
        ]
        
        sequential_start = time.time()
        
        for i in range(num_requests):
            phrase = test_phrases[i % len(test_phrases)]
            self.send_request(thread_id=1, request_text=phrase, request_id=i+1)
        
        sequential_end = time.time()
        sequential_total = (sequential_end - sequential_start) * 1000
        
        print(f"\n📊 シーケンシャル実行結果:")
        print(f"  総実行時間: {sequential_total:.1f}ms")
        print(f"  平均応答時間: {sequential_total/num_requests:.1f}ms")
        
        return sequential_total

    def test_parallel_requests(self, num_threads=3, requests_per_thread=2):
        """並列リクエストテスト"""
        print("\n" + "="*60)
        print("⚡ 並列リクエストテスト")
        print("="*60)
        
        test_phrases = [
            "今日はいい天気ですね",
            "お疲れ様でした", 
            "ありがとうございます",
            "こんにちは",
            "元気ですか？",
            "おはようございます",
            "お疲れ様です",
            "こんばんは"
        ]
        
        parallel_start = time.time()
        
        # スレッドプールを使用して並列実行
        with ThreadPoolExecutor(max_workers=num_threads) as executor:
            futures = []
            
            for thread_id in range(1, num_threads + 1):
                for req_id in range(1, requests_per_thread + 1):
                    phrase = test_phrases[(thread_id * requests_per_thread + req_id - 1) % len(test_phrases)]
                    future = executor.submit(self.send_request, thread_id, phrase, req_id)
                    futures.append(future)
            
            # 全てのタスクの完了を待つ
            completed_futures = []
            for future in as_completed(futures):
                result = future.result()
                completed_futures.append(result)
        
        parallel_end = time.time()
        parallel_total = (parallel_end - parallel_start) * 1000
        
        print(f"\n📊 並列実行結果:")
        print(f"  総実行時間: {parallel_total:.1f}ms")
        print(f"  スレッド数: {num_threads}")
        print(f"  スレッド当たりリクエスト数: {requests_per_thread}")
        print(f"  総リクエスト数: {num_threads * requests_per_thread}")
        
        return parallel_total, completed_futures

    def analyze_results(self):
        """結果の詳細分析"""
        if not self.results:
            print("❌ 分析対象の結果がありません")
            return
        
        print("\n" + "="*60)
        print("📈 詳細分析結果")
        print("="*60)
        
        # 成功/失敗の統計
        successful_requests = [r for r in self.results if 'error' not in r]
        failed_requests = [r for r in self.results if 'error' in r]
        
        print(f"📊 実行統計:")
        print(f"  総リクエスト数: {len(self.results)}")
        print(f"  成功: {len(successful_requests)}")
        print(f"  失敗: {len(failed_requests)}")
        print(f"  成功率: {len(successful_requests)/len(self.results)*100:.1f}%")
        
        if successful_requests:
            # 応答時間統計
            durations = [r['duration_ms'] for r in successful_requests]
            
            print(f"\n⏱️  応答時間統計:")
            print(f"  平均: {statistics.mean(durations):.1f}ms")
            print(f"  中央値: {statistics.median(durations):.1f}ms")
            print(f"  最小: {min(durations):.1f}ms")
            print(f"  最大: {max(durations):.1f}ms")
            print(f"  標準偏差: {statistics.stdev(durations):.1f}ms")
            
            # スレッド別統計
            thread_stats = {}
            for result in successful_requests:
                thread_id = result['thread_id']
                if thread_id not in thread_stats:
                    thread_stats[thread_id] = []
                thread_stats[thread_id].append(result['duration_ms'])
            
            print(f"\n🧵 スレッド別統計:")
            for thread_id, durations in thread_stats.items():
                print(f"  Thread-{thread_id}: 平均{statistics.mean(durations):.1f}ms, リクエスト数{len(durations)}")
            
            # 時系列での重複確認
            print(f"\n🔄 並列実行の重複確認:")
            sorted_results = sorted(successful_requests, key=lambda x: x['start_time'])
            
            overlapping_count = 0
            for i in range(len(sorted_results) - 1):
                current = sorted_results[i]
                next_req = sorted_results[i + 1]
                
                if current['end_time'] > next_req['start_time']:
                    overlapping_count += 1
                    print(f"  重複検出: Thread-{current['thread_id']} と Thread-{next_req['thread_id']} が重複実行")
            
            print(f"  重複実行数: {overlapping_count}")
            print(f"  並列実行率: {overlapping_count/(len(sorted_results)-1)*100:.1f}%")
        
        # 失敗したリクエストの詳細
        if failed_requests:
            print(f"\n❌ 失敗したリクエスト:")
            for req in failed_requests:
                print(f"  Thread-{req['thread_id']} Request-{req['request_id']}: {req['error']}")

def main():
    print("🚀 Ollama並列リクエスト処理テスト開始")
    print("="*60)
    
    # テスト環境確認
    try:
        tester = OllamaParallelTester()
        print("✅ Ollama ChatOllama接続確認成功")
    except Exception as e:
        print(f"❌ Ollama接続エラー: {e}")
        print("以下を確認してください:")
        print("1. Ollamaが起動していること: ollama serve")
        print("2. gemma3:27bモデルがロードされていること: ollama run gemma3:27b")
        return
    
    # 1. シーケンシャルリクエストテスト
    sequential_time = tester.test_sequential_requests(num_requests=3)
    
    # 結果をクリア
    tester.results.clear()
    
    # 2. 並列リクエストテスト
    parallel_time, parallel_results = tester.test_parallel_requests(num_threads=3, requests_per_thread=2)
    
    # 3. 結果分析
    tester.analyze_results()
    
    # 4. 並列処理効果の評価
    print("\n" + "="*60)
    print("🎯 並列処理効果評価")
    print("="*60)
    
    expected_parallel_time = sequential_time * 2  # 6リクエスト分
    efficiency = (expected_parallel_time / parallel_time) * 100
    
    print(f"📊 効率性評価:")
    print(f"  シーケンシャル時間（3リクエスト）: {sequential_time:.1f}ms")
    print(f"  並列実行時間（6リクエスト）: {parallel_time:.1f}ms")
    print(f"  理論的シーケンシャル時間（6リクエスト）: {expected_parallel_time:.1f}ms")
    print(f"  並列化効率: {efficiency:.1f}%")
    
    if efficiency > 150:
        print("✅ 優秀: Ollamaは並列リクエストを効率的に処理しています")
    elif efficiency > 100:
        print("🟡 良好: Ollamaは並列リクエストを処理できますが、効率は限定的です")
    else:
        print("❌ 問題: 並列処理の効果が見られません")
    
    print("\n🏁 テスト完了")

if __name__ == "__main__":
    main()