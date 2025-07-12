#!/usr/bin/env python3
"""
measure_e2e_latency.py - DiaROSエンドツーエンド遅延測定ツール
音声認識から音声合成完了までの遅延をリアルタイムで測定・統計化します。
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Iasr, Iss
import time
from datetime import datetime
import sys
import statistics
from collections import deque

class E2ELatencyMeasurer(Node):
    def __init__(self):
        super().__init__('e2e_latency_measurer')
        
        # 遅延測定データ
        self.asr_timestamps = {}  # ASR結果のタイムスタンプ
        self.latencies = deque(maxlen=100)  # 最新100個の遅延データを保持
        
        # 統計情報
        self.total_measurements = 0
        self.min_latency = float('inf')
        self.max_latency = 0
        
        # サブスクライバーの設定
        self.sub_asr = self.create_subscription(
            Iasr, 'ASRtoNLU', 
            self.asr_callback, 10)
        
        self.sub_ss = self.create_subscription(
            Iss, 'SStoDM', 
            self.ss_callback, 10)
        
        # 定期的な統計表示用タイマー
        self.stats_timer = self.create_timer(2.0, self.display_stats)
        
        print("\n" + "="*70)
        print("DiaROS エンドツーエンド遅延測定ツール")
        print("="*70)
        print("音声認識(ASR) → 音声合成(SS)の遅延をリアルタイム測定中...")
        print("Ctrl+Cで終了")
        print("-"*70)
        print("時刻      | ASR結果                | 遅延[ms] | 平均[ms] | 統計")
        print("-"*70)
        
    def asr_callback(self, msg):
        """音声認識結果を受信"""
        if msg.you and msg.is_final:
            # is_finalがTrueの場合のみ測定対象とする
            timestamp = time.time()
            # ASR結果をキーとしてタイムスタンプを保存
            self.asr_timestamps[msg.you] = timestamp
            
    def ss_callback(self, msg):
        """音声合成結果を受信"""
        if msg.filename:
            ss_timestamp = time.time()
            
            # 最も最近のASR結果との遅延を計算
            if self.asr_timestamps:
                # 最新のASRタイムスタンプを取得
                latest_asr_text = max(self.asr_timestamps.keys(), 
                                    key=lambda k: self.asr_timestamps[k])
                asr_timestamp = self.asr_timestamps[latest_asr_text]
                
                # 遅延計算 (ミリ秒)
                latency_ms = (ss_timestamp - asr_timestamp) * 1000
                
                # 統計データ更新
                self.latencies.append(latency_ms)
                self.total_measurements += 1
                self.min_latency = min(self.min_latency, latency_ms)
                self.max_latency = max(self.max_latency, latency_ms)
                
                # リアルタイム表示
                current_time = datetime.now().strftime("%H:%M:%S")
                avg_latency = statistics.mean(self.latencies) if self.latencies else 0
                
                # ASR結果を20文字に制限
                asr_display = (latest_asr_text[:17] + "...") if len(latest_asr_text) > 20 else latest_asr_text
                
                print(f"{current_time} | {asr_display:<20} | {latency_ms:6.0f}ms | {avg_latency:6.0f}ms | n={self.total_measurements}")
                
                # 使用済みのASRタイムスタンプを削除（メモリリーク防止）
                del self.asr_timestamps[latest_asr_text]
                
                # 古いASRタイムスタンプをクリーンアップ（10秒以上古いもの）
                current_time = time.time()
                old_keys = [k for k, v in self.asr_timestamps.items() 
                           if current_time - v > 10.0]
                for key in old_keys:
                    del self.asr_timestamps[key]
    
    def display_stats(self):
        """定期的な統計表示"""
        if self.total_measurements == 0:
            return
            
        # 統計計算
        avg_latency = statistics.mean(self.latencies)
        median_latency = statistics.median(self.latencies)
        std_latency = statistics.stdev(self.latencies) if len(self.latencies) > 1 else 0
        
        # P95パーセンタイル（上位5%）
        sorted_latencies = sorted(self.latencies)
        p95_index = int(len(sorted_latencies) * 0.95)
        p95_latency = sorted_latencies[p95_index] if sorted_latencies else 0
        
        # 統計情報を画面下部に表示
        print("\n" + "="*70)
        print(f"【統計情報】測定回数: {self.total_measurements}")
        print(f"平均遅延: {avg_latency:.0f}ms | 中央値: {median_latency:.0f}ms | 標準偏差: {std_latency:.0f}ms")
        print(f"最小遅延: {self.min_latency:.0f}ms | 最大遅延: {self.max_latency:.0f}ms | P95: {p95_latency:.0f}ms")
        
        # 遅延評価
        if avg_latency < 1000:
            evaluation = "🟢 優秀"
        elif avg_latency < 2000:
            evaluation = "🟡 良好"
        elif avg_latency < 3000:
            evaluation = "🟠 注意"
        else:
            evaluation = "🔴 改善要"
            
        print(f"遅延評価: {evaluation} (目標: <1000ms, 許容: <2000ms)")
        print("="*70)

def main():
    rclpy.init()
    measurer = E2ELatencyMeasurer()
    
    try:
        rclpy.spin(measurer)
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print("遅延測定を終了しました。")
        if measurer.total_measurements > 0:
            print(f"総測定回数: {measurer.total_measurements}")
            avg = statistics.mean(measurer.latencies)
            print(f"平均遅延: {avg:.0f}ms")
            print(f"遅延範囲: {measurer.min_latency:.0f}ms - {measurer.max_latency:.0f}ms")
        else:
            print("測定データがありません。DiaROSが正常に動作していることを確認してください。")
        print("="*70)
    finally:
        measurer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()