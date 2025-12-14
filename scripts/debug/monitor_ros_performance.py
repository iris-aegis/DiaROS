#!/usr/bin/env python3
"""
ROS通信とQueue性能監視スクリプト
大量の音声認識履歴送信によるボトルネックを検証
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Idm, Inlg, Iasr, Itt, Ibc
import time
import threading
import queue
import statistics
from datetime import datetime, timedelta
import sys

class ROSPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('ros_performance_monitor')
        
        # メッセージ受信統計
        self.dm_stats = {
            'count': 0,
            'total_words': 0,
            'max_words': 0,
            'timestamps': [],
            'word_counts': [],
            'intervals': []
        }
        
        self.nlg_stats = {
            'count': 0,
            'timestamps': [],
            'intervals': []
        }
        
        self.asr_stats = {
            'count': 0,
            'timestamps': [],
            'intervals': []
        }
        
        # Queue詰まり検出用
        self.queue_warning_threshold = 1.0  # 1秒以上の間隔で警告
        self.large_message_threshold = 50   # 50個以上の履歴で大容量メッセージ判定
        
        # Subscription設定（queue_size=1で詰まりやすくして検証）
        self.dm_sub = self.create_subscription(
            Idm, 'DMtoNLG', self.dm_callback, 1
        )
        self.nlg_sub = self.create_subscription(
            Inlg, 'NLGtoSS', self.nlg_callback, 1
        )
        self.asr_sub = self.create_subscription(
            Iasr, 'NLUtoDM', self.asr_callback, 1
        )
        
        # 統計表示タイマー
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        # ログファイル
        self.log_file = open(f'/tmp/ros_performance_{int(time.time())}.log', 'w')
        self.log_file.write("timestamp,topic,message_size,interval_ms,queue_warning\\n")
        
        self.get_logger().info("🔍 ROS通信性能監視を開始しました")
        print("=" * 60)
        print("🔍 ROS通信とQueue性能監視")
        print("=" * 60)
        print("監視対象:")
        print("  - DMtoNLG: 音声認識履歴送信")
        print("  - NLGtoSS: 対話生成結果")
        print("  - NLUtoDM: 音声認識結果")
        print(f"  - 大容量メッセージ閾値: {self.large_message_threshold}個")
        print(f"  - Queue詰まり警告閾値: {self.queue_warning_threshold}秒")
        print("=" * 60)

    def dm_callback(self, msg):
        """DMtoNLG (DialogManagement → NaturalLanguageGeneration)"""
        now = time.time()
        word_count = len(msg.words)
        
        # 統計更新
        self.dm_stats['count'] += 1
        self.dm_stats['total_words'] += word_count
        self.dm_stats['max_words'] = max(self.dm_stats['max_words'], word_count)
        
        # 間隔計算
        if self.dm_stats['timestamps']:
            interval = now - self.dm_stats['timestamps'][-1]
            self.dm_stats['intervals'].append(interval)
            
            # Queue詰まり警告
            if interval > self.queue_warning_threshold:
                warning_msg = f"⚠️  [DMtoNLG] Queue詰まりの可能性: {interval:.2f}秒間隔"
                print(warning_msg)
                self.get_logger().warn(warning_msg)
        
        self.dm_stats['timestamps'].append(now)
        self.dm_stats['word_counts'].append(word_count)
        
        # 大容量メッセージ検出
        if word_count >= self.large_message_threshold:
            large_msg = f"📦 [DMtoNLG] 大容量メッセージ: {word_count}個の履歴"
            print(large_msg)
            self.get_logger().info(large_msg)
        
        # ログ出力
        interval_ms = self.dm_stats['intervals'][-1] * 1000 if self.dm_stats['intervals'] else 0
        queue_warning = interval_ms > (self.queue_warning_threshold * 1000)
        self.log_file.write(f"{now},DMtoNLG,{word_count},{interval_ms:.1f},{queue_warning}\\n")
        self.log_file.flush()

    def nlg_callback(self, msg):
        """NLGtoSS (NaturalLanguageGeneration → SpeechSynthesis)"""
        now = time.time()
        
        self.nlg_stats['count'] += 1
        
        if self.nlg_stats['timestamps']:
            interval = now - self.nlg_stats['timestamps'][-1]
            self.nlg_stats['intervals'].append(interval)
        
        self.nlg_stats['timestamps'].append(now)
        
        # 応答生成通知
        print(f"🎯 [NLGtoSS] 対話生成完了: '{msg.reply[:50]}{'...' if len(msg.reply) > 50 else ''}'")
        
        # ログ出力
        interval_ms = self.nlg_stats['intervals'][-1] * 1000 if self.nlg_stats['intervals'] else 0
        self.log_file.write(f"{now},NLGtoSS,{len(msg.reply)},{interval_ms:.1f},False\\n")
        self.log_file.flush()

    def asr_callback(self, msg):
        """NLUtoDM (ASR → DialogManagement)"""
        now = time.time()
        
        self.asr_stats['count'] += 1
        
        if self.asr_stats['timestamps']:
            interval = now - self.asr_stats['timestamps'][-1]
            self.asr_stats['intervals'].append(interval)
        
        self.asr_stats['timestamps'].append(now)
        
        # ログ出力のみ（詳細出力は控える）
        interval_ms = self.asr_stats['intervals'][-1] * 1000 if self.asr_stats['intervals'] else 0
        self.log_file.write(f"{now},NLUtoDM,{len(msg.you)},{interval_ms:.1f},False\\n")
        self.log_file.flush()

    def print_stats(self):
        """統計情報の定期表示"""
        print("\\n" + "=" * 60)
        print(f"📊 性能統計 - {datetime.now().strftime('%H:%M:%S')}")
        print("=" * 60)
        
        # DMtoNLG統計
        if self.dm_stats['count'] > 0:
            avg_words = self.dm_stats['total_words'] / self.dm_stats['count']
            avg_interval = statistics.mean(self.dm_stats['intervals']) if self.dm_stats['intervals'] else 0
            max_interval = max(self.dm_stats['intervals']) if self.dm_stats['intervals'] else 0
            
            print(f"🔄 DMtoNLG (音声認識履歴送信):")
            print(f"  メッセージ数: {self.dm_stats['count']}")
            print(f"  平均履歴数: {avg_words:.1f}個")
            print(f"  最大履歴数: {self.dm_stats['max_words']}個")
            print(f"  平均送信間隔: {avg_interval*1000:.1f}ms")
            print(f"  最大送信間隔: {max_interval*1000:.1f}ms")
            if max_interval > self.queue_warning_threshold:
                print(f"  ⚠️  Queue詰まりリスク: 最大間隔{max_interval:.2f}秒")
        
        # NLGtoSS統計
        if self.nlg_stats['count'] > 0:
            avg_interval = statistics.mean(self.nlg_stats['intervals']) if self.nlg_stats['intervals'] else 0
            print(f"\\n🎯 NLGtoSS (対話生成結果):")
            print(f"  メッセージ数: {self.nlg_stats['count']}")
            print(f"  平均生成間隔: {avg_interval*1000:.1f}ms")
        
        # ASR統計
        if self.asr_stats['count'] > 0:
            avg_interval = statistics.mean(self.asr_stats['intervals']) if self.asr_stats['intervals'] else 0
            print(f"\\n🎤 NLUtoDM (音声認識結果):")
            print(f"  メッセージ数: {self.asr_stats['count']}")
            print(f"  平均認識間隔: {avg_interval*1000:.1f}ms")
        
        print("=" * 60)

    def __del__(self):
        if hasattr(self, 'log_file'):
            self.log_file.close()

def main():
    rclpy.init()
    monitor = ROSPerformanceMonitor()
    
    try:
        print("\\n🚀 監視開始 - Ctrl+Cで終了")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\\n📊 監視終了")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()