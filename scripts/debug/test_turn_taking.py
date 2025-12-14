#!/usr/bin/env python3
"""
test_turn_taking.py - ターンテイキング判定テストツール
TurnTakingノードの動作を確認し、confidence値を監視します。
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Itt
import time
from datetime import datetime

class TurnTakingTester(Node):
    def __init__(self):
        super().__init__('turn_taking_tester')
        
        # サブスクライバー（TTからの判定結果を監視）
        self.sub_tt = self.create_subscription(
            Itt, 'TTtoDM', 
            self.tt_callback, 10)
        
        # 統計情報
        self.tt_count = 0
        self.high_confidence_count = 0
        self.confidence_values = []
        self.last_tt_time = None
        
        print("\n" + "="*60)
        print("ターンテイキング判定モニター")
        print("="*60)
        print("TTtoDMトピックを監視しています...")
        print("閾値: 0.75以上で応答生成トリガー")
        print("Ctrl+Cで終了")
        print("-"*60 + "\n")
        
    def tt_callback(self, msg):
        """TTからの判定結果を受信"""
        self.tt_count += 1
        confidence = float(msg.confidence)
        self.confidence_values.append(confidence)
        
        # バーグラフで可視化
        bar_length = 50
        filled = int(confidence * bar_length)
        threshold_pos = int(0.75 * bar_length)
        
        bar = ""
        for i in range(bar_length):
            if i == threshold_pos:
                bar += "|"
            elif i < filled:
                bar += "■"
            else:
                bar += " "
                
        # 高confidence値をカウント
        if confidence >= 0.75:
            self.high_confidence_count += 1
            status = "🔴 [応答トリガー]"
        else:
            status = "⚪"
            
        print(f"\n{status} TT判定 #{self.tt_count}")
        print(f"   Confidence: {confidence:.3f} [{bar}]")
        print(f"   Result: {msg.result}")
        print(f"   時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        
        self.last_tt_time = time.time()
        
    def get_summary(self):
        """統計情報のサマリーを表示"""
        print("\n" + "="*60)
        print("ターンテイキング統計")
        print("="*60)
        print(f"総判定数: {self.tt_count}")
        print(f"高confidence判定数 (≥0.75): {self.high_confidence_count}")
        
        if self.confidence_values:
            avg_confidence = sum(self.confidence_values) / len(self.confidence_values)
            max_confidence = max(self.confidence_values)
            min_confidence = min(self.confidence_values)
            
            print(f"\nConfidence統計:")
            print(f"  平均: {avg_confidence:.3f}")
            print(f"  最大: {max_confidence:.3f}")
            print(f"  最小: {min_confidence:.3f}")
            
            # ヒストグラム
            print("\nConfidence分布:")
            bins = [0, 0.25, 0.5, 0.75, 1.0]
            for i in range(len(bins)-1):
                count = sum(1 for v in self.confidence_values if bins[i] <= v < bins[i+1])
                percentage = (count / len(self.confidence_values)) * 100
                print(f"  {bins[i]:.2f}-{bins[i+1]:.2f}: {count:3d} ({percentage:5.1f}%)")
                
        print("="*60)

def main(args=None):
    rclpy.init(args=args)
    tester = TurnTakingTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_summary()
        print("\nモニタリングを終了しました。")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()