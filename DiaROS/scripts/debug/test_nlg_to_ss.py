#!/usr/bin/env python3
"""
NLGtoSSトピック監視スクリプト
音声認識結果リストの送受信テスト用
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Inlg
import sys

class NLGtoSSMonitor(Node):
    def __init__(self):
        super().__init__('nlg_to_ss_monitor')
        
        self.subscription = self.create_subscription(
            Inlg, 'NLGtoSS', self.callback, 10
        )
        
        print("🔍 NLGtoSS トピック監視開始")
        print("=" * 60)
        print("監視内容:")
        print("  - 対話生成結果（reply）")
        print("  - 音声認識結果リスト（source_words）")
        print("=" * 60)
    
    def callback(self, msg):
        print(f"📨 NLGtoSS メッセージ受信:")
        print(f"  対話生成結果: '{msg.reply}'")
        print(f"  音声認識履歴数: {len(msg.source_words)}個")
        
        if len(msg.source_words) > 0:
            print(f"  音声認識履歴（全履歴）:")
            for i, word in enumerate(msg.source_words):
                print(f"    [{i+1:3d}] {word}")
        else:
            print("  履歴内容: （空）")
        
        print("-" * 60)

def main():
    rclpy.init()
    monitor = NLGtoSSMonitor()
    
    try:
        print("Ctrl+C で終了")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\\n📊 監視終了")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()