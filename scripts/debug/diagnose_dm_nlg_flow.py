#!/usr/bin/env python3
"""
DM→NLGトピック通信診断ツール
"""

import rclpy
import time
import threading
from rclpy.node import Node
from interfaces.msg import Iasr, Idm
from std_msgs.msg import String

class DMNLGFlowDiagnostic(Node):
    def __init__(self):
        super().__init__('dm_nlg_flow_diagnostic')
        
        # サブスクライバー
        self.sub_asr = self.create_subscription(Iasr, 'NLUtoDM', self.asr_callback, 1)
        self.sub_dm = self.create_subscription(Idm, 'DMtoNLG', self.dm_callback, 1)
        
        # パブリッシャー（テスト用）
        self.pub_asr_test = self.create_publisher(Iasr, 'NLUtoDM', 1)
        
        # 統計データ
        self.asr_count = 0
        self.dm_count = 0
        self.last_asr_msg = None
        self.last_dm_msg = None
        
        print("🔍 DM→NLG トピック通信診断ツール開始")
        print("=" * 50)
        print("監視中のトピック:")
        print("  - NLUtoDM (ASR→DM)")
        print("  - DMtoNLG (DM→NLG)")
        print("=" * 50)
    
    def asr_callback(self, msg):
        self.asr_count += 1
        self.last_asr_msg = msg
        timestamp = time.strftime('%H:%M:%S.%f')[:-3]
        print(f"[{timestamp}] ✅ ASR→DM受信 #{self.asr_count}: '{msg.you}' (is_final: {msg.is_final})")
    
    def dm_callback(self, msg):
        self.dm_count += 1
        self.last_dm_msg = msg
        timestamp = time.strftime('%H:%M:%S.%f')[:-3]
        words_str = ', '.join([f"'{w}'" for w in msg.words if w])
        print(f"[{timestamp}] 🚀 DM→NLG送信 #{self.dm_count}: [{words_str}]")
        
        if not any(msg.words):
            print("    ⚠️  警告: 空のwords配列が送信されました")
    
    def send_test_asr(self, text="テスト音声認識結果"):
        """テスト用ASRメッセージ送信"""
        test_msg = Iasr()
        test_msg.you = text
        test_msg.is_final = True
        
        self.pub_asr_test.publish(test_msg)
        timestamp = time.strftime('%H:%M:%S.%f')[:-3]
        print(f"[{timestamp}] 📤 テストASRメッセージ送信: '{text}'")
    
    def print_statistics(self):
        """統計情報出力"""
        print(f"\n📊 統計情報:")
        print(f"  ASR→DM受信回数: {self.asr_count}")
        print(f"  DM→NLG送信回数: {self.dm_count}")
        
        if self.asr_count > 0 and self.dm_count == 0:
            print("  ❌ 問題: ASRは受信しているがDMからNLGへ送信されていません")
        elif self.asr_count == 0:
            print("  ❌ 問題: ASRメッセージが受信されていません")
        elif self.dm_count > 0:
            print("  ✅ DM→NLG通信は正常に動作しています")
        
        if self.last_asr_msg:
            print(f"  最新ASR: '{self.last_asr_msg.you}'")
        
        if self.last_dm_msg:
            words = [w for w in self.last_dm_msg.words if w]
            print(f"  最新DM: {words}")

def main():
    rclpy.init()
    
    diagnostic = DMNLGFlowDiagnostic()
    
    # 別スレッドでROS2を実行
    def spin_thread():
        rclpy.spin(diagnostic)
    
    thread = threading.Thread(target=spin_thread)
    thread.daemon = True
    thread.start()
    
    # メインループ
    try:
        print("\n🔧 診断コマンド:")
        print("  t + Enter: テストASRメッセージ送信")
        print("  s + Enter: 統計情報表示")
        print("  q + Enter: 終了")
        print()
        
        while True:
            command = input("診断コマンド (t/s/q): ").strip().lower()
            
            if command == 't':
                text = input("テストメッセージ (Enterでデフォルト): ").strip()
                if not text:
                    text = f"テスト音声認識結果 {int(time.time())}"
                diagnostic.send_test_asr(text)
                
            elif command == 's':
                diagnostic.print_statistics()
                
            elif command == 'q':
                print("診断ツールを終了します...")
                break
                
            else:
                print("無効なコマンドです。t/s/q のいずれかを入力してください。")
    
    except KeyboardInterrupt:
        print("\n診断ツールを終了します...")
    
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()