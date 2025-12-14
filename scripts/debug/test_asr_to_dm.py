#!/usr/bin/env python3
"""
test_asr_to_dm.py - ASRからDMへの対話フローテストツール
音声認識（ASR）から対話管理（DM）への情報伝達と応答生成をテストします。
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Iasr, Idm
import time
from datetime import datetime

class ASRtoDMTester(Node):
    def __init__(self):
        super().__init__('asr_to_dm_tester')
        
        # パブリッシャー（ASR結果をNLUtoDMトピックに送信）
        self.pub_asr = self.create_publisher(Iasr, 'NLUtoDM', 10)
        
        # サブスクライバー（DMからの出力を監視）
        self.sub_dm_to_nlg = self.create_subscription(
            Idm, 'DMtoNLG', 
            self.dm_output_callback, 10)
        
        # テスト用の発話
        self.test_utterances = [
            ("こんにちは", True),
            ("今日はいい天気ですね", True),
            ("そうですね", True),
            ("ありがとうございます", True),
        ]
        self.test_index = 0
        
        # 統計情報
        self.asr_sent = 0
        self.dm_received = 0
        self.valid_dm_received = 0
        
        # 5秒ごとにASRメッセージを送信
        self.test_timer = self.create_timer(5.0, self.send_test_asr)
        
        print("\n" + "="*60)
        print("ASR→DM対話フローテスト")
        print("="*60)
        print("5秒ごとにテストASRメッセージを送信します")
        print("Ctrl+Cで終了")
        print("-"*60 + "\n")
        
    def send_test_asr(self):
        """テスト用のASRメッセージを送信"""
        utterance, is_final = self.test_utterances[self.test_index]
        
        msg = Iasr()
        msg.you = utterance
        msg.is_final = is_final
        
        self.pub_asr.publish(msg)
        self.asr_sent += 1
        
        print(f"\n🎤 [ASR送信] '{msg.you}' (is_final: {msg.is_final})")
        print(f"   送信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        
        # 次の発話へ
        self.test_index = (self.test_index + 1) % len(self.test_utterances)
        
    def dm_output_callback(self, msg):
        """DMからの出力を受信"""
        self.dm_received += 1
        
        # 空でない要素のみフィルタリング
        non_empty_words = [w for w in msg.words if w and w.strip()]
        
        if non_empty_words:
            self.valid_dm_received += 1
            print(f"\n💭 [DM出力] words: {non_empty_words}")
            print(f"   受信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        else:
            print(f"\n⚠️  [DM出力] 空のwordsを受信")
            print(f"   受信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
            
    def get_summary(self):
        """テスト結果のサマリーを表示"""
        print("\n" + "="*60)
        print("テスト結果サマリー")
        print("="*60)
        print(f"ASR送信数: {self.asr_sent}")
        print(f"DM受信数: {self.dm_received} (有効: {self.valid_dm_received})")
        
        if self.asr_sent > 0:
            response_rate = (self.valid_dm_received / self.asr_sent) * 100
            print(f"DM応答率: {response_rate:.1f}%")
            
        print("="*60)

def main(args=None):
    rclpy.init(args=args)
    tester = ASRtoDMTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_summary()
        print("\nテストを終了しました。")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()