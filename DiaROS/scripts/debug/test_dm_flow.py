#!/usr/bin/env python3
"""
test_dm_flow.py - DMからNLGへの対話フローテストツール
対話管理（DM）から自然言語生成（NLG）への応答生成フローを詳細にテストします。
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Iasr, Idm, Inlg
from std_msgs.msg import String
import time
from datetime import datetime

class DMFlowTester(Node):
    def __init__(self):
        super().__init__('dm_flow_tester')
        
        # パブリッシャー（テスト用のASRメッセージを送信）
        self.pub_asr = self.create_publisher(Iasr, 'NLUtoDM', 10)
        
        # サブスクライバー（DMとNLGの応答を監視）
        self.sub_dm_to_nlg = self.create_subscription(
            Idm, 'DMtoNLG', 
            self.dm_to_nlg_callback, 10)
        
        self.sub_nlg_to_dm = self.create_subscription(
            Inlg, 'NLGtoDM', 
            self.nlg_to_dm_callback, 10)
        
        # テストシナリオ
        self.test_phrases = [
            "こんにちは",
            "今日はいい天気ですね",
            "最近どうですか",
            "何か面白い話はありますか",
            "ありがとう"
        ]
        self.test_index = 0
        
        # テスト用タイマー（3秒ごとにASRメッセージを送信）
        self.test_timer = self.create_timer(3.0, self.send_test_asr)
        
        # 統計情報
        self.stats = {
            'asr_sent': 0,
            'dm_received': 0,
            'nlg_received': 0,
            'empty_dm': 0,
            'valid_dm': 0
        }
        
        print("\n" + "="*60)
        print("DM→NLG対話フローテスト")
        print("="*60)
        print("3秒ごとにテストASRメッセージを送信します")
        print("Ctrl+Cで終了")
        print("-"*60 + "\n")
        
    def send_test_asr(self):
        """テスト用のASRメッセージを送信"""
        msg = Iasr()
        msg.you = self.test_phrases[self.test_index]
        msg.is_final = True
        
        self.pub_asr.publish(msg)
        self.stats['asr_sent'] += 1
        
        print(f"\n🎤 [テストASR送信] '{msg.you}' (is_final: {msg.is_final})")
        print(f"   送信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        
        # 次のフレーズへ
        self.test_index = (self.test_index + 1) % len(self.test_phrases)
        
    def dm_to_nlg_callback(self, msg):
        """DM→NLGメッセージを受信"""
        self.stats['dm_received'] += 1
        
        # wordsフィールドの内容を確認
        if msg.words and any(word.strip() for word in msg.words):
            self.stats['valid_dm'] += 1
            print(f"\n💭 [DM→NLG受信] words配列:")
            for i, word in enumerate(msg.words):
                if word.strip():  # 空でない要素のみ表示
                    print(f"   [{i}]: '{word}'")
            print(f"   受信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        else:
            self.stats['empty_dm'] += 1
            print(f"\n⚠️  [DM→NLG受信] 空のwords配列を受信")
            print(f"   受信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
            
    def nlg_to_dm_callback(self, msg):
        """NLG→DMメッセージを受信"""
        self.stats['nlg_received'] += 1
        print(f"\n🤖 [NLG応答] '{msg.response}'")
        print(f"   受信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        
    def get_summary(self):
        """テスト結果のサマリーを表示"""
        print("\n" + "="*60)
        print("テスト結果サマリー")
        print("="*60)
        print(f"ASR送信数: {self.stats['asr_sent']}")
        print(f"DM受信数: {self.stats['dm_received']} (有効: {self.stats['valid_dm']}, 空: {self.stats['empty_dm']})")
        print(f"NLG応答数: {self.stats['nlg_received']}")
        
        if self.stats['asr_sent'] > 0:
            dm_rate = (self.stats['valid_dm'] / self.stats['asr_sent']) * 100
            nlg_rate = (self.stats['nlg_received'] / self.stats['asr_sent']) * 100
            print(f"\nDM応答率: {dm_rate:.1f}%")
            print(f"NLG応答率: {nlg_rate:.1f}%")
        print("="*60)

def main(args=None):
    rclpy.init(args=args)
    tester = DMFlowTester()
    
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