#!/usr/bin/env python3
"""
test_nlg_response.py - NLG応答生成テストツール
DMからNLGへの応答生成要求を送信して、応答が正しく生成されるかテストします。
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Idm, Inlg
import time
from datetime import datetime

class NLGResponseTester(Node):
    def __init__(self):
        super().__init__('nlg_response_tester')
        
        # パブリッシャー（DMtoNLGトピックにテストメッセージを送信）
        self.pub_dm_to_nlg = self.create_publisher(Idm, 'DMtoNLG', 10)
        
        # サブスクライバー（NLGからの応答を監視）
        self.sub_nlg_to_ss = self.create_subscription(
            Inlg, 'NLGtoSS', 
            self.nlg_response_callback, 10)
        
        # テスト用のメッセージ
        self.test_messages = [
            ["こんにちは"],
            ["今日はいい天気ですね"],
            ["最近どうですか"],
            ["ありがとう"]
        ]
        self.test_index = 0
        
        # 統計情報
        self.requests_sent = 0
        self.responses_received = 0
        self.last_request_time = None
        
        # 3秒ごとにテストメッセージを送信
        self.test_timer = self.create_timer(5.0, self.send_test_message)
        
        print("\n" + "="*60)
        print("NLG応答生成テスト")
        print("="*60)
        print("5秒ごとにテストメッセージをDMtoNLGトピックに送信します")
        print("Ctrl+Cで終了")
        print("-"*60 + "\n")
        
    def send_test_message(self):
        """テストメッセージを送信"""
        msg = Idm()
        msg.words = self.test_messages[self.test_index]
        
        self.pub_dm_to_nlg.publish(msg)
        self.requests_sent += 1
        self.last_request_time = time.time()
        
        print(f"\n📤 [送信] DMtoNLGメッセージ: {msg.words}")
        print(f"   送信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        
        # 次のメッセージへ
        self.test_index = (self.test_index + 1) % len(self.test_messages)
        
    def nlg_response_callback(self, msg):
        """NLGからの応答を受信"""
        self.responses_received += 1
        
        if self.last_request_time:
            response_time = (time.time() - self.last_request_time) * 1000
            print(f"\n🤖 [受信] NLG応答: '{msg.reply}'")
            print(f"   応答時間: {response_time:.0f}ms")
            print(f"   受信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        else:
            print(f"\n🤖 [受信] NLG応答: '{msg.reply}'")
            print(f"   受信時刻: {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
            
    def get_summary(self):
        """テスト結果のサマリーを表示"""
        print("\n" + "="*60)
        print("テスト結果サマリー")
        print("="*60)
        print(f"送信リクエスト数: {self.requests_sent}")
        print(f"受信レスポンス数: {self.responses_received}")
        
        if self.requests_sent > 0:
            success_rate = (self.responses_received / self.requests_sent) * 100
            print(f"応答成功率: {success_rate:.1f}%")
            
        print("="*60)

def main(args=None):
    rclpy.init(args=args)
    tester = NLGResponseTester()
    
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