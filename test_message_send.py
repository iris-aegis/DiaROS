#!/usr/bin/env python3
"""
ROS2メッセージ送信テスト
"""
import sys
import time
import rclpy
from rclpy.node import Node
from interfaces.msg import Idm

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(Idm, 'DMtoNLG', 10)
        self.timer = self.create_timer(2.0, self.publish_test_message)
        self.msg_count = 0

    def publish_test_message(self):
        msg = Idm()
        test_words = ["こんにちは", "今日は", "良い天気", "ですね"]
        msg.words = test_words
        msg.session_id = f"test_session_{self.msg_count}"
        
        self.publisher.publish(msg)
        self.get_logger().info(f'テストメッセージ送信: {test_words}')
        
        self.msg_count += 1
        if self.msg_count >= 3:  # 3回送信したら終了
            self.get_logger().info('テスト完了')
            rclpy.shutdown()

def main():
    print("[テスト] ROS2メッセージ送信テスト開始")
    
    try:
        rclpy.init()
        
        test_pub = TestPublisher()
        
        print("[テスト] DMtoNLGトピックにテストメッセージを送信中...")
        rclpy.spin(test_pub)
        
        print("[テスト] ✅ メッセージ送信テスト完了")
        
    except Exception as e:
        print(f"[テスト] ❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()