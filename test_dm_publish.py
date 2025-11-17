#!/usr/bin/env python3
"""
DMtoNLGトピックにテストメッセージを送信
"""
import rclpy
from rclpy.node import Node
from interfaces.msg import Idm
import time

class TestDMPublisher(Node):
    def __init__(self):
        super().__init__('test_dm_publisher')
        self.publisher = self.create_publisher(Idm, 'DMtoNLG', 10)

    def publish_test_message(self, words):
        msg = Idm()
        msg.words = words
        msg.session_id = 'test_session_001'

        print(f"[テスト] メッセージ送信: {words}")
        self.publisher.publish(msg)
        print("[テスト] メッセージ送信完了")

def main():
    rclpy.init()
    publisher = TestDMPublisher()

    # 少し待機してから送信
    time.sleep(1)

    # テストメッセージ送信
    publisher.publish_test_message(['こんにちは'])

    time.sleep(1)
    publisher.publish_test_message(['今日は', '良い天気', 'ですね'])

    time.sleep(1)
    print("[テスト] テスト完了")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
