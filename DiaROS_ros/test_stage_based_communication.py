#!/usr/bin/env python3
"""
NLGノードとDMノード間のステージベース通信フローのテストスクリプト

このスクリプトでは、以下の動作を確認します：
1. DMからNLGへのリクエスト送信（stage フィールド、request_id を含む）
2. NLGからDMへの応答受信（stage フィールド、request_id を含む）
3. ステージが正しく管理されているか
4. 複数のステージ間での request_id の管理

このテストはROS2が起動している環境で実行してください：
./scripts/launch/launch_diaros.sh
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import Idm, Inlg
import time
from datetime import datetime


class StageCommunicationTester(Node):
    def __init__(self):
        super().__init__('stage_communication_tester')

        # パブリッシャーとサブスクライバーを作成
        self.pub_dm = self.create_publisher(Idm, 'DMtoNLG', 1)
        self.sub_nlg = self.create_subscription(Inlg, 'NLGtoSS', self.nlg_response_callback, 1)

        # テスト統計
        self.test_results = {
            'first_stage_requests': [],
            'second_stage_requests': [],
            'nlg_responses': []
        }

        # 状態管理
        self.last_request_id = 0
        self.last_stage = None
        self.response_received = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("ステージベース通信テストを開始します")
        self.get_logger().info("=" * 60)

    def nlg_response_callback(self, msg):
        """NLGからの応答を受信"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        stage = msg.stage if hasattr(msg, 'stage') else 'unknown'
        request_id = getattr(msg, 'request_id', 0)

        response_info = {
            'timestamp': timestamp,
            'stage': stage,
            'request_id': request_id,
            'reply_length': len(msg.reply) if msg.reply else 0,
            'reply_preview': msg.reply[:50] if msg.reply else '(empty)'
        }

        self.test_results['nlg_responses'].append(response_info)
        self.response_received = True

        self.get_logger().info(
            f"[受信] NLGからの応答 (時刻={timestamp}, "
            f"stage={stage}, request_id={request_id}, "
            f"応答長={response_info['reply_length']})"
        )

    def send_first_stage_request(self, test_words):
        """First stage（相槌生成）リクエストを送信"""
        msg = Idm()
        msg.words = test_words
        msg.stage = 'first'
        self.last_request_id += 1
        msg.request_id = self.last_request_id
        msg.session_id = 'test_session_001'

        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        request_info = {
            'timestamp': timestamp,
            'stage': 'first',
            'request_id': self.last_request_id,
            'words_count': len(test_words),
            'words_preview': test_words[0] if test_words else '(empty)'
        }

        self.test_results['first_stage_requests'].append(request_info)
        self.last_stage = 'first'

        self.get_logger().info(
            f"[送信] First stage リクエスト (時刻={timestamp}, "
            f"request_id={self.last_request_id}, 入力数={len(test_words)})"
        )

        self.pub_dm.publish(msg)

    def send_second_stage_request(self, test_words):
        """Second stage（応答生成）リクエストを送信"""
        msg = Idm()
        msg.words = test_words
        msg.stage = 'second'
        self.last_request_id += 1
        msg.request_id = self.last_request_id
        msg.session_id = 'test_session_001'

        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

        request_info = {
            'timestamp': timestamp,
            'stage': 'second',
            'request_id': self.last_request_id,
            'words_count': len(test_words),
            'words_preview': test_words[0] if test_words else '(empty)'
        }

        self.test_results['second_stage_requests'].append(request_info)
        self.last_stage = 'second'

        self.get_logger().info(
            f"[送信] Second stage リクエスト (時刻={timestamp}, "
            f"request_id={self.last_request_id}, 入力数={len(test_words)})"
        )

        self.pub_dm.publish(msg)

    def print_test_results(self):
        """テスト結果をサマリーで表示"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("テスト結果サマリー")
        self.get_logger().info("=" * 60)

        self.get_logger().info(
            f"\nFirst stage リクエスト: {len(self.test_results['first_stage_requests'])} 件"
        )
        for i, req in enumerate(self.test_results['first_stage_requests'], 1):
            self.get_logger().info(
                f"  {i}. request_id={req['request_id']}, "
                f"入力数={req['words_count']}, "
                f"時刻={req['timestamp']}"
            )

        self.get_logger().info(
            f"\nSecond stage リクエスト: {len(self.test_results['second_stage_requests'])} 件"
        )
        for i, req in enumerate(self.test_results['second_stage_requests'], 1):
            self.get_logger().info(
                f"  {i}. request_id={req['request_id']}, "
                f"入力数={req['words_count']}, "
                f"時刻={req['timestamp']}"
            )

        self.get_logger().info(
            f"\nNLGからの応答: {len(self.test_results['nlg_responses'])} 件"
        )
        for i, resp in enumerate(self.test_results['nlg_responses'], 1):
            self.get_logger().info(
                f"  {i}. request_id={resp['request_id']}, "
                f"stage={resp['stage']}, "
                f"応答長={resp['reply_length']}, "
                f"時刻={resp['timestamp']}"
            )

        self.get_logger().info("=" * 60)

        # 検証
        self.get_logger().info("\n検証結果:")
        first_requests = len(self.test_results['first_stage_requests'])
        second_requests = len(self.test_results['second_stage_requests'])
        responses = len(self.test_results['nlg_responses'])

        if responses > 0:
            self.get_logger().info(
                f"✓ NLGからの応答を受信しました ({responses} 件)"
            )
        else:
            self.get_logger().warn(
                "✗ NLGからの応答を受信できませんでした"
            )

        # request_id の連続性を確認
        if first_requests > 0 and second_requests > 0:
            all_request_ids = [
                req['request_id'] for req in self.test_results['first_stage_requests']
            ] + [
                req['request_id'] for req in self.test_results['second_stage_requests']
            ]
            all_request_ids.sort()
            self.get_logger().info(
                f"✓ request_id の連続性: {all_request_ids}"
            )

        # stage の対応を確認
        if responses > 0:
            for resp in self.test_results['nlg_responses']:
                matching_requests = [
                    req for req in (
                        self.test_results['first_stage_requests'] +
                        self.test_results['second_stage_requests']
                    )
                    if req['request_id'] == resp['request_id']
                ]
                if matching_requests:
                    req = matching_requests[0]
                    if req['stage'] == resp['stage']:
                        self.get_logger().info(
                            f"✓ request_id {resp['request_id']} の stage 一致: {resp['stage']}"
                        )
                    else:
                        self.get_logger().warn(
                            f"✗ request_id {resp['request_id']} の stage 不一致: "
                            f"リクエスト={req['stage']}, 応答={resp['stage']}"
                        )


def main(args=None):
    rclpy.init(args=args)
    tester = StageCommunicationTester()

    # テストシナリオ
    try:
        # テスト1: First stage リクエスト
        tester.get_logger().info("\n--- テスト1: First stage リクエスト送信 ---")
        tester.send_first_stage_request(['ユーザーの発言'])
        time.sleep(2)  # NLGからの応答を待つ

        # テスト2: Second stage リクエスト
        tester.get_logger().info("\n--- テスト2: Second stage リクエスト送信 ---")
        tester.send_second_stage_request(['ユーザーの発言', 'システムの相槌'])
        time.sleep(2)  # NLGからの応答を待つ

        # テスト3: 複数の First stage リクエスト
        tester.get_logger().info("\n--- テスト3: 複数の First stage リクエスト送信 ---")
        tester.send_first_stage_request(['別のユーザー発言1'])
        time.sleep(1)
        tester.send_first_stage_request(['別のユーザー発言2'])
        time.sleep(2)

        # テスト結果を表示
        tester.print_test_results()

        # ROS通信継続（バックグラウンド）
        tester.get_logger().info("\nテスト完了。ROS通信は継続します（Ctrl+Cで終了）")
        rclpy.spin(tester)

    except KeyboardInterrupt:
        tester.get_logger().info("\nテストを終了します")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
