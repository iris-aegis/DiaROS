#!/usr/bin/env python3
"""
test_diaros_response.py - DiaROS応答テストツール
ROS2ノードの状態を確認し、音声認識から音声合成までの流れをテストします。
"""

import rclpy
from rclpy.node import Node
import time
import sys
import os
import subprocess
from datetime import datetime

class DiaROSResponseTest(Node):
    def __init__(self):
        super().__init__('diaros_response_test')
        self.get_logger().info('DiaROS応答テストを開始します...')
        
        # テスト結果を記録
        self.test_results = {
            'node_check': False,
            'asr_flow': False,
            'dm_flow': False,
            'nlg_flow': False,
            'ss_flow': False,
            'audio_synthesis': False
        }
        
        # タイマーで定期的にチェック
        self.timer = self.create_timer(2.0, self.run_tests)
        self.test_count = 0
        self.max_tests = 10
        
    def check_ros2_nodes(self):
        """ROS2ノードの起動状態を確認"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                    capture_output=True, text=True)
            nodes = result.stdout.strip().split('\n')
            
            required_nodes = [
                '/acoustic_analysis',
                '/automatic_speech_recognition',
                '/dialog_management',
                '/speech_synthesis',
                '/turn_taking',
                '/back_channel'
            ]
            
            missing_nodes = []
            for node in required_nodes:
                if node not in nodes:
                    missing_nodes.append(node)
            
            if missing_nodes:
                self.get_logger().warn(f'以下のノードが起動していません: {missing_nodes}')
                return False
            else:
                self.get_logger().info('✅ 全ての必要なノードが起動しています')
                self.test_results['node_check'] = True
                return True
                
        except Exception as e:
            self.get_logger().error(f'ノードチェックでエラー: {e}')
            return False
    
    def check_topic_flow(self):
        """トピックの通信フローを確認"""
        try:
            # ASR -> DM のフローを確認
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                    capture_output=True, text=True)
            topics = result.stdout.strip().split('\n')
            
            # 必要なトピックのチェック
            flow_topics = {
                'ASRtoDM': 'asr_flow',
                'DMtoNLG': 'dm_flow',
                'NLGtoDM': 'nlg_flow',
                'DMtoSS': 'ss_flow'
            }
            
            for topic, result_key in flow_topics.items():
                if f'/{topic}' in topics:
                    self.get_logger().info(f'✅ トピック /{topic} が存在します')
                    self.test_results[result_key] = True
                else:
                    self.get_logger().warn(f'❌ トピック /{topic} が見つかりません')
                    
        except Exception as e:
            self.get_logger().error(f'トピックチェックでエラー: {e}')
    
    def check_audio_synthesis(self):
        """音声合成ファイルの生成を確認"""
        try:
            # tmpディレクトリの音声ファイルをチェック
            tmp_dir = 'tmp'
            if os.path.exists(tmp_dir):
                wav_files = [f for f in os.listdir(tmp_dir) if f.endswith('.wav')]
                if wav_files:
                    self.get_logger().info(f'✅ 音声合成ファイルが生成されています: {len(wav_files)}個')
                    self.test_results['audio_synthesis'] = True
                    # 最新のファイルを表示
                    latest_file = max(wav_files, key=lambda f: os.path.getmtime(os.path.join(tmp_dir, f)))
                    self.get_logger().info(f'   最新ファイル: {latest_file}')
                else:
                    self.get_logger().warn('❌ 音声合成ファイルが見つかりません')
            else:
                self.get_logger().warn('❌ tmpディレクトリが存在しません')
                
        except Exception as e:
            self.get_logger().error(f'音声合成チェックでエラー: {e}')
    
    def run_tests(self):
        """テストを実行"""
        self.test_count += 1
        self.get_logger().info(f'\n=== テスト実行 {self.test_count}/{self.max_tests} ===')
        
        # 各種チェックを実行
        if self.check_ros2_nodes():
            self.check_topic_flow()
            self.check_audio_synthesis()
        
        # 結果サマリーを表示
        if self.test_count >= self.max_tests or all(self.test_results.values()):
            self.print_summary()
            self.destroy_timer(self.timer)
            rclpy.shutdown()
    
    def print_summary(self):
        """テスト結果のサマリーを表示"""
        print('\n' + '='*50)
        print('DiaROS応答テスト結果サマリー')
        print('='*50)
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for v in self.test_results.values() if v)
        
        for test_name, result in self.test_results.items():
            status = '✅ PASS' if result else '❌ FAIL'
            print(f'{test_name:20}: {status}')
        
        print('-'*50)
        print(f'合計: {passed_tests}/{total_tests} テスト合格')
        print(f'成功率: {passed_tests/total_tests*100:.1f}%')
        print('='*50)
        
        if passed_tests == total_tests:
            print('🎉 全てのテストに合格しました！')
        else:
            print('⚠️  一部のテストが失敗しました。ログを確認してください。')

def main():
    print('DiaROS応答テストツール')
    print('このツールはDiaROSシステムの応答フローをテストします。')
    print('-'*50)
    
    rclpy.init()
    test_node = DiaROSResponseTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print('\nテストを中断しました。')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()