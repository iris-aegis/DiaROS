#!/usr/bin/env python3
"""
完全な通信フローテスト
"""
import subprocess
import time
import threading
import os
import signal

class DiaROSFlowTester:
    def __init__(self):
        self.nlg_process = None
        self.test_completed = False
        
    def start_nlg_node(self):
        """NLGノードを起動"""
        print("[テスト] NLGノードを起動中...")
        
        env = os.environ.copy()
        
        cmd = "source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && ros2 launch diaros_package sdsmod.launch.py"
        
        self.nlg_process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
            cwd="/workspace/DiaROS/DiaROS_ros",
            executable='/bin/bash'
        )
        
        # 起動待機
        time.sleep(3)
        print("[テスト] ✅ NLGノード起動完了")
        
    def monitor_nlg_output(self):
        """NLG出力を監視"""
        if not self.nlg_process:
            return
            
        print("[テスト] NLG出力監視開始")
        
        while not self.test_completed and self.nlg_process.poll() is None:
            try:
                output = self.nlg_process.stdout.readline()
                if output:
                    print(f"[NLG] {output.strip()}")
                    
                    # 応答生成完了のメッセージを検出
                    if "送信する対話生成内容:" in output:
                        print("[テスト] ✅ 対話生成成功を検出!")
                        self.test_completed = True
                        
            except:
                pass
                
            time.sleep(0.1)
    
    def send_test_message(self):
        """テストメッセージを送信"""
        print("[テスト] 3秒後にテストメッセージを送信...")
        time.sleep(3)
        
        env = os.environ.copy()
        
        # テストメッセージ送信コマンド
        test_cmd = """
source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && 
ros2 topic pub --once /DMtoNLG interfaces/msg/Idm '{words: ["こんにちは", "今日は", "良い天気", "ですね"], session_id: "test_session_001"}'
"""
        
        print("[テスト] DMtoNLGトピックにメッセージ送信中...")
        
        result = subprocess.run(
            test_cmd,
            shell=True,
            capture_output=True,
            text=True,
            env=env,
            cwd="/workspace/DiaROS/DiaROS_ros",
            executable='/bin/bash'
        )
        
        if result.returncode == 0:
            print("[テスト] ✅ メッセージ送信成功")
        else:
            print(f"[テスト] ❌ メッセージ送信失敗: {result.stderr}")
    
    def run_test(self):
        """テスト実行"""
        try:
            # NLGノード起動
            self.start_nlg_node()
            
            # 出力監視スレッド開始
            monitor_thread = threading.Thread(target=self.monitor_nlg_output)
            monitor_thread.daemon = True
            monitor_thread.start()
            
            # テストメッセージ送信
            self.send_test_message()
            
            # 結果待機（最大10秒）
            wait_time = 0
            while not self.test_completed and wait_time < 10:
                time.sleep(0.5)
                wait_time += 0.5
            
            if self.test_completed:
                print("[テスト] ✅ フロー全体のテスト成功!")
            else:
                print("[テスト] ⚠️  タイムアウト: 応答生成が確認できませんでした")
                
        except Exception as e:
            print(f"[テスト] ❌ エラー: {e}")
            
        finally:
            # プロセス終了
            if self.nlg_process and self.nlg_process.poll() is None:
                self.nlg_process.terminate()
                self.nlg_process.wait(timeout=5)
                print("[テスト] NLGノード終了")

def main():
    tester = DiaROSFlowTester()
    tester.run_test()

if __name__ == "__main__":
    main()