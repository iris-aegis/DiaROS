#!/usr/bin/env python3
"""
ROS2 NLGノードのテスト
"""
import os
import sys
import subprocess
import time
import threading

def test_ros2_node():
    print("[テスト] ROS2 NLGノード起動テスト開始")
    
    # 環境設定
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '0'
    
    try:
        # ROS2セットアップ
        setup_cmd = "source /opt/ros/humble/setup.bash && source ./install/local_setup.bash"
        
        # ROS2 NLGノード起動
        cmd = f"{setup_cmd} && ros2 run diaros_package ros2_natural_language_generation"
        
        print(f"[テスト] コマンド実行: {cmd}")
        
        # タイムアウト付きでプロセス起動
        proc = subprocess.Popen(
            cmd, 
            shell=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
            cwd="/workspace/DiaROS/DiaROS_ros"
        )
        
        # 10秒間出力を監視
        timeout = 10
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if proc.poll() is not None:
                break
            
            # ノンブロッキングで出力を読み取り
            try:
                output = proc.stdout.readline()
                if output:
                    print(f"[NLGノード] {output.strip()}")
            except:
                pass
            
            time.sleep(0.1)
        
        # プロセス終了
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=5)
            print("[テスト] ✅ NLGノードが正常に起動しました（タイムアウトで終了）")
        else:
            return_code = proc.returncode
            if return_code == 0:
                print("[テスト] ✅ NLGノードが正常に完了しました")
            else:
                print(f"[テスト] ❌ NLGノードがエラーで終了しました (return code: {return_code})")
        
    except Exception as e:
        print(f"[テスト] ❌ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_ros2_node()