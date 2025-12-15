#!/bin/bash
# DMtoNLGトピックの詰まり・遅延監視スクリプト（シンプル版）

set -e

echo "=========================================="
echo "🔍 DMtoNLG通信監視（詰まり・遅延チェック）"
echo "=========================================="

# ROS2環境セットアップ
echo "🔧 ROS2環境をセットアップ中..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble環境をロード"
else
    echo "❌ ROS2 Humble環境が見つかりません"
    exit 1
fi

# DiaROSローカル環境セットアップ
DIAROS_DIR="/workspace/DiaROS_ros"
if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_DIR/install/local_setup.bash"
    echo "✅ DiaROSローカル環境をロード"
else
    echo "❌ DiaROSビルドが見つかりません: $DIAROS_DIR/install/local_setup.bash"
    exit 1
fi

echo ""
echo "📡 DMtoNLGトピック監視開始..."
echo "=========================================="
echo "監視項目:"
echo "  - メッセージ送信間隔（遅延検出）"
echo "  - 履歴数（大容量メッセージ検出）"
echo "  - Queue詰まり警告（1000ms以上の間隔）"
echo ""
echo "Ctrl+C で終了"
echo "=========================================="

# DMtoNLGトピックの存在確認
if ! ros2 topic list | grep -q "^/DMtoNLG$"; then
    echo "⚠️  /DMtoNLGトピックが見つかりません"
    echo "DiaROSが起動していることを確認してください"
    exit 1
fi

# 監視用Pythonスクリプトを一時作成
cat > /tmp/dmtonlg_monitor.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import Idm
import time
from datetime import datetime

class DMtoNLGMonitor(Node):
    def __init__(self):
        super().__init__('dmtonlg_monitor')
        self.subscription = self.create_subscription(
            Idm, 'DMtoNLG', self.callback, 10
        )
        self.last_timestamp = None
        self.message_count = 0
        
        print("🚀 DMtoNLG監視開始")
        print("時刻        | 間隔(ms) | 履歴数 | 状態")
        print("-" * 50)
    
    def callback(self, msg):
        now = time.time()
        current_time = datetime.fromtimestamp(now).strftime('%H:%M:%S.%f')[:-3]
        
        word_count = len(msg.words)
        self.message_count += 1
        
        if self.last_timestamp is not None:
            interval_ms = (now - self.last_timestamp) * 1000
            
            # 状態判定
            if interval_ms > 1000:
                status = "⚠️ 詰まり"
            elif interval_ms > 500:
                status = "⚡ 遅延"
            elif word_count > 50:
                status = "📦 大容量"
            else:
                status = "✅ 正常"
            
            print(f"{current_time} | {interval_ms:7.1f} | {word_count:6d} | {status}")
            
        else:
            print(f"{current_time} |    初回   | {word_count:6d} | 🚀 開始")
        
        self.last_timestamp = now

def main():
    rclpy.init()
    monitor = DMtoNLGMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n📊 監視終了")
        print(f"総メッセージ数: {monitor.message_count}")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

# Pythonスクリプト実行
python3 /tmp/dmtonlg_monitor.py

# クリーンアップ
rm -f /tmp/dmtonlg_monitor.py

echo ""
echo "🏁 DMtoNLG監視完了"