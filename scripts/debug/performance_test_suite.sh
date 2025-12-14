#!/bin/bash
# ROS通信とQueue性能総合テストスイート

set -e

echo "=========================================="
echo "🔬 DiaROS性能テストスイート"
echo "=========================================="

# 色設定
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 作業ディレクトリ
DIAROS_DIR="/workspace/DiaROS/DiaROS_ros"
SCRIPT_DIR="/workspace/DiaROS/scripts/debug"

# ROS2環境セットアップ
setup_ros_environment() {
    echo -e "${BLUE}🔧 ROS2環境をセットアップ中...${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}❌ ROS2環境が見つかりません${NC}"
        exit 1
    fi
    
    if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
        source "$DIAROS_DIR/install/local_setup.bash"
    else
        echo -e "${RED}❌ DiaROSビルドが見つかりません${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ ROS2環境セットアップ完了${NC}"
}

# テストオプション表示
show_test_options() {
    echo ""
    echo -e "${YELLOW}📋 利用可能なテストオプション:${NC}"
    echo "=========================================="
    echo "1. 基本トピック情報確認"
    echo "2. リアルタイム帯域幅・レート監視"
    echo "3. Python性能監視スクリプト起動"
    echo "4. 総合監視ダッシュボード"
    echo "5. ログファイル解析"
    echo "6. 負荷テスト実行"
    echo "7. 終了"
    echo "=========================================="
}

# 基本トピック情報確認
test_basic_info() {
    echo -e "${BLUE}📊 基本トピック情報を確認中...${NC}"
    bash "$SCRIPT_DIR/check_topic_bandwidth.sh"
}

# リアルタイム監視
test_realtime_monitoring() {
    echo -e "${BLUE}📡 リアルタイム監視を開始...${NC}"
    echo "主要トピックの監視（Ctrl+Cで終了）:"
    
    # 複数のターミナルでの監視をシミュレート
    echo -e "${YELLOW}DMtoNLGトピック監視:${NC}"
    timeout 30 ros2 topic hz /DMtoNLG &
    PID1=$!
    
    echo -e "${YELLOW}NLGtoSSトピック監視:${NC}"
    timeout 30 ros2 topic hz /NLGtoSS &
    PID2=$!
    
    echo -e "${YELLOW}帯域幅監視:${NC}"
    timeout 30 ros2 topic bw /DMtoNLG &
    PID3=$!
    
    # 30秒間監視
    wait $PID1 $PID2 $PID3 2>/dev/null
    echo -e "${GREEN}✅ リアルタイム監視完了${NC}"
}

# Python性能監視スクリプト起動
test_python_monitoring() {
    echo -e "${BLUE}🐍 Python性能監視スクリプトを起動...${NC}"
    echo "詳細な性能統計とボトルネック検出を開始"
    echo "(Ctrl+Cで終了)"
    
    python3 "$SCRIPT_DIR/monitor_ros_performance.py"
}

# 総合監視ダッシュボード
test_dashboard() {
    echo -e "${BLUE}📈 総合監視ダッシュボードを起動...${NC}"
    
    # tmuxが利用可能かチェック
    if command -v tmux >/dev/null 2>&1; then
        echo "tmux仮想端末で複数の監視を同時実行"
        
        # tmuxセッション作成
        tmux new-session -d -s diaros_monitoring
        
        # 複数のペインで異なる監視を実行
        tmux send-keys -t diaros_monitoring "python3 $SCRIPT_DIR/monitor_ros_performance.py" Enter
        tmux split-window -v -t diaros_monitoring
        tmux send-keys -t diaros_monitoring "ros2 topic hz /DMtoNLG" Enter
        tmux split-window -h -t diaros_monitoring
        tmux send-keys -t diaros_monitoring "ros2 topic bw /DMtoNLG" Enter
        
        # tmuxセッションにアタッチ
        tmux attach-session -t diaros_monitoring
    else
        echo "tmuxが利用できません。Pythonスクリプトを単体実行"
        test_python_monitoring
    fi
}

# ログファイル解析
test_log_analysis() {
    echo -e "${BLUE}📄 ログファイル解析...${NC}"
    
    LOG_FILES=$(find /tmp -name "ros_performance_*.log" -type f 2>/dev/null | head -5)
    
    if [ -z "$LOG_FILES" ]; then
        echo -e "${YELLOW}⚠️  性能ログファイルが見つかりません${NC}"
        echo "先にPython監視スクリプトを実行してください"
        return
    fi
    
    echo "利用可能なログファイル:"
    echo "$LOG_FILES"
    
    # 最新のログファイルを解析
    LATEST_LOG=$(echo "$LOG_FILES" | head -1)
    echo -e "${YELLOW}最新ログファイル解析: $LATEST_LOG${NC}"
    
    if [ -f "$LATEST_LOG" ]; then
        echo "統計サマリー:"
        echo "----------------------------------------"
        echo "総メッセージ数:"
        tail -n +2 "$LATEST_LOG" | wc -l
        
        echo "トピック別統計:"
        tail -n +2 "$LATEST_LOG" | cut -d',' -f2 | sort | uniq -c
        
        echo "Queue詰まり警告数:"
        tail -n +2 "$LATEST_LOG" | grep "True" | wc -l
        
        echo "平均メッセージサイズ（DMtoNLG）:"
        tail -n +2 "$LATEST_LOG" | grep "DMtoNLG" | cut -d',' -f3 | awk '{sum+=$1; count++} END {if(count>0) print sum/count " 個"}'
        
        echo "最大間隔（ms）:"
        tail -n +2 "$LATEST_LOG" | cut -d',' -f4 | sort -n | tail -1
    fi
}

# 負荷テスト
test_load_testing() {
    echo -e "${BLUE}🔥 負荷テスト実行...${NC}"
    echo "大量の音声認識履歴送信をシミュレート"
    
    # 負荷テスト用のPythonスクリプトを作成・実行
    cat > /tmp/load_test.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import Idm
import time
import random

class LoadTester(Node):
    def __init__(self):
        super().__init__('load_tester')
        self.publisher = self.create_publisher(Idm, 'DMtoNLG', 10)
        self.timer = self.create_timer(0.1, self.send_large_message)
        self.counter = 0
        
    def send_large_message(self):
        msg = Idm()
        # 大量の履歴をシミュレート（100-500個の履歴）
        large_history = [f"テスト音声認識結果{i}" for i in range(random.randint(100, 500))]
        msg.words = large_history
        self.publisher.publish(msg)
        self.counter += 1
        
        if self.counter % 10 == 0:
            print(f"負荷テスト: {self.counter}回送信完了（履歴数: {len(large_history)}）")
        
        if self.counter >= 100:  # 100回送信で終了
            print("負荷テスト完了")
            rclpy.shutdown()

def main():
    rclpy.init()
    tester = LoadTester()
    rclpy.spin(tester)

if __name__ == '__main__':
    main()
EOF
    
    echo "負荷テストスクリプトを30秒間実行..."
    timeout 30 python3 /tmp/load_test.py &
    LOAD_PID=$!
    
    # 同時に監視も実行
    timeout 30 python3 "$SCRIPT_DIR/monitor_ros_performance.py" &
    MONITOR_PID=$!
    
    wait $LOAD_PID $MONITOR_PID 2>/dev/null
    
    # クリーンアップ
    rm -f /tmp/load_test.py
    
    echo -e "${GREEN}✅ 負荷テスト完了${NC}"
}

# メイン実行部分
main() {
    setup_ros_environment
    
    while true; do
        show_test_options
        echo ""
        read -p "テスト番号を選択してください (1-7): " choice
        
        case $choice in
            1)
                test_basic_info
                ;;
            2)
                test_realtime_monitoring
                ;;
            3)
                test_python_monitoring
                ;;
            4)
                test_dashboard
                ;;
            5)
                test_log_analysis
                ;;
            6)
                test_load_testing
                ;;
            7)
                echo -e "${GREEN}🏁 テスト終了${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}❌ 無効な選択です${NC}"
                ;;
        esac
        
        echo ""
        read -p "続行するにはEnterキーを押してください..."
    done
}

# スクリプト実行
main "$@"