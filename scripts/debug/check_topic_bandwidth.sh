#!/bin/bash
# ROS2トピックの帯域幅とレート監視スクリプト

set -e

echo "=========================================="
echo "🌐 ROS2トピック性能監視"
echo "=========================================="

# ROS2環境セットアップ
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ ROS2環境が見つかりません"
    exit 1
fi

# DiaROSローカル環境セットアップ
DIAROS_DIR="/workspace/DiaROS/DiaROS_ros"
if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_DIR/install/local_setup.bash"
else
    echo "❌ DiaROSビルドが見つかりません"
    exit 1
fi

echo "📊 利用可能なトピック一覧:"
ros2 topic list

echo ""
echo "🔍 主要トピックの詳細情報:"
echo "------------------------------------------"

# 主要なトピックの存在確認と統計表示
TOPICS=("/DMtoNLG" "/NLGtoSS" "/NLUtoDM" "/AAtoDM" "/TTtoDM" "/BCtoDM")

for topic in "${TOPICS[@]}"; do
    if ros2 topic list | grep -q "^$topic$"; then
        echo "📈 $topic:"
        
        # トピック情報を取得
        echo "  型情報:"
        ros2 topic info "$topic" | sed 's/^/    /'
        
        echo "  帯域幅・レート監視を開始します..."
        echo "  (10秒間の統計を取得中...)"
        
        # 10秒間の統計を取得（バックグラウンドで実行）
        timeout 10 ros2 topic bw "$topic" 2>/dev/null &
        BW_PID=$!
        
        timeout 10 ros2 topic hz "$topic" 2>/dev/null &
        HZ_PID=$!
        
        # プロセス完了待機
        wait $BW_PID 2>/dev/null || echo "    帯域幅: データなし"
        wait $HZ_PID 2>/dev/null || echo "    周波数: データなし"
        
        echo ""
    else
        echo "⚠️  $topic: トピックが見つかりません"
    fi
done

echo "=========================================="
echo "🔧 リアルタイム監視オプション:"
echo "------------------------------------------"
echo "特定トピックの詳細監視："
echo "  ros2 topic echo /DMtoNLG    # 音声認識履歴送信内容"
echo "  ros2 topic echo /NLGtoSS    # 対話生成結果"
echo "  ros2 topic bw /DMtoNLG      # DMtoNLGの帯域幅"
echo "  ros2 topic hz /DMtoNLG      # DMtoNLGの送信頻度"
echo ""
echo "Python監視スクリプト："
echo "  python3 /workspace/DiaROS/scripts/debug/monitor_ros_performance.py"
echo ""
echo "全トピック同時監視："
echo "  ros2 topic list | xargs -I {} ros2 topic hz {}"
echo "=========================================="